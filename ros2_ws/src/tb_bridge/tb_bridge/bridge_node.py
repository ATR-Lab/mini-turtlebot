#!/usr/bin/env python3
"""
tb_bridge/bridge_node.py

ROS2 -> WebSocket JSONL bridge (v0)
- Subscribes: /<robot_id>/cmd_vel (geometry_msgs/Twist)
- Sends WS text frames containing one JSON object per frame (JSON + '\n' for JSONL compatibility)
- Reconnects automatically
- Watchdog: if no cmd_vel for cmd_timeout_ms, sends stop once
- Clean shutdown without Node.add_on_shutdown (not available on Jazzy rclpy Node)
"""

import asyncio
import json
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import websockets


@dataclass
class TwistSample:
    linear: float
    angular: float
    t_monotonic: float  # when received


class TbBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("tb_bridge")

        # --- Parameters ---
        self.declare_parameter("robot_id", "tb_01")
        self.declare_parameter("ws_url", "ws://192.168.0.123:9000/")
        self.declare_parameter("protocol_version", 1)
        self.declare_parameter("src", "ros2")
        self.declare_parameter("priority", 4)

        self.declare_parameter("cmd_timeout_ms", 500)     # stop if stale
        self.declare_parameter("send_rate_hz", 20.0)      # max send rate
        self.declare_parameter("reconnect_backoff_s", 1.0)
        self.declare_parameter("max_backoff_s", 10.0)

        self.robot_id: str = self.get_parameter("robot_id").value
        self.ws_url: str = self.get_parameter("ws_url").value
        self.v: int = int(self.get_parameter("protocol_version").value)
        self.src: str = self.get_parameter("src").value
        self.priority: int = int(self.get_parameter("priority").value)

        self.cmd_timeout_ms: int = int(self.get_parameter("cmd_timeout_ms").value)
        self.send_rate_hz: float = float(self.get_parameter("send_rate_hz").value)
        self.reconnect_backoff_s: float = float(self.get_parameter("reconnect_backoff_s").value)
        self.max_backoff_s: float = float(self.get_parameter("max_backoff_s").value)

        # --- State ---
        self._seq = 0
        self._latest_twist: Optional[TwistSample] = None
        self._latest_lock = threading.Lock()

        self._shutdown_evt = threading.Event()

        # ROS subscription
        topic = f"/{self.robot_id}/cmd_vel"
        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self.get_logger().info(f"Subscribed to {topic}")

        # Run WebSocket client loop in a background thread (asyncio)
        self._ws_thread = threading.Thread(target=self._ws_thread_main, daemon=True)
        self._ws_thread.start()

    # ---------------- ROS callbacks ----------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        sample = TwistSample(
            linear=float(msg.linear.x),
            angular=float(msg.angular.z),
            t_monotonic=time.monotonic(),
        )
        with self._latest_lock:
            self._latest_twist = sample

    # ---------------- Message building ----------------

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _json_dumps(self, obj: dict) -> str:
        # compact JSON
        return json.dumps(obj, separators=(",", ":"))

    def _make_vel_msg(self, linear: float, angular: float) -> str:
        payload = {
            "v": self.v,
            "type": "cmd",
            "robot_id": self.robot_id,
            "cmd": "vel",
            "linear": float(linear),
            "angular": float(angular),
            "seq": self._next_seq(),
            "src": self.src,
            "priority": int(self.priority),
        }
        return self._json_dumps(payload)

    def _make_stop_msg(self) -> str:
        payload = {
            "v": self.v,
            "type": "cmd",
            "robot_id": self.robot_id,
            "cmd": "stop",
            "seq": self._next_seq(),
            "src": self.src,
            "priority": 2,
        }
        return self._json_dumps(payload)

    # ---------------- WebSocket loop ----------------

    def stop(self) -> None:
        """Signal WS loop to exit."""
        self._shutdown_evt.set()

    def _ws_thread_main(self) -> None:
        try:
            asyncio.run(self._ws_main())
        except Exception as e:
            # Don't crash ROS node because WS thread died
            try:
                self.get_logger().error(f"WS thread fatal error: {e}")
            except Exception:
                pass

    async def _ws_main(self) -> None:
        backoff = self.reconnect_backoff_s
        self.get_logger().info(f"WebSocket target: {self.ws_url}")

        last_sent_linear = None
        last_sent_angular = None
        last_send_time = 0.0
        sent_stop_for_timeout = False

        send_period = 1.0 / max(self.send_rate_hz, 1.0)

        while not self._shutdown_evt.is_set():
            try:
                async with websockets.connect(
                    self.ws_url,
                    ping_interval=20,
                    ping_timeout=20,
                    close_timeout=2,
                    max_size=1_000_000,
                ) as ws:
                    self.get_logger().info("WebSocket connected")
                    backoff = self.reconnect_backoff_s
                    sent_stop_for_timeout = False

                    while not self._shutdown_evt.is_set():
                        now = time.monotonic()

                        with self._latest_lock:
                            latest = self._latest_twist

                        # Watchdog: stop if stale
                        stale = (
                            latest is None
                            or (now - latest.t_monotonic) * 1000.0 > self.cmd_timeout_ms
                        )
                        if stale:
                            if not sent_stop_for_timeout:
                                await ws.send(self._make_stop_msg() + "\n")
                                self.get_logger().warn(
                                    f"cmd_vel timeout (> {self.cmd_timeout_ms} ms). Sent STOP."
                                )
                                sent_stop_for_timeout = True
                                last_sent_linear = 0.0
                                last_sent_angular = 0.0
                                last_send_time = now
                            await asyncio.sleep(0.05)
                            continue

                        sent_stop_for_timeout = False

                        # Rate limit
                        if (now - last_send_time) < send_period:
                            await asyncio.sleep(0.001)
                            continue

                        # Suppress duplicates
                        if (
                            last_sent_linear is not None
                            and abs(latest.linear - last_sent_linear) < 1e-6
                            and abs(latest.angular - last_sent_angular) < 1e-6
                        ):
                            await asyncio.sleep(0.002)
                            continue

                        await ws.send(self._make_vel_msg(latest.linear, latest.angular) + "\n")
                        last_sent_linear = latest.linear
                        last_sent_angular = latest.angular
                        last_send_time = now

            except asyncio.CancelledError:
                break
            except Exception as e:
                # Expected when robot is offline
                self.get_logger().warn(f"WebSocket error: {e}. Reconnecting in {backoff:.1f}s")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, self.max_backoff_s)

        self.get_logger().info("WS loop exiting")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TbBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Signal WS loop to stop and give it a moment
        node.stop()
        try:
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
