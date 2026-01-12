#!/usr/bin/env python3
"""
tb_bridge/bridge_node.py

ROS2 -> WebSocket JSONL bridge (v0)
- Subscribes: /<robot_id>/cmd_vel (geometry_msgs/Twist)
- Sends WS text frames containing one JSON object per frame (JSON + '\n' for JSONL compatibility)
- Reconnects automatically
- Watchdog: if no cmd_vel for cmd_timeout_ms, sends stop once
- Exposes: /<robot_id>/set_streams (tb_msgs/srv/SetStreams)
  - Sends/queues a WS "cfg" command to enable/disable sensor streams.
- Receives WS frames (JSONL) and logs them ("WS IN: ...")
  - Later: parse ACK + publish sensor topics.
"""

import asyncio
import json
import threading
import time
import queue
from dataclasses import dataclass
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tb_msgs.srv import SetStreams

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

        # Read parameters
        self.robot_id: str = self.get_parameter("robot_id").value
        self.ws_url: str = self.get_parameter("ws_url").value
        self.v: int = int(self.get_parameter("protocol_version").value)
        self.src: str = self.get_parameter("src").value
        self.priority: int = int(self.get_parameter("priority").value)

        self.cmd_timeout_ms: int = int(self.get_parameter("cmd_timeout_ms").value)
        self.send_rate_hz: float = float(self.get_parameter("send_rate_hz").value)
        self.reconnect_backoff_s: float = float(self.get_parameter("reconnect_backoff_s").value)
        self.max_backoff_s: float = float(self.get_parameter("max_backoff_s").value)

        # --- Service state / WS state ---
        self._next_cfg_id = 1
        self.ws_connected = False
        self._pending_cfg_json: Optional[str] = None

        self._ack_waiters = {}          # cfg_id -> threading.Event
        self._ack_results = {}          # cfg_id -> (ok: bool, msg: str)
        self._ack_lock = threading.Lock()


        # --- State ---
        self._seq = 0
        self._latest_twist: Optional[TwistSample] = None
        self._latest_lock = threading.Lock()
        self._shutdown_evt = threading.Event()
        self._tx_queue: "queue.Queue[str]" = queue.Queue()


        # --- ROS interfaces ---
        topic = f"/{self.robot_id}/cmd_vel"
        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self.get_logger().info(f"Subscribed to {topic}")

        self.srv_set_streams = self.create_service(
            SetStreams,
            f"/{self.robot_id}/set_streams",
            self.on_set_streams,
        )

        # --- Run WebSocket client loop in a background thread (asyncio) ---
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

    def on_set_streams(self, request, response):
        """
        Build a WS cfg command. If WS is connected, send immediately.
        Otherwise queue it for send-on-connect.
        """
        cfg_id = self._next_cfg_id
        self._next_cfg_id += 1

        evt = threading.Event()
        with self._ack_lock:
            self._ack_waiters[cfg_id] = evt
            # remove any stale result for this id (shouldn't exist, but safe)
            self._ack_results.pop(cfg_id, None)


        cmd: Dict[str, Any] = {
            "v": self.v,
            "type": "cfg",
            "robot_id": self.robot_id,
            "id": cfg_id,
            "ts_ms": int(time.time() * 1000),
            "streams": {
                "motors": {"en": bool(request.motors), "hz": float(request.motors_hz)},
                "imu": {"en": bool(request.imu), "hz": float(request.imu_hz)},
                "ir": {"en": bool(request.ir), "hz": float(request.ir_hz)},
                "lidar": {"en": bool(request.lidar), "hz": float(request.lidar_hz)},
            },
        }

        # Your srv appears to include these fields; include them if present.
        if hasattr(request, "lidar360") and hasattr(request, "lidar360_hz"):
            cmd["streams"]["lidar360"] = {"en": bool(request.lidar360), "hz": float(request.lidar360_hz)}

        cmd_json = json.dumps(cmd)
        self._pending_cfg_json = cmd_json  # last-config-wins queue

        self.get_logger().info(f"CFG OUT: {cmd_json}")

        # enqueue for immediate send (WS thread will send it if connected)
        self._tx_queue.put(cmd_json + "\n")


        # if self.ws_connected:
        #     # WS loop will also pick up _pending_cfg_json on connect;
        #     # here we report success in accepting the request.
        #     response.ok = True
        #     response.message = f"cfg queued id={cfg_id} (WS connected; will send now)"
        # else:
        #     response.ok = True
        #     response.message = f"cfg queued id={cfg_id} (WS not connected; will send on connect)"
        #     self.get_logger().warn("WS not connected; cfg queued for send-on-connect")

        # return response
    
        # If WS isn't connected, don't wait; keep it queued for later.
        
        if not self.ws_connected:
            response.ok = False
            response.message = f"WS not connected; cfg queued id={cfg_id}"
            return response

        # Wait for ACK (timeout seconds)
        timeout_s = 1.5
        got = evt.wait(timeout_s)

        with self._ack_lock:
            # Clean up waiter regardless
            self._ack_waiters.pop(cfg_id, None)
            result = self._ack_results.pop(cfg_id, None)

        if not got:
            response.ok = False
            response.message = f"timeout waiting for ack id={cfg_id}"
            return response

        if result is None:
            response.ok = False
            response.message = f"ack missing result id={cfg_id}"
            return response

        ok, msg = result
        response.ok = bool(ok)
        response.message = msg if msg else f"ack received id={cfg_id}"
        return response

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

    async def _ws_recv_loop(self, ws) -> None:
        """
        Receives JSONL frames from TB and logs them.
        Later we will parse ACK + publish topics.
        """
        while not self._shutdown_evt.is_set():
            try:
                msg = await ws.recv()
            except Exception as e:
                self.get_logger().warn(f"WS recv ended: {e}")
                return

            if not msg:
                continue

            if isinstance(msg, bytes):
                msg = msg.decode("utf-8", errors="ignore")

            for line in msg.splitlines():
                line = line.strip()
                if not line:
                    continue
                self.get_logger().info(f"WS IN: {line}")                

                try:
                    obj = json.loads(line)
                except Exception:
                    continue

                # if obj.get("type") == "ack":
                #     self.get_logger().info(
                #         f"ACK IN: id={obj.get('id')} ok={obj.get('ok')} msg={obj.get('msg')}"

                #         with self._ack_lock:
                #             self._ack_results[cfg_id] = (bool(obj.get("ok", False)), str(obj.get("msg", "")))
                #             evt = self._ack_waiters.get(cfg_id)

                #         if evt is not None:
                #             evt.set()
                #     )

                if obj.get("type") == "ack":
                    cfg_id = obj.get("id")
                    self.get_logger().info(
                        f"ACK IN: id={cfg_id} ok={obj.get('ok')} msg={obj.get('msg')}"
                    )

                    if isinstance(cfg_id, int):
                        with self._ack_lock:
                            self._ack_results[cfg_id] = (bool(obj.get("ok", False)), str(obj.get("msg", "")))
                            evt = self._ack_waiters.get(cfg_id)
                        if evt is not None:
                            evt.set()



    async def _ws_main(self) -> None:
        backoff = self.reconnect_backoff_s
        self.get_logger().info(f"WebSocket target: {self.ws_url}")

        last_sent_linear = None
        last_sent_angular = None
        last_send_time = 0.0
        sent_stop_for_timeout = False

        send_period = 1.0 / max(self.send_rate_hz, 1.0)

        while not self._shutdown_evt.is_set():
            recv_task = None
            try:
                async with websockets.connect(
                    self.ws_url,
                    ping_interval=20,
                    ping_timeout=20,
                    close_timeout=2,
                    max_size=1_000_000,
                ) as ws:
                    self.ws_connected = True
                    self.get_logger().info("WebSocket connected")

                    # Start receiver
                    recv_task = asyncio.create_task(self._ws_recv_loop(ws))

                    # Send pending cfg immediately on connect
                    if self._pending_cfg_json is not None:
                        try:
                            await ws.send(self._pending_cfg_json + "\n")
                            self.get_logger().info(f"CFG SENT ON CONNECT: {self._pending_cfg_json}")
                            self._pending_cfg_json = None
                        except Exception as e:
                            self.get_logger().warn(f"Failed to send pending cfg on connect: {e}")

                    backoff = self.reconnect_backoff_s
                    sent_stop_for_timeout = False

                    while not self._shutdown_evt.is_set():


                        # Send any queued outgoing messages (cfg, later other things)
                        try:
                            while True:
                                out = self._tx_queue.get_nowait()
                                await ws.send(out)
                                # Optional: debug log
                                if out.startswith("{") and '"type":"cfg"' in out:
                                    self.get_logger().info(f"CFG SENT: {out.strip()}")
                        except queue.Empty:
                            pass


                        # If receiver died, treat as connection failure
                        if recv_task is not None and recv_task.done():
                            raise ConnectionError("WS recv loop ended")

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
                self.get_logger().warn(f"WebSocket error: {e}. Reconnecting in {backoff:.1f}s")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, self.max_backoff_s)
            finally:
                self.ws_connected = False
                if recv_task is not None:
                    try:
                        recv_task.cancel()
                    except Exception:
                        pass

        self.get_logger().info("WS loop exiting")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TbBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
