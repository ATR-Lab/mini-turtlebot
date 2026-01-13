#!/usr/bin/env python3
"""tb_bridge/bridge_node.py

ROS2 <-> WebSocket JSONL bridge (v1)

ROS -> WS:
- Subscribes: /<robot_id>/cmd_vel (geometry_msgs/Twist)
- Sends WS JSONL frames (one JSON per line, newline terminated)
- Watchdog: if no cmd_vel for cmd_timeout_ms, sends STOP once (including immediately on startup)
- Reconnects automatically with backoff

ROS Service -> WS (cfg):
- Provides: /<robot_id>/set_streams (tb_msgs/srv/SetStreams)
- Sends WS "cfg" command and waits for WS "ack" (id-matched)
- If WS not connected: returns ok=False and message includes queued cfg id
  (cfg is still queued and will send on next connect)

WS -> ROS (receive):
- Receives WS frames and logs "WS IN: ..."
- Parses "ack" frames and wakes waiting service calls
- Publishes:
    /<robot_id>/motor_state      (tb_msgs/MotorState) from state=motors
    /<robot_id>/imu/data_raw     (sensor_msgs/Imu)   from state=imu

Notes:
- Uses a background thread running an asyncio WS client.
- ROS callbacks run in the main rclpy thread.
- Cross-thread send uses a threadsafe Queue (self._tx_queue).
"""

import asyncio
import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from tb_msgs.srv import SetStreams
from tb_msgs.msg import MotorState

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
        self.declare_parameter("ws_url", "ws://192.168.0.123:9000/ws")
        self.declare_parameter("protocol_version", 1)
        self.declare_parameter("src", "ros2")
        self.declare_parameter("priority", 4)

        self.declare_parameter("cmd_timeout_ms", 500)     # stop if stale
        self.declare_parameter("send_rate_hz", 20.0)      # send vel at this rate while fresh
        self.declare_parameter("reconnect_backoff_s", 1.0)
        self.declare_parameter("max_backoff_s", 10.0)

        # SetStreams behavior
        self.declare_parameter("set_streams_ack_timeout_s", 1.5)

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

        self.set_streams_ack_timeout_s: float = float(
            self.get_parameter("set_streams_ack_timeout_s").value
        )

        # --- WS state / control ---
        self.ws_connected = False
        self._shutdown_evt = threading.Event()

        # Outbound queue (ROS thread -> WS thread)
        self._tx_queue: "queue.Queue[str]" = queue.Queue()

        # For "send on next connect" convenience (last-config-wins)
        self._pending_cfg_json: Optional[str] = None

        # --- ACK waiters for SetStreams ---
        self._next_cfg_id = 1
        self._ack_lock = threading.Lock()
        self._ack_waiters: Dict[int, threading.Event] = {}
        self._ack_results: Dict[int, Tuple[bool, str]] = {}

        # --- cmd_vel state ---
        self._seq = 0
        self._latest_twist: Optional[TwistSample] = None
        self._latest_lock = threading.Lock()

        # --- ROS interfaces ---
        topic = f"/{self.robot_id}/cmd_vel"
        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self.get_logger().info(f"Subscribed to {topic}")

        self.create_service(SetStreams, f"/{self.robot_id}/set_streams", self.on_set_streams)

        self.pub_motor_state = self.create_publisher(
            MotorState, f"/{self.robot_id}/motor_state", 10
        )
        self.pub_imu = self.create_publisher(
            Imu, f"/{self.robot_id}/imu/data_raw", 10
        )

        # --- WS thread ---
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
        """Send "cfg" over WS and wait for matching "ack".

        If WS not connected, queue cfg for later and return ok=False.
        """
        cfg_id = self._next_cfg_id
        self._next_cfg_id += 1

        # Register waiter before sending
        evt = threading.Event()
        with self._ack_lock:
            self._ack_waiters[cfg_id] = evt
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

        # If your srv has lidar360 fields, include them too
        if hasattr(request, "lidar360") and hasattr(request, "lidar360_hz"):
            cmd["streams"]["lidar360"] = {
                "en": bool(getattr(request, "lidar360")),
                "hz": float(getattr(request, "lidar360_hz")),
            }

        cmd_json = json.dumps(cmd)
        self._pending_cfg_json = cmd_json  # last-config-wins fallback
        self.get_logger().info(f"CFG OUT: {cmd_json}")

        # Always enqueue for WS thread send
        self._tx_queue.put(cmd_json + "\n")

        # If not connected, don't wait. (Still queued for next connect.)
        if not self.ws_connected:
            with self._ack_lock:
                self._ack_waiters.pop(cfg_id, None)
                self._ack_results.pop(cfg_id, None)
            response.ok = False
            response.message = f"WS not connected; cfg queued id={cfg_id}"
            self.get_logger().warn(response.message)
            return response

        # Wait for ACK
        got = evt.wait(self.set_streams_ack_timeout_s)

        with self._ack_lock:
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

        ok, msg_text = result
        response.ok = bool(ok)
        response.message = msg_text if msg_text else f"ack received id={cfg_id}"
        return response

    # ---------------- Message building ----------------

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _json_dumps(self, obj: dict) -> str:
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
        self._shutdown_evt.set()

    def _ws_thread_main(self) -> None:
        try:
            asyncio.run(self._ws_main())
        except Exception as e:
            try:
                self.get_logger().error(f"WS thread fatal error: {e}")
            except Exception:
                pass

    async def _ws_recv_loop(self, ws) -> None:
        """Receives JSONL frames from TB.

        Handles:
        - ack: wakes waiting SetStreams call
        - state motors: publishes MotorState
        - state imu: publishes sensor_msgs/Imu

        Expected motor frame (from firmware):
          {"type":"state","state":"motors","left_rads":..,"right_rads":..,"left_pwm":..,"right_pwm":..}

        Expected imu frame (you will add in firmware):
          {"type":"state","state":"imu","ax":..,"ay":..,"az":..,"gx":..,"gy":..,"gz":..,
           "qx":..,"qy":..,"qz":..,"qw":..}
        (orientation fields optional; if missing, orientation_covariance[0] = -1)
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

            for line in str(msg).splitlines():
                line = line.strip()
                if not line:
                    continue

                self.get_logger().info(f"WS IN: {line}")

                try:
                    obj = json.loads(line)
                except Exception:
                    continue

                msg_type = obj.get("type")

                # --- ACK ---
                if msg_type == "ack":
                    cfg_id = obj.get("id")
                    ok = bool(obj.get("ok", False))
                    msg_text = str(obj.get("msg", ""))
                    self.get_logger().info(f"ACK IN: id={cfg_id} ok={ok} msg={msg_text}")

                    if isinstance(cfg_id, int):
                        with self._ack_lock:
                            self._ack_results[cfg_id] = (ok, msg_text)
                            evt = self._ack_waiters.get(cfg_id)
                        if evt is not None:
                            evt.set()
                    continue

                # --- STATE ---
                if msg_type == "state":
                    state = obj.get("state")

                    if state == "motors":
                        try:
                            m = MotorState()
                            m.left_rads = float(obj.get("left_rads", 0.0))
                            m.right_rads = float(obj.get("right_rads", 0.0))
                            m.left_pwm = int(obj.get("left_pwm", 0))
                            m.right_pwm = int(obj.get("right_pwm", 0))
                            self.pub_motor_state.publish(m)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to publish MotorState: {e}")

                    elif state == "imu":
                        try:
                            imu = Imu()
                            imu.header.stamp = self.get_clock().now().to_msg()
                            imu.header.frame_id = f"{self.robot_id}/imu_link"

                            # linear acceleration (m/s^2)
                            imu.linear_acceleration.x = float(obj.get("ax", 0.0))
                            imu.linear_acceleration.y = float(obj.get("ay", 0.0))
                            imu.linear_acceleration.z = float(obj.get("az", 0.0))

                            # angular velocity (rad/s)
                            imu.angular_velocity.x = float(obj.get("gx", 0.0))
                            imu.angular_velocity.y = float(obj.get("gy", 0.0))
                            imu.angular_velocity.z = float(obj.get("gz", 0.0))

                            # orientation optional
                            if all(k in obj for k in ("qx", "qy", "qz", "qw")):
                                imu.orientation.x = float(obj.get("qx", 0.0))
                                imu.orientation.y = float(obj.get("qy", 0.0))
                                imu.orientation.z = float(obj.get("qz", 0.0))
                                imu.orientation.w = float(obj.get("qw", 1.0))
                                # If you later provide covariances, fill them; for now leave defaults (0)
                            else:
                                # Unknown orientation
                                imu.orientation_covariance[0] = -1.0

                            self.pub_imu.publish(imu)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to publish Imu: {e}")

    async def _ws_main(self) -> None:
        backoff = self.reconnect_backoff_s
        self.get_logger().info(f"WebSocket target: {self.ws_url}")

        sent_stop_for_timeout = False
        send_period = 1.0 / max(self.send_rate_hz, 1.0)
        last_send_time = 0.0

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

                    recv_task = asyncio.create_task(self._ws_recv_loop(ws))

                    # If something was queued before connection, push it once.
                    if self._pending_cfg_json is not None:
                        try:
                            await ws.send(self._pending_cfg_json + "\n")
                            self.get_logger().info(f"CFG SENT ON CONNECT: {self._pending_cfg_json}")
                            self._pending_cfg_json = None
                        except Exception as e:
                            self.get_logger().warn(f"Failed to send pending cfg on connect: {e}")

                    # Keep the behavior: STOP immediately on startup/connect
                    sent_stop_for_timeout = False
                    last_send_time = 0.0

                    backoff = self.reconnect_backoff_s

                    while not self._shutdown_evt.is_set():
                        # If receiver died, treat as connection failure
                        if recv_task is not None and recv_task.done():
                            raise ConnectionError("WS recv loop ended")

                        # Send queued outbound messages (cfg etc.)
                        try:
                            while True:
                                out = self._tx_queue.get_nowait()
                                await ws.send(out)
                        except queue.Empty:
                            pass

                        now = time.monotonic()

                        with self._latest_lock:
                            latest = self._latest_twist

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
                                last_send_time = now
                            await asyncio.sleep(0.05)
                            continue

                        # cmd_vel is fresh
                        sent_stop_for_timeout = False

                        # Rate limit
                        if (now - last_send_time) < send_period:
                            await asyncio.sleep(0.001)
                            continue

                        # IMPORTANT FIX:
                        # Do NOT suppress duplicates. TB firmware has its own timeout; it needs refresh.
                        await ws.send(self._make_vel_msg(latest.linear, latest.angular) + "\n")
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