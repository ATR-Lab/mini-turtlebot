# Implementation

## Robot (Pico‑W)
- MicroPython or C SDK may be used. Minimal services:
  - **JSONL Control (TCP :9000):** Commands: `ping`, `set_speed`, `imu` (examples).
  - **MJPEG Server (HTTP :8080):** Stub OK; replace with your camera driver.
  - **LiDAR UDP (:5601):** Send scans as newline‑delimited JSON for simplicity.
- On‑robot modules: Camera Sensor Frame Buffer → Vision → MJPEG server; Motor Management; On‑robot Localization (optional); Battery Management; Config.

## Ground Station (ROS 2)
- `tb_driver`: converts `/cmd_vel` to left/right and talks JSONL over TCP.
- `tb_cam_client`: pulls MJPEG, publishes `sensor_msgs/Image`.
- `tb_lidar_client`: parses UDP JSON → `sensor_msgs/LaserScan`.
- `tb_localization`: EKF stub compatible with Nav2; plug your filter (e.g., robot_localization) via params in `tb_localization/config/ekf.yaml`.
- `tb_bringup`: one‑shot launch for all clients; parameters for robot host/ports.

## Scratch / ATR Bridge
- WebSocket bridge converts Scratch intents (from PC mic) into ROS topics.
- No link between TB_AUDIO_Client and Scratch. Audio remains ROS‑only.
