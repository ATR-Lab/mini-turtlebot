# Proposed System

## Overview
A **single-radio Pico‑W** robot handles motor control, IMU/IR/encoders, and battery while exposing three network services:

- **Control TCP JSONL (:9000):** Text-based control/telemetry. Each line is a JSON object, e.g. `{ "cmd": "set_speed", "left": 0.2, "right": 0.2 }`.
- **MJPEG Stream (:8080):** HTTP multipart stream of JPEG frames from the Turtle‑Head camera (SPI/I²C). **MJPEG‑only** by design.
- **LiDAR UDP (:5601):** Newline‑delimited JSON arrays of `[angle_deg, distance_m]` pairs.

The **Ground Station (ROS 2)** runs: `tb_driver`, `tb_cam_client`, `tb_lidar_client`, `tb_localization (EKF)`, with optional `Odometry/SLAM` and `Nav2`.

The **Scratch Interface + ATR Bridge** lives on the PC. The **PC microphone** is used for Voice Commands that are translated to ROS topics and then to the robot via `tb_driver`. Robot audio does **not** connect to Scratch.
