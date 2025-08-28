<p align="center">
  <a href="" rel="noopener">
 <img src="resources/images/ATR-logo.gif" alt="ATR"></a>
</p>

<h3 align="center">ATR Mini TurtleBot</h3>

<div align="center">

  [![Status](https://img.shields.io/badge/status-active-success.svg)]() 
  [![build](https://img.shields.io/badge/build-melodic-green)]()
  [![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> this repo contains the steps for setup your environment and necessary source code for the Mini TurtleBot development
</p>

# Mini TurtleBot

A complete scaffold for the **MiniTurtleBot**:

- **Robot (Pico W, single radio):** drives motors, reads IMU/IR/encoders and hosts three lightweight network services:
  - **:8080** — MJPEG Stream Server (SPI/I²C camera in Turtle‑Head)
  - **:5601** — LiDAR UDP Server (newline‑delimited JSON scans)
  - **:9000** — Control TCP JSONL Server (one JSON object per line)
- **PC Side (ROS 2):** modules:
  - `tb_driver` (connects to :9000)
  - `tb_cam_client` (MJPEG only camera client)
  - `tb_lidar_client` (UDP scan parser → `sensor_msgs/LaserScan`)
  - `tb_localization` (EKF stub; plug into Nav2/SLAM as needed)
  - `tb_bringup` (launch file to start the above)
- **Scratch / ATR Bridge:** Scratch 3 extension + WebSocket bridge to ROS 2.  
  **Voice Cmd uses the PC microphone** → ATR bridge → ROS. **Robot audio (ADC→Audio Stream→TB_AUDIO_Client) is ROS‑only and not connected to Scratch**.

