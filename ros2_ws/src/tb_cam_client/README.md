# tb_cam_client

Reads MJPEG from `http://ROBOT_IP:8080/` and publishes `sensor_msgs/Image` (MJPEG‑only by design).

## Run
```bash
source ../install/setup.bash
ros2 run tb_cam_client client_node
```
