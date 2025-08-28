# tb_driver

Connects to the robot's Control JSONL TCP server (:9000). Subscribes to `/cmd_vel`, sends `set_speed`, provides basic IMU polling example.

## Run
```bash
source ../install/setup.bash
ros2 run tb_driver driver_node
```
