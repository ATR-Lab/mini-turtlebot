# ROS 2 Workspace

Contains client packages and bringup utilities to connect to the turtlebot.

# Launch ROS Bridge

`ros2 launch tb_bringup bringup.launch.py robot_id:=<tb_id> robot_ip:=<ip_address> port:=9000 path:=/ws`

Replace `<tb_id>` with turtlebot id. for example: `tb_01`
Replace `<ip_address>` with the turtlebot's ip address.

Example command for tb_01 with ip address 192.168.1.101 will be:
`ros2 launch tb_bringup bringup.launch.py robot_id:=tb_01 robot_ip:=192.168.1.101 port:=9000 path:=/ws`

# Speed command
The below command publishes speed command (30% speed) continuously at 10Hz to turtlebot: `tb_01` 
`ros2 topic pub -r 10 /tb_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"`

The below command publishes speed command (30% speed) once to turtlebot: `tb_01` 
`ros2 topic pub --once /tb_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"`


