from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    robot_ip = LaunchConfiguration('robot_ip')
    port = LaunchConfiguration('port')
    path = LaunchConfiguration('path')

    # If you keep WS at root "/", keep path:="/"
    # If you implement "/control", set path:="/control"
    ws_url = [ 'ws://', robot_ip, ':', port, path ]

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='tb_01'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.105'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('path', default_value='/'),

        Node(
            package='tb_bridge',
            executable='tb_bridge_node',
            name=['tb_bridge_', robot_id],
            output='screen',
            parameters=[
                {'robot_id': robot_id},
                {'ws_url': ws_url},
                {'cmd_timeout_ms': 500},
                {'send_rate_hz': 20.0},
                {'src': 'ros2'},
                {'priority': 4},
            ],
        ),
    ])
