from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    robot_ip = LaunchConfiguration('robot_ip')
    port = LaunchConfiguration('port')
    path = LaunchConfiguration('path')
    ws_url = LaunchConfiguration('ws_url')

    # Build from robot_ip/port/path (original behavior)
    ws_url_built = ['ws://', robot_ip, ':', port, path]

    # True when ws_url != ""
    use_ws_url = IfCondition(PythonExpression(["'", ws_url, "' != ''"]))
    use_built = UnlessCondition(PythonExpression(["'", ws_url, "' != ''"]))

    common_params = [
        {'robot_id': robot_id},
        {'cmd_timeout_ms': 500},
        {'send_rate_hz': 20.0},
        {'src': 'ros2'},
        {'priority': 4},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='tb_01'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.105'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('path', default_value='/ws'),

        # Optional override. If empty, we fall back to ws://robot_ip:port/path
        DeclareLaunchArgument('ws_url', default_value=''),

        # Case 1: ws_url provided (localhost testing, etc.)
        Node(
            condition=use_ws_url,
            package='tb_bridge',
            executable='tb_bridge_node',
            name=['tb_bridge_', robot_id],
            output='screen',
            parameters=common_params + [
                {'ws_url': ws_url},
            ],
        ),

        # Case 2: ws_url not provided -> build from robot_ip/port/path
        Node(
            condition=use_built,
            package='tb_bridge',
            executable='tb_bridge_node',
            name=['tb_bridge_', robot_id],
            output='screen',
            parameters=common_params + [
                {'ws_url': ws_url_built},
            ],
        ),
    ])
