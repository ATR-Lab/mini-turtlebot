from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    """
    robots argument format (comma-separated):
      tb_01@192.168.2.105,tb_02@192.168.0.52,tb_03@192.168.0.53
    """
    robots = LaunchConfiguration('robots').perform(context).strip()
    port = LaunchConfiguration('port').perform(context).strip()
    path = LaunchConfiguration('path').perform(context).strip()

    if not robots:
        return []

    nodes = []
    for item in robots.split(','):
        item = item.strip()
        if not item:
            continue
        try:
            robot_id, ip = item.split('@', 1)
            robot_id = robot_id.strip()
            ip = ip.strip()
        except ValueError:
            raise RuntimeError(f'Bad robots entry "{item}". Expected robot_id@ip')

        ws_url = f'ws://{ip}:{port}{path}'
        nodes.append(
            Node(
                package='tb_bridge',
                executable='tb_bridge_node',
                name=f'tb_bridge_{robot_id}',
                output='screen',
                parameters=[
                    {'robot_id': robot_id},
                    {'ws_url': ws_url},
                    {'cmd_timeout_ms': 500},
                    {'send_rate_hz': 20.0},
                    {'src': 'ros2'},
                    {'priority': 4},
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='tb_01@192.168.0.51,tb_02@192.168.0.52',
            description='Comma-separated list robot_id@ip',
        ),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('path', default_value='/'),
        OpaqueFunction(function=_make_nodes),
    ])
