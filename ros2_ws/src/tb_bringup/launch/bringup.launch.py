from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_host = 'pico.local'
    return LaunchDescription([
        Node(package='tb_driver', executable='driver_node', name='tb_driver',
             parameters=[{'robot_host': robot_host, 'robot_port': 9000}]),
        Node(package='tb_cam_client', executable='cam_client_node', name='tb_cam_client',
             parameters=[{'robot_host': robot_host, 'mjpeg_port': 8080}]),
        Node(package='tb_lidar_client', executable='lidar_client_node', name='tb_lidar_client',
             parameters=[{'lidar_port': 5601}]),
        Node(package='tb_localization', executable='ekf_node', name='tb_localization'),
    ])
