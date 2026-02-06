#!/usr/bin/env python3
"""Launch hikvision_camera_node + image_rotate_node (camera + 90° rotate)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = get_package_share_directory('hikvision_camera_ros2')
    config_path = os.path.join(config_dir, 'config', 'camera_params.yaml')

    device_ip_arg = DeclareLaunchArgument(
        'device_ip', default_value='192.168.1.64', description='Camera IP address'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='8000', description='Camera SDK port'
    )
    username_arg = DeclareLaunchArgument(
        'username', default_value='admin', description='Login username'
    )
    password_arg = DeclareLaunchArgument(
        'password', default_value='qwer1234', description='Login password (required)'
    )
    channel_visible_arg = DeclareLaunchArgument(
        'channel_visible', default_value='1', description='Channel number for visible camera'
    )
    channel_ir_arg = DeclareLaunchArgument(
        'channel_ir', default_value='2', description='Channel number for IR camera'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='25.0', description='Publish rate (Hz), for SDK mode'
    )
    use_rtsp_arg = DeclareLaunchArgument(
        'use_rtsp', default_value='true', description='Use RTSP for smooth high-FPS stream'
    )

    camera_node = Node(
        package='hikvision_camera_ros2',
        executable='hikvision_camera_node',
        name='hikvision_camera_node',
        output='screen',
        parameters=[
            config_path,
            {
                'device_ip': LaunchConfiguration('device_ip'),
                'port': LaunchConfiguration('port'),
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),
                'channel_visible': LaunchConfiguration('channel_visible'),
                'channel_ir': LaunchConfiguration('channel_ir'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'use_rtsp': LaunchConfiguration('use_rtsp'),
            },
        ],
    )

    rotate_node = Node(
        package='hikvision_camera_ros2',
        executable='image_rotate_node',
        name='image_rotate_node',
        output='screen',
    )

    return LaunchDescription([
        device_ip_arg,
        port_arg,
        username_arg,
        password_arg,
        channel_visible_arg,
        channel_ir_arg,
        frame_rate_arg,
        use_rtsp_arg,
        camera_node,
        rotate_node,
    ])
