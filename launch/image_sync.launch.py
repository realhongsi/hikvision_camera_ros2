#!/usr/bin/env python3
"""Launch image_sync_node. Run after hikvision_camera_node is publishing."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='hikvision_camera_ros2',
        executable='image_sync_node',
        name='image_sync_node',
        output='screen',
    )
    return LaunchDescription([node])
