#!/usr/bin/env python3
"""Launch image_rotate_node. Run after hikvision_camera_node (or synced topics)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='hikvision_camera_ros2',
        executable='image_rotate_node',
        name='image_rotate_node',
        output='screen',
    )
    return LaunchDescription([node])
