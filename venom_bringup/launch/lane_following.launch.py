"""Launch USB camera lane detection and optional low-speed lane following."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('venom_bringup')
    default_params = os.path.join(pkg_share, 'config', 'scout_mini', 'lane_following.yaml')

    params_file = LaunchConfiguration('params_file')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_detector = LaunchConfiguration('enable_detector')
    enable_controller = LaunchConfiguration('enable_controller')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_controller', default_value='false'),
        Node(
            package='venom_bringup',
            executable='usb_camera_node',
            name='usb_camera_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_camera),
        ),
        Node(
            package='venom_bringup',
            executable='lane_detector_node',
            name='lane_detector_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_detector),
        ),
        Node(
            package='venom_bringup',
            executable='lane_follow_controller',
            name='lane_follow_controller',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_controller),
        ),
    ])
