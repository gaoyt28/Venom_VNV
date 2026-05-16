"""Scout Mini bringup with overtake manager nodes."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    venom_bringup_dir = get_package_share_directory('venom_bringup')
    overtake_dir = get_package_share_directory('venom_overtake_manager')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    overtake_params_file = LaunchConfiguration('overtake_params_file')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(venom_bringup_dir, 'launch', 'bringup_all.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'nav2_params_file': nav2_params_file,
        }.items(),
    )

    tracker_node = Node(
        package='venom_overtake_manager',
        executable='tracker_node',
        name='tracker_node',
        output='screen',
        parameters=[overtake_params_file, {'use_sim_time': use_sim_time}],
    )

    lead_selector_node = Node(
        package='venom_overtake_manager',
        executable='lead_selector_node',
        name='lead_selector_node',
        output='screen',
        parameters=[overtake_params_file, {'use_sim_time': use_sim_time}],
    )

    overtake_manager_node = Node(
        package='venom_overtake_manager',
        executable='overtake_manager_node',
        name='overtake_manager_node',
        output='screen',
        parameters=[overtake_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                venom_bringup_dir,
                'config',
                'scout_mini',
                'craic_nav2_params.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'overtake_params_file',
            default_value=os.path.join(
                overtake_dir,
                'config',
                'overtake_manager.yaml',
            ),
        ),
        bringup_launch,
        tracker_node,
        lead_selector_node,
        overtake_manager_node,
    ])
