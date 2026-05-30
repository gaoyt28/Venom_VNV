"""Scout Mini odom-frame waypoint test without mapping or RTK."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    venom_bringup_source_dir = Path(__file__).resolve().parents[2]
    scout_base_dir = FindPackageShare('scout_base')
    robot_description_dir = FindPackageShare('venom_robot_description')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    headless = LaunchConfiguration('headless')
    waypoint_file = LaunchConfiguration('waypoint_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    point_lio_params_file = LaunchConfiguration('point_lio_params_file')
    enable_livox = LaunchConfiguration('enable_livox')

    default_waypoint_file = os.path.join(
        str(venom_bringup_source_dir), 'config', 'scout_mini', 'waypoint.txt'
    )
    default_nav2_params_file = os.path.join(
        str(venom_bringup_source_dir), 'config', 'scout_mini', 'nav2_params_odom_waypoint.yaml'
    )
    default_point_lio_params_file = os.path.join(
        str(venom_bringup_source_dir), 'config', 'scout_mini', 'point_lio.yaml'
    )
    livox_config_path = os.path.join(
        str(venom_bringup_source_dir), 'config', 'scout_mini', 'MID360_config.json'
    )

    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([scout_base_dir, 'launch', 'scout_mini_base.launch.py'])
        ),
        launch_arguments={
            'port_name': 'can0',
            'is_scout_mini': 'true',
            'is_omni_wheel': 'false',
            'odom_frame': 'odom',
            'odom_topic_name': 'odom',
            'base_frame': 'base_link',
        }.items(),
    )

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'base_link'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': livox_config_path},
            {'cmdline_input_bd_code': '47MDLAS0020103'},
        ],
        condition=IfCondition(enable_livox),
    )

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[point_lio_params_file, {'pcd_save.pcd_save_en': False}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(enable_livox),
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': 0.05,
            'max_height': 0.8,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.001,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 60.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(enable_livox),
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                robot_description_dir,
                'launch',
                'scout_mini_description.launch.py',
            ])
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'True',
            'container_name': 'nav2_container',
            'use_respawn': 'True',
        }.items(),
    )
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[nav2_params_file, {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    nav2_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),
        SetRemap(src='cmd_vel', dst='cmd_vel_raw'),
        SetRemap(src='/cmd_vel_smoothed', dst='/cmd_vel_raw'),
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_raw'),
        nav2_container,
        nav2_launch,
    ])

    base_link_fake_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_fake_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link_fake'],
        condition=IfCondition('false'),
    )
    fake_vel_transform_node = Node(
        package='venom_bringup',
        executable='fake_vel_transform',
        name='fake_vel_transform',
        parameters=[{
            'input_cmd_vel_topic': '/cmd_vel_raw',
            'output_cmd_vel_topic': '/cmd_vel',
            'local_plan_topic': '/local_plan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'fake_base_frame': 'base_link_fake',
            'spin_speed': 0.0,
            'publish_frequency_hz': 20.0,
        }],
        output='screen',
    )

    mission_node = Node(
        package='venom_bringup',
        executable='craic_mission_main',
        name='craic_mission_main',
        parameters=[{
            'waypoint_file': waypoint_file,
            'coordinate_mode': 'cartesian_m',
            'waypoint_frame_id': 'odom',
            'pose_tracking_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel_raw',
            'robot_base_frame': 'base_link_fake',
            'global_frame': 'odom',
            'require_map_topic': False,
            'require_pose_topic': True,
            'require_tf_ready': True,
            'startup_wait_timeout_sec': 90.0,
            'nav2_activation_timeout_sec': 60.0,
            'final_goal_stop_distance_m': 0.2,
            'stuck_timeout_sec': 20.0,
            'max_recovery_attempts': 0,
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(str(venom_bringup_source_dir), 'rviz_cfg', 'mapping.rviz')],
        output='screen',
        condition=UnlessCondition(
            PythonExpression(["'", headless, "' == 'true' or '", headless, "' == 'True'"])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_livox', default_value='false'),
        DeclareLaunchArgument('waypoint_file', default_value=default_waypoint_file),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_file),
        DeclareLaunchArgument('point_lio_params_file', default_value=default_point_lio_params_file),
        scout_base_launch,
        livox_driver_node,
        robot_description_launch,
        point_lio_node,
        pointcloud_to_laserscan_node,
        base_link_fake_static_tf,
        fake_vel_transform_node,
        nav2_group,
        TimerAction(period=8.0, actions=[mission_node]),
        rviz_node,
    ])
