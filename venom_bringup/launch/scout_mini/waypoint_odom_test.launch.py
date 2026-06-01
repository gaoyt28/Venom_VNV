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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    venom_bringup_source_dir = Path(__file__).resolve().parents[2]
    scout_base_dir = FindPackageShare('scout_base')
    robot_description_dir = FindPackageShare('venom_robot_description')
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
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odom', '/point_lio/odom'),
        ],
        condition=IfCondition(enable_livox),
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': 0.12,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.005,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 6.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
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

    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings + [('cmd_vel', '/cmd_vel_raw')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ]
    nav2_group = GroupAction([
        *nav2_nodes,
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
            'local_plan_topic': '/unused_local_plan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'fake_base_frame': 'base_link_fake',
            'spin_speed': 0.0,
            'publish_frequency_hz': 20.0,
            'cmd_vel_timeout_sec': 0.25,
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
            'robot_base_frame': 'base_link',
            'global_frame': 'odom',
            'require_map_topic': False,
            'require_pose_topic': True,
            'require_tf_ready': True,
            'startup_wait_timeout_sec': 90.0,
            'nav2_activation_timeout_sec': 60.0,
            'final_goal_stop_distance_m': 0.2,
            'stuck_timeout_sec': 60.0,
            'max_recovery_attempts': 1,
            'cruise_max_linear_speed_mps': 0.45,
            'cruise_max_speed_xy_mps': 0.45,
            'cruise_max_angular_speed_radps': 0.7,
            'left_turn_max_linear_speed_mps': 0.25,
            'left_turn_max_speed_xy_mps': 0.25,
            'left_turn_max_angular_speed_radps': 0.6,
            'right_turn_max_linear_speed_mps': 0.25,
            'right_turn_max_speed_xy_mps': 0.25,
            'right_turn_max_angular_speed_radps': 0.6,
            'lane_change_left_max_linear_speed_mps': 0.30,
            'lane_change_left_max_speed_xy_mps': 0.30,
            'lane_change_left_max_angular_speed_radps': 0.6,
            'lane_change_slant_angle_rad': 1.0472,
            'lane_change_slant_distance_m': 1.2,
            'lane_change_right_max_linear_speed_mps': 0.30,
            'lane_change_right_max_speed_xy_mps': 0.30,
            'lane_change_right_max_angular_speed_radps': 0.6,
            'overtake_max_linear_speed_mps': 0.45,
            'overtake_max_speed_xy_mps': 0.45,
            'overtake_max_angular_speed_radps': 0.7,
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
        TimerAction(period=3.0, actions=[nav2_group]),
        TimerAction(period=12.0, actions=[mission_node]),
        rviz_node,
    ])
