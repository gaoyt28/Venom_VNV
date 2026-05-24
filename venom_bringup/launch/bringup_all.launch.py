"""One-command CRAIC bringup for Scout Mini."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    venom_bringup_dir = get_package_share_directory('venom_bringup')
    robot_description_dir = get_package_share_directory('venom_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    scout_base_dir = get_package_share_directory('scout_base')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    waypoint_file = LaunchConfiguration('waypoint_file')
    road_network_file = LaunchConfiguration('road_network_file')
    route_name = LaunchConfiguration('route_name')
    route_nodes = LaunchConfiguration('route_nodes')
    start_node_id = LaunchConfiguration('start_node_id')
    goal_node_id = LaunchConfiguration('goal_node_id')
    start_x_m = LaunchConfiguration('start_x_m')
    start_y_m = LaunchConfiguration('start_y_m')
    goal_x_m = LaunchConfiguration('goal_x_m')
    goal_y_m = LaunchConfiguration('goal_y_m')
    use_start_goal_xy = LaunchConfiguration('use_start_goal_xy')
    blocked_edges = LaunchConfiguration('blocked_edges')
    coordinate_mode = LaunchConfiguration('coordinate_mode')
    map_origin_longitude_deg = LaunchConfiguration('map_origin_longitude_deg')
    map_origin_latitude_deg = LaunchConfiguration('map_origin_latitude_deg')
    map_origin_x_m = LaunchConfiguration('map_origin_x_m')
    map_origin_y_m = LaunchConfiguration('map_origin_y_m')
    map_origin_yaw_rad = LaunchConfiguration('map_origin_yaw_rad')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    point_lio_params_file = LaunchConfiguration('point_lio_params_file')
    startup_wait_timeout_sec = LaunchConfiguration('startup_wait_timeout_sec')
    nav2_activation_timeout_sec = LaunchConfiguration('nav2_activation_timeout_sec')
    final_goal_stop_distance_m = LaunchConfiguration('final_goal_stop_distance_m')
    stuck_timeout_sec = LaunchConfiguration('stuck_timeout_sec')
    stuck_progress_radius_m = LaunchConfiguration('stuck_progress_radius_m')
    max_recovery_attempts = LaunchConfiguration('max_recovery_attempts')
    backup_distance_m = LaunchConfiguration('backup_distance_m')
    backup_speed_mps = LaunchConfiguration('backup_speed_mps')
    spin_angle_rad = LaunchConfiguration('spin_angle_rad')
    recovery_time_allowance_sec = LaunchConfiguration('recovery_time_allowance_sec')
    progress_check_period_sec = LaunchConfiguration('progress_check_period_sec')

    enable_scout_base = LaunchConfiguration('enable_scout_base')
    enable_livox = LaunchConfiguration('enable_livox')
    enable_lio_to_scan = LaunchConfiguration('enable_lio_to_scan')
    enable_rtk = LaunchConfiguration('enable_rtk')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_obstacle_filter = LaunchConfiguration('enable_obstacle_filter')
    enable_yolo = LaunchConfiguration('enable_yolo')
    enable_ipm = LaunchConfiguration('enable_ipm')
    use_rtk_tcp_fix_bridge = LaunchConfiguration('use_rtk_tcp_fix_bridge')
    rtk_host = LaunchConfiguration('rtk_host')
    rtk_port = LaunchConfiguration('rtk_port')
    rtk_frame_id = LaunchConfiguration('rtk_frame_id')
    rtk_fix_topic = LaunchConfiguration('rtk_fix_topic')
    rtk_default_altitude_m = LaunchConfiguration('rtk_default_altitude_m')
    rtk_use_altitude_from_stream = LaunchConfiguration('rtk_use_altitude_from_stream')
    rtk_covariance_m2 = LaunchConfiguration('rtk_covariance_m2')
    rtk_socket_timeout_sec = LaunchConfiguration('rtk_socket_timeout_sec')
    rtk_reconnect_interval_sec = LaunchConfiguration('rtk_reconnect_interval_sec')

    default_waypoint_file = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'waypoint.txt'
    )
    default_road_network_file = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'road_network.yaml'
    )
    default_nav2_params_file = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'nav2_params.yaml'
    )
    default_ekf_params_file = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'craic_ekf.yaml'
    )
    default_point_lio_params_file = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'point_lio.yaml'
    )
    livox_config_path = os.path.join(
        venom_bringup_dir, 'config', 'scout_mini', 'MID360_config.json'
    )

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('waypoint_file', default_value=default_waypoint_file),
        DeclareLaunchArgument('road_network_file', default_value=''),
        DeclareLaunchArgument('route_name', default_value=''),
        DeclareLaunchArgument('route_nodes', default_value=''),
        DeclareLaunchArgument('start_node_id', default_value=''),
        DeclareLaunchArgument('goal_node_id', default_value=''),
        DeclareLaunchArgument('start_x_m', default_value='0.0'),
        DeclareLaunchArgument('start_y_m', default_value='0.0'),
        DeclareLaunchArgument('goal_x_m', default_value='0.0'),
        DeclareLaunchArgument('goal_y_m', default_value='0.0'),
        DeclareLaunchArgument('use_start_goal_xy', default_value='false'),
        DeclareLaunchArgument('blocked_edges', default_value=''),
        DeclareLaunchArgument('coordinate_mode', default_value='geodetic'),
        DeclareLaunchArgument('map_origin_longitude_deg', default_value='0.0'),
        DeclareLaunchArgument('map_origin_latitude_deg', default_value='0.0'),
        DeclareLaunchArgument('map_origin_x_m', default_value='0.0'),
        DeclareLaunchArgument('map_origin_y_m', default_value='0.0'),
        DeclareLaunchArgument('map_origin_yaw_rad', default_value='0.0'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_file),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params_file),
        DeclareLaunchArgument('point_lio_params_file', default_value=default_point_lio_params_file),
        DeclareLaunchArgument('startup_wait_timeout_sec', default_value='90.0'),
        DeclareLaunchArgument('nav2_activation_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('final_goal_stop_distance_m', default_value='1.0'),
        DeclareLaunchArgument('stuck_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('stuck_progress_radius_m', default_value='0.2'),
        DeclareLaunchArgument('max_recovery_attempts', default_value='1'),
        DeclareLaunchArgument('backup_distance_m', default_value='0.3'),
        DeclareLaunchArgument('backup_speed_mps', default_value='0.2'),
        DeclareLaunchArgument('spin_angle_rad', default_value='0.6'),
        DeclareLaunchArgument('recovery_time_allowance_sec', default_value='8.0'),
        DeclareLaunchArgument('progress_check_period_sec', default_value='0.2'),
        DeclareLaunchArgument('enable_scout_base', default_value='true'),
        DeclareLaunchArgument('enable_livox', default_value='true'),
        DeclareLaunchArgument('enable_lio_to_scan', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_obstacle_filter', default_value='false'),
        DeclareLaunchArgument('enable_yolo', default_value='false'),
        DeclareLaunchArgument('enable_ipm', default_value='false'),
        DeclareLaunchArgument('use_rtk_tcp_fix_bridge', default_value='true'),
        DeclareLaunchArgument('rtk_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rtk_port', default_value='9000'),
        DeclareLaunchArgument('rtk_frame_id', default_value='gps_link'),
        DeclareLaunchArgument('rtk_fix_topic', default_value='/fix'),
        DeclareLaunchArgument('rtk_default_altitude_m', default_value='0.0'),
        DeclareLaunchArgument('rtk_use_altitude_from_stream', default_value='true'),
        DeclareLaunchArgument('rtk_covariance_m2', default_value='4.0'),
        DeclareLaunchArgument('rtk_socket_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument('rtk_reconnect_interval_sec', default_value='1.0'),
        DeclareLaunchArgument('enable_pedestrian_avoidance', default_value='false'),
        DeclareLaunchArgument('pedestrian_yolo_package', default_value='yolo_detector'),
        DeclareLaunchArgument('pedestrian_yolo_executable', default_value='yolo_node'),
        DeclareLaunchArgument('pedestrian_yolo_model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('pedestrian_yolo_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('camera_package', default_value='v4l2_camera'),
        DeclareLaunchArgument('camera_executable', default_value='v4l2_camera_node'),
        DeclareLaunchArgument('rtk_package', default_value='nmea_navsat_driver'),
        DeclareLaunchArgument('rtk_executable', default_value='nmea_serial_driver'),
        DeclareLaunchArgument('obstacle_filter_package', default_value='venom_bringup'),
        DeclareLaunchArgument('obstacle_filter_executable', default_value='craic_obstacle_processor'),
        DeclareLaunchArgument('yolo_package', default_value='craic_perception'),
        DeclareLaunchArgument('yolo_executable', default_value='lane_line_detector'),
        DeclareLaunchArgument('ipm_package', default_value='craic_perception'),
        DeclareLaunchArgument('ipm_executable', default_value='ipm_lane_projector'),
    ]

    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_base_dir, 'launch', 'scout_mini_base.launch.py')
        ),
        launch_arguments={
            'port_name': 'can0',
            'is_scout_mini': 'true',
            'is_omni_wheel': 'false',
            'odom_frame': 'scout_odom',
            'odom_topic_name': 'scout_odom',
            'base_frame': 'scout_base_link',
        }.items(),
        condition=IfCondition(enable_scout_base),
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
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(enable_livox),
    )

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[point_lio_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': use_sim_time,
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
        condition=IfCondition(enable_lio_to_scan),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/global')],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', '/livox/imu'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_dir, 'launch', 'scout_mini_description.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )
    nav2_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),
        SetRemap(src='cmd_vel', dst='cmd_vel_raw'),
        SetRemap(src='/cmd_vel_smoothed', dst='/cmd_vel_raw'),
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_raw'),
        nav2_launch,
    ])

    fake_vel_transform_node = Node(
        package='venom_bringup',
        executable='fake_vel_transform',
        name='fake_vel_transform',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cmd_vel_topic': '/cmd_vel_raw',
            'output_cmd_vel_topic': '/cmd_vel',
            'local_plan_topic': '/local_plan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'fake_base_frame': 'base_link_fake',
            'spin_speed': 0.0,
            'publish_frequency_hz': 20.0,
        }],
    )

    camera_node = Node(
        package=LaunchConfiguration('camera_package'),
        executable=LaunchConfiguration('camera_executable'),
        name='camera_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_camera),
    )

    rtk_tcp_fix_bridge_node = Node(
        package='venom_bringup',
        executable='rtk_tcp_fix_bridge',
        name='rtk_fix_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'host': rtk_host,
            'port': rtk_port,
            'frame_id': rtk_frame_id,
            'fix_topic': rtk_fix_topic,
            'default_altitude_m': rtk_default_altitude_m,
            'use_altitude_from_stream': rtk_use_altitude_from_stream,
            'covariance_m2': rtk_covariance_m2,
            'socket_timeout_sec': rtk_socket_timeout_sec,
            'reconnect_interval_sec': rtk_reconnect_interval_sec,
        }],
        condition=IfCondition(
            PythonExpression([
                '"', enable_rtk, '" == "true" and "',
                use_rtk_tcp_fix_bridge, '" == "true"',
            ])
        ),
    )

    rtk_node = Node(
        package=LaunchConfiguration('rtk_package'),
        executable=LaunchConfiguration('rtk_executable'),
        name='rtk_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('fix', rtk_fix_topic), ('/fix', rtk_fix_topic)],
        condition=IfCondition(
            PythonExpression([
                '"', enable_rtk, '" == "true" and "',
                use_rtk_tcp_fix_bridge, '" != "true"',
            ])
        ),
    )

    obstacle_filter_node = Node(
        package=LaunchConfiguration('obstacle_filter_package'),
        executable=LaunchConfiguration('obstacle_filter_executable'),
        name='obstacle_filter',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'input_topic': '/cloud_registered',
                'output_topic': '/obstacle_points',
                'target_frame': 'base_link',
                'min_x_m': -1.0,
                'max_x_m': 10.0,
                'max_abs_y_m': 4.0,
                'min_z_m': 0.05,
                'max_z_m': 1.8,
                'robot_clearance_radius_m': 0.75,
                'max_range_m': 12.0,
                'voxel_size_m': 0.15,
                'max_points': 1500,
                'min_points_to_publish': 0,
                'enable_pedestrian_detections': LaunchConfiguration('enable_pedestrian_avoidance'),
                'detections_topic': '/perception/detections',
                'pedestrian_class_names': 'person,pedestrian',
            }
        ],
        remappings=[
            ('input', '/cloud_registered'),
            ('output', '/obstacle_points'),
        ],
        condition=IfCondition(enable_obstacle_filter),
    )

    yolo_lane_node = Node(
        package=LaunchConfiguration('yolo_package'),
        executable=LaunchConfiguration('yolo_executable'),
        name='lane_line_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('image_raw', '/image_raw'),
            ('lane_pixels', '/lane_pixels'),
        ],
        condition=IfCondition(enable_yolo),
    )

    pedestrian_yolo_node = Node(
        package=LaunchConfiguration('pedestrian_yolo_package'),
        executable=LaunchConfiguration('pedestrian_yolo_executable'),
        name='pedestrian_yolo_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'model_path': LaunchConfiguration('pedestrian_yolo_model_path'),
                'image_topic': LaunchConfiguration('pedestrian_yolo_image_topic'),
                'output_topic': '/perception/detections',
                'class_ids': '0',
                'confidence_threshold': 0.45,
                'publish_annotated_image': False,
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('pedestrian_yolo_image_topic')),
            ('detections', '/perception/detections'),
        ],
        condition=IfCondition(enable_pedestrian_avoidance),
    )

    ipm_lane_projector_node = Node(
        package=LaunchConfiguration('ipm_package'),
        executable=LaunchConfiguration('ipm_executable'),
        name='ipm_lane_projector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('lane_pixels', '/lane_pixels'),
            ('virtual_lane_points', '/virtual_lane_points'),
        ],
        condition=IfCondition(enable_ipm),
    )

    mission_node = Node(
        package='venom_bringup',
        executable='craic_mission_main',
        name='craic_mission_main',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'waypoint_file': waypoint_file,
                'road_network_file': road_network_file,
                'route_name': route_name,
                'route_nodes': route_nodes,
                'start_node_id': start_node_id,
                'goal_node_id': goal_node_id,
                'start_x_m': start_x_m,
                'start_y_m': start_y_m,
                'goal_x_m': goal_x_m,
                'goal_y_m': goal_y_m,
                'use_start_goal_xy': use_start_goal_xy,
                'blocked_edges': blocked_edges,
                'coordinate_mode': coordinate_mode,
                'map_origin_longitude_deg': map_origin_longitude_deg,
                'map_origin_latitude_deg': map_origin_latitude_deg,
                'map_origin_x_m': map_origin_x_m,
                'map_origin_y_m': map_origin_y_m,
                'map_origin_yaw_rad': map_origin_yaw_rad,
                'pose_tracking_topic': '/odometry/global',
                'cmd_vel_topic': '/cmd_vel_raw',
                'waypoint_frame_id': 'map',
                'robot_base_frame': 'base_link_fake',
                'use_first_waypoint_as_origin': True,
                'startup_wait_timeout_sec': startup_wait_timeout_sec,
                'nav2_activation_timeout_sec': nav2_activation_timeout_sec,
                'final_goal_stop_distance_m': final_goal_stop_distance_m,
                'stuck_timeout_sec': stuck_timeout_sec,
                'stuck_progress_radius_m': stuck_progress_radius_m,
                'max_recovery_attempts': max_recovery_attempts,
                'backup_distance_m': backup_distance_m,
                'backup_speed_mps': backup_speed_mps,
                'spin_angle_rad': spin_angle_rad,
                'recovery_time_allowance_sec': recovery_time_allowance_sec,
                'progress_check_period_sec': progress_check_period_sec,
            }
        ],
    )

    delayed_mission_node = TimerAction(period=5.0, actions=[mission_node])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(venom_bringup_dir, 'rviz_cfg', 'mapping.rviz')],
        output='screen',
        condition=UnlessCondition(headless),
    )

    return LaunchDescription(
        arguments
        + [
            scout_base_launch,
            livox_driver_node,
            camera_node,
            rtk_node,
            rtk_tcp_fix_bridge_node,
            robot_description_launch,
            point_lio_node,
            pointcloud_to_laserscan_node,
            obstacle_filter_node,
            yolo_lane_node,
            pedestrian_yolo_node,
            ipm_lane_projector_node,
            ekf_node,
            navsat_transform_node,
            fake_vel_transform_node,
            nav2_group,
            delayed_mission_node,
            rviz_node,
        ]
    )
