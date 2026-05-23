"""Launch RTKLIB SOL to Odometry bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    odom_topic = LaunchConfiguration('odom_topic')
    navsat_topic = LaunchConfiguration('navsat_topic')
    frame_id = LaunchConfiguration('frame_id')
    child_frame_id = LaunchConfiguration('child_frame_id')
    origin_mode = LaunchConfiguration('origin_mode')
    origin_latitude_deg = LaunchConfiguration('origin_latitude_deg')
    origin_longitude_deg = LaunchConfiguration('origin_longitude_deg')
    origin_height_m = LaunchConfiguration('origin_height_m')
    map_origin_yaw_rad = LaunchConfiguration('map_origin_yaw_rad')
    max_solution_quality = LaunchConfiguration('max_solution_quality')
    publish_tf = LaunchConfiguration('publish_tf')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/global'),
        DeclareLaunchArgument('navsat_topic', default_value='/rtk/fix'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('child_frame_id', default_value='base_link'),
        DeclareLaunchArgument('origin_mode', default_value='auto'),
        DeclareLaunchArgument('origin_latitude_deg', default_value='0.0'),
        DeclareLaunchArgument('origin_longitude_deg', default_value='0.0'),
        DeclareLaunchArgument('origin_height_m', default_value='0.0'),
        DeclareLaunchArgument('map_origin_yaw_rad', default_value='0.0'),
        DeclareLaunchArgument('max_solution_quality', default_value='2'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(
            package='venom_bringup',
            executable='rtk_sol_odom_node',
            name='rtk_sol_odom_node',
            output='screen',
            parameters=[{
                'host': host,
                'port': port,
                'odom_topic': odom_topic,
                'navsat_topic': navsat_topic,
                'frame_id': frame_id,
                'child_frame_id': child_frame_id,
                'origin_mode': origin_mode,
                'origin_latitude_deg': origin_latitude_deg,
                'origin_longitude_deg': origin_longitude_deg,
                'origin_height_m': origin_height_m,
                'map_origin_yaw_rad': map_origin_yaw_rad,
                'max_solution_quality': max_solution_quality,
                'publish_tf': publish_tf,
            }],
        ),
    ])
