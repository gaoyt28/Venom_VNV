"""Standalone RTK TCP -> /fix bridge launcher."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    frame_id = LaunchConfiguration('frame_id')
    fix_topic = LaunchConfiguration('fix_topic')
    default_altitude_m = LaunchConfiguration('default_altitude_m')
    use_altitude_from_stream = LaunchConfiguration('use_altitude_from_stream')
    covariance_m2 = LaunchConfiguration('covariance_m2')
    socket_timeout_sec = LaunchConfiguration('socket_timeout_sec')
    reconnect_interval_sec = LaunchConfiguration('reconnect_interval_sec')

    bridge_node = Node(
        package='venom_bringup',
        executable='rtk_tcp_fix_bridge',
        name='rtk_fix_bridge',
        output='screen',
        parameters=[{
            'host': host,
            'port': port,
            'frame_id': frame_id,
            'fix_topic': fix_topic,
            'default_altitude_m': default_altitude_m,
            'use_altitude_from_stream': use_altitude_from_stream,
            'covariance_m2': covariance_m2,
            'socket_timeout_sec': socket_timeout_sec,
            'reconnect_interval_sec': reconnect_interval_sec,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('frame_id', default_value='gps_link'),
        DeclareLaunchArgument('fix_topic', default_value='/fix'),
        DeclareLaunchArgument('default_altitude_m', default_value='0.0'),
        DeclareLaunchArgument('use_altitude_from_stream', default_value='true'),
        DeclareLaunchArgument('covariance_m2', default_value='4.0'),
        DeclareLaunchArgument('socket_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument('reconnect_interval_sec', default_value='1.0'),
        bridge_node,
    ])
