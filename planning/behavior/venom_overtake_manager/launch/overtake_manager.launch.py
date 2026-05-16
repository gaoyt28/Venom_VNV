from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=''),
        Node(
            package='venom_overtake_manager',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='venom_overtake_manager',
            executable='lead_selector_node',
            name='lead_selector_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='venom_overtake_manager',
            executable='overtake_manager_node',
            name='overtake_manager_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
