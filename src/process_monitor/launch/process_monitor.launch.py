from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('process_monitor'),
        'config',
        'process_monitor.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Full path to the process_monitor parameter file.',
    )

    config_file = LaunchConfiguration('config_file')

    process_monitor_node = Node(
        package='process_monitor',
        executable='process_monitor',
        name='process_monitor',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        process_monitor_node,
    ])
