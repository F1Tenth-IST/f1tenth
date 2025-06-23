import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node



def generate_launch_description():

    zed_config = os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'odom_zed2i.yaml')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_xacro_path = os.path.join(zed_wrapper_dir, 'urdf', 'zed_descr.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=zed_config,
            description='Path to the config file'
        ),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=zed_xacro_path,
            description='Path to the ZED xacro URDF file.'
        ),
        # Robot State Publisher (to publish TFs from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='zed_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', LaunchConfiguration('xacro_path')
                ])
            }]
        ),
        # Composable container with ZED node
        ComposableNodeContainer(
            name='zed_container',
            namespace='zed',
            package='rclcpp_components',
            executable='component_container_isolated',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            composable_node_descriptions=[
                ComposableNode(
                    package='zed_components',
                    plugin='stereolabs::ZedCamera',
                    name='zed_node',
                    parameters=[LaunchConfiguration('config_file')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ]
        )
    ])
