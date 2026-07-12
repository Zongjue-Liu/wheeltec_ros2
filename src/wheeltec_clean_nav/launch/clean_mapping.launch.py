import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clean_dir = get_package_share_directory('wheeltec_clean_nav')
    lidar_dir = get_package_share_directory('lslidar_driver')
    params_file = LaunchConfiguration('params_file')
    start_lidar = LaunchConfiguration('start_lidar')
    trajectory_file = LaunchConfiguration('trajectory_file')

    lidar = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        condition=IfCondition(start_lidar),
        parameters=[os.path.join(
            lidar_dir, 'params', 'lidar_uart_ros2', 'lsm10_p.yaml')],
        respawn=True,
        respawn_delay=3.0,
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
    )
    trajectory_recorder = Node(
        package='wheeltec_clean_nav',
        executable='trajectory_recorder',
        name='mapping_trajectory_recorder',
        output='screen',
        parameters=[{
            'output_file': ParameterValue(trajectory_file, value_type=str),
            'min_distance': 0.02,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                clean_dir, 'config', 'slam_toolbox_mapping.yaml')),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'trajectory_file',
            default_value='/home/wheeltec/mapping_sessions/current_odom_path.csv'),
        lidar,
        slam,
        trajectory_recorder,
    ])
