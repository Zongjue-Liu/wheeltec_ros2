import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    clean_dir = get_package_share_directory('wheeltec_clean_nav')
    nav2_dir = get_package_share_directory('nav2_bringup')
    lidar_dir = get_package_share_directory('lslidar_driver')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    start_lidar = LaunchConfiguration('start_lidar')
    start_route = LaunchConfiguration('start_route')
    forward_only_bt = os.path.join(
        clean_dir, 'behavior_trees', 'navigate_to_pose_forward_only.xml')
    forward_only_through_poses_bt = os.path.join(
        clean_dir, 'behavior_trees', 'navigate_through_poses_forward_only.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'default_nav_to_pose_bt_xml': forward_only_bt,
            'default_nav_through_poses_bt_xml': forward_only_through_poses_bt,
        },
        convert_types=True,
    )

    # The vendor M10-P driver has occasionally aborted after serial read errors.
    # Restart the lidar driver after transient serial failures.
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
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': configured_params,
            'namespace': namespace,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    navigation = GroupAction([
        SetRemap(src='cmd_vel', dst='/nav2/cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': configured_params,
                'namespace': namespace,
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])
    # Let map_server and AMCL finish lifecycle activation before the heavier
    # controller and Hybrid-A* nodes start configuring.
    delayed_navigation = TimerAction(period=8.0, actions=[navigation])
    velocity_filter = Node(
        package='turn_on_wheeltec_robot',
        executable='traffic_light_nav_filter_node.py',
        name='traffic_light_nav_filter',
        output='screen',
        parameters=[{
            'nav_cmd_topic': '/nav2/cmd_vel',
            'state_topic': '/traffic_light/state',
            'output_topic': '/cmd_vel',
            'allow_reverse': False,
            'max_linear_speed': 0.12,
            'slow_linear_speed': 0.05,
            'max_angular_speed': 0.45,
        }],
    )
    route = Node(
        package='wheeltec_clean_nav',
        executable='route_runner',
        name='clean_route_runner',
        output='screen',
        condition=IfCondition(start_route),
    )
    course_markers = Node(
        package='wheeltec_clean_nav',
        executable='course_markers',
        name='course_markers',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(clean_dir, 'maps', 'WHEELTEC.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(clean_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_route', default_value='false'),
        lidar,
        localization,
        delayed_navigation,
        velocity_filter,
        course_markers,
        route,
    ])
