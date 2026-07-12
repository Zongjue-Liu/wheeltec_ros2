from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    conf = LaunchConfiguration("conf")
    green_hold_sec = LaunchConfiguration("green_hold_sec")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    use_depth = LaunchConfiguration("use_depth")
    stop_distance_m = LaunchConfiguration("stop_distance_m")
    slow_distance_m = LaunchConfiguration("slow_distance_m")
    traffic_light_distance_m = LaunchConfiguration("traffic_light_distance_m")
    pedestrian_distance_m = LaunchConfiguration("pedestrian_distance_m")
    no_target_state = LaunchConfiguration("no_target_state")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/wheeltec/wheeltec_ros2/src/turn_on_wheeltec_robot/model/best2.engine",
            ),
            DeclareLaunchArgument("conf", default_value="0.4"),
            DeclareLaunchArgument("green_hold_sec", default_value="0.8"),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument("use_depth", default_value="true"),
            DeclareLaunchArgument("stop_distance_m", default_value="0.53"),
            DeclareLaunchArgument("slow_distance_m", default_value="0.53"),
            DeclareLaunchArgument(
                "traffic_light_distance_m", default_value="0.53"
            ),
            DeclareLaunchArgument("pedestrian_distance_m", default_value="0.53"),
            DeclareLaunchArgument("no_target_state", default_value="GO"),
            # Start perception only. Navigation speed is handled by the separate
            # traffic_light_nav_filter node, so the legacy velocity node is not started.
            Node(
                package="turn_on_wheeltec_robot",
                executable="traffic_light_debug_node.py",
                name="traffic_light_debug_node",
                output="screen",
                arguments=[
                    "--model",
                    model_path,
                    "--conf",
                    conf,
                    "--green-only",
                    "--green-hold-sec",
                    green_hold_sec,
                    "--image-topic",
                    image_topic,
                    "--depth-topic",
                    depth_topic,
                    "--use-depth",
                    use_depth,
                    "--stop-distance-m",
                    stop_distance_m,
                    "--slow-distance-m",
                    slow_distance_m,
                    "--traffic-light-distance-m",
                    traffic_light_distance_m,
                    "--pedestrian-distance-m",
                    pedestrian_distance_m,
                    "--no-target-state",
                    no_target_state,
                ],
            ),
        ]
    )
