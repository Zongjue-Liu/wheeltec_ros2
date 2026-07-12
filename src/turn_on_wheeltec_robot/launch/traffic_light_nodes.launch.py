from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    output_topic = LaunchConfiguration("output_topic")
    cruise_speed = LaunchConfiguration("cruise_speed")
    slow_speed = LaunchConfiguration("slow_speed")
    accel_limit = LaunchConfiguration("accel_limit")
    decel_limit = LaunchConfiguration("decel_limit")
    state_timeout_sec = LaunchConfiguration("state_timeout_sec")

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
            DeclareLaunchArgument("traffic_light_distance_m", default_value="0.53"),
            DeclareLaunchArgument("pedestrian_distance_m", default_value="0.53"),
            DeclareLaunchArgument("no_target_state", default_value="GO"),
            DeclareLaunchArgument(
                "output_topic", default_value="/traffic_light/cmd_vel_test"
            ),
            DeclareLaunchArgument("cruise_speed", default_value="0.12"),
            DeclareLaunchArgument("slow_speed", default_value="0.05"),
            DeclareLaunchArgument("accel_limit", default_value="0.08"),
            DeclareLaunchArgument("decel_limit", default_value="0.30"),
            DeclareLaunchArgument("state_timeout_sec", default_value="1.5"),
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
            Node(
                package="turn_on_wheeltec_robot",
                executable="traffic_light_velocity_node.py",
                name="traffic_light_velocity_node",
                output="screen",
                parameters=[
                    {
                        "state_topic": "/traffic_light/state",
                        "output_topic": output_topic,
                        "publish_hz": 20.0,
                        "cruise_speed": ParameterValue(
                            cruise_speed, value_type=float
                        ),
                        "slow_speed": ParameterValue(slow_speed, value_type=float),
                        "accel_limit": ParameterValue(accel_limit, value_type=float),
                        "decel_limit": ParameterValue(decel_limit, value_type=float),
                        "state_timeout_sec": ParameterValue(
                            state_timeout_sec, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
