from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    conf = LaunchConfiguration("conf")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/wheeltec/wheeltec_ros2/src/turn_on_wheeltec_robot/model/best2.engine",
            ),
            DeclareLaunchArgument("conf", default_value="0.4"),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            Node(
                package="turn_on_wheeltec_robot",
                executable="traffic_light_debug_node.py",
                name="traffic_light_color_debug_node",
                output="screen",
                arguments=[
                    "--model",
                    model_path,
                    "--conf",
                    conf,
                    "--image-topic",
                    image_topic,
                    "--use-depth",
                    "false",
                    "--traffic-light-only",
                ],
            ),
        ]
    )
