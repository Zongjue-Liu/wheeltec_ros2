"""
Wheeltec 小车前端控制一体化启动文件

支持两种摄像头：
  - UVC USB 摄像头（如 Sonix USB 2.0 Camera）→ /image_raw
  - Astra 深度摄像头 → /camera/color/image_raw

用法：
  ros2 launch turn_on_wheeltec_robot wheeltec_frontend.launch.py
  ros2 launch turn_on_wheeltec_robot wheeltec_frontend.launch.py enable_camera:=true
  ros2 launch turn_on_wheeltec_robot wheeltec_frontend.launch.py enable_camera:=true camera_type:=usb
"""

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def camera_condition(enable_var, type_var, target_type):
    """生成条件：enable_camera==true AND camera_type==target_type"""
    return IfCondition(
        PythonExpression([
            '"', enable_var, '" == "true" and "', type_var, '" == "', target_type, '"'
        ])
    )


def generate_launch_description():
    turn_on_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(turn_on_dir, 'launch')

    enable_camera = LaunchConfiguration("enable_camera", default='true')
    camera_type = LaunchConfiguration('camera_type', default='usb')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='是否启动摄像头节点'
    )
    camera_type_arg = DeclareLaunchArgument(
        'camera_type', default_value='usb',
        description='摄像头类型: usb (UVC USB 摄像头) 或 astra (Astra/Orbbec 深度相机)'
    )

    # 1. 底盘驱动
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')
        ),
    )

    # 2. rosbridge WebSocket（端口 9090）
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090, 'address': '0.0.0.0', 'retry_startup_delay': 5.0}],
    )

    # 3. rosapi 节点
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    # 4. web_video_server（MJPEG 流，端口 8080）
    web_video_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080, 'address': '0.0.0.0'}],
    )

    # 5. UVC USB 摄像头（话题: /image_raw）
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'camera_name': 'rgb_camera',
            'frame_id': 'rgb_camera',
            'camera_info_url': 'file:///home/wheeltec/wheeltec_ros2/config/astra_s_rgb_640x480.yaml',
 
        }],
        condition=camera_condition(enable_camera, camera_type, 'usb'),
    )

    ld = LaunchDescription()
    ld.add_action(enable_camera_arg)
    ld.add_action(camera_type_arg)
    ld.add_action(wheeltec_robot)
    ld.add_action(rosbridge_node)
    ld.add_action(rosapi_node)
    ld.add_action(web_video_node)
    ld.add_action(usb_cam_node)

    # 6. Astra 深度摄像头（话题: /camera/color/image_raw — 需硬件 + 已编译）
    try:
        get_package_share_directory('astra_camera')
        astra_cam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'wheeltec_camera.launch.py'),
            ),
            condition=camera_condition(enable_camera, camera_type, 'astra'),
        )
        ld.add_action(astra_cam)
        ld.add_action(LogInfo(msg='可用 camera_type:=astra 切换 Astra 深度相机'))
    except PackageNotFoundError:
        ld.add_action(LogInfo(msg='astra_camera 未编译，仅支持 usb 摄像头'))

    return ld
