# 运行依赖说明

本文档说明比赛项目源码与原厂、第三方运行环境之间的边界。仓库只提交项目开发和修改的源文件，不重复提交完整 WHEELTEC 原厂工作空间及可通过构建重新生成的文件。

## 已验证环境

以下环境已于 2026 年 7 月 14 日在比赛小车上核对：

| 项目 | 已验证环境 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS |
| CPU 架构 | aarch64 |
| ROS 发行版 | ROS 2 Humble |
| Python | Python 3.10 |
| OpenCV | 5.0.0 |
| NumPy | 1.26.4 |
| Ultralytics | 8.4.90 |
| TensorRT | 10.3.0 |
| 底盘 | WHEELTEC mini Ackermann |
| RGB-D 相机 | Astra/Orbbec |
| LiDAR | WHEELTEC M10-P，使用 `lslidar_driver` |

上述版本用于记录当前比赛环境，不表示项目只能使用完全相同的补丁版本。TensorRT engine 与 GPU 架构、CUDA 和 TensorRT 版本相关，迁移设备时应优先使用 `best2.pt` 或 `best2.onnx` 重新导出。

## 仓库内源码

本仓库直接维护以下项目内容：

| 路径 | 用途 |
| --- | --- |
| `src/turn_on_wheeltec_robot` | 底盘集成、相机和前端启动、交通目标识别、视觉导航速度过滤。 |
| `src/wheeltec_clean_nav` | AMCL、Nav2、地图、路径跟踪、建图和轨迹工具。 |
| `deployment_scripts` | 三阶段启动、统一停止、systemd 服务和建图脚本。 |

`turn_on_wheeltec_robot` 保留了 WHEELTEC 原厂包结构。项目新增和修改文件已在主 README 中单独列明。

## 原厂工作空间依赖

以下 ROS 包由比赛小车现有 WHEELTEC ROS 2 Humble 工作空间提供，不属于本项目自研源码：

| ROS 包 | 用途 |
| --- | --- |
| `wheeltec_robot_msg` | 原厂底盘消息定义。 |
| `serial` | 底盘串口通信库。 |
| `wheeltec_robot_urdf` | 当前原厂启动文件使用的车型描述；不参与路径规划算法。 |
| `astra_camera`、`astra_camera_msgs` | Astra RGB 和深度图像驱动。 |
| `lslidar_driver`、`lslidar_msgs` | M10-P LiDAR 驱动和消息。 |
| `rosbridge_server`、`rosapi` | 浏览器前端与 ROS 图通信。 |
| `web_video_server` | 浏览器 MJPEG 图像流。 |

在原比赛小车上，这些包已经位于 `/home/wheeltec/wheeltec_ros2/install` 或系统 ROS 安装目录。若在全新的 ROS 2 Humble 系统上部署，必须先从 WHEELTEC 官方工作空间、对应上游项目或系统软件源安装这些依赖。

## ROS 与系统依赖

运行导航和传感器流程还需要：

- Nav2 与 `nav2_bringup`
- AMCL 和 Map Server
- SLAM Toolbox
- `robot_localization`
- `imu_filter_madgwick`
- `robot_state_publisher`、`joint_state_publisher`
- `tf2_ros`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、`std_msgs`
- OpenCV、NumPy、PyYAML
- Astra/Orbbec SDK 及对应 udev 设备规则
- CUDA 与 TensorRT 运行库（加载 `best2.engine` 时）
- Ultralytics/PyTorch（直接加载 `best2.pt` 时）

仅安装基础 ROS 2 Humble 并不足以驱动底盘、相机、LiDAR 和 TensorRT 识别。比赛仓库面向已经安装原厂硬件环境的 WHEELTEC 小车。

## 构建

在依赖已经满足的 WHEELTEC 工作空间中执行：

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_wheeltec_robot wheeltec_clean_nav
source install/setup.bash
```

`colcon build` 会在工作空间根目录生成：

| 目录 | 内容 | 是否提交 Git |
| --- | --- | --- |
| `build/` | CMake、Python 和编译中间文件 | 否 |
| `install/` | 可被 ROS 2 加载的安装结果 | 否 |
| `log/` | 本次及历史构建日志 | 否 |

删除这些目录不会删除源代码。删除 `build/` 和 `log/` 后可以直接重新构建；删除正在使用的 `install/` 后，必须成功执行一次 `colcon build` 才能再次启动对应节点。

## 提交范围

比赛代码提交包含源代码、启动文件、参数、地图、模型和部署文档。以下内容不属于源码提交范围：

```text
build/
install/
log/
.Trash-1000/
__pycache__/
*.pyc
运行日志
调试备份
完整原厂工作空间
```

这种提交方式保留项目开发成果和复现信息，同时避免上传可重新生成的编译产物及未修改的第三方代码。
