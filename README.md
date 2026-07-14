# WHEELTEC ROS 2 视觉感知与导航系统

本仓库是 WHEELTEC mini Ackermann 小车的比赛提交版本，包含交通标志与信号灯识别、RGB/深度相机管理、低速 Nav2 导航、速度安全过滤、比赛场地图以及部署脚本。

仓库只保留比赛功能相关源码，不包含完整原厂工作区、编译产物、运行日志、训练数据和调试备份。运行时仍依赖小车已有的 WHEELTEC ROS 2 Humble 环境及原厂传感器驱动。

完整的原厂、第三方和系统依赖边界见 [`DEPENDENCIES.md`](DEPENDENCIES.md)。

## 功能概览

- Astra RGB 与深度图像采集
- YOLOv8/TensorRT 目标识别
- 交通灯、停车标志、慢行标志和行人状态输出
- Nav2 路径规划与 Regulated Pure Pursuit 路径跟踪
- 交通状态与导航速度融合
- 最大线速度 `0.12 m/s`
- 禁止导航倒车和自动运动恢复
- AMCL 定位、LiDAR 障碍物检测与 RViz 可视化
- 三阶段安全启动和统一停止
- SLAM 建图、轨迹记录与地图保存
- Rosbridge 与 Web 视频前端支持

## 仓库结构

```text
wheeltec_ros2/
├── README.md
├── .gitignore
├── src/
│   ├── turn_on_wheeltec_robot/              [原厂包，包含项目修改]
│   │   ├── launch/
│   │   │   ├── traffic_light_nodes.launch.py
│   │   │   ├── traffic_light_perception.launch.py
│   │   │   ├── traffic_light_color_debug.launch.py
│   │   │   ├── wheeltec_camera.launch.py
│   │   │   └── wheeltec_frontend.launch.py
│   │   ├── scripts/
│   │   │   ├── traffic_light_debug_node.py
│   │   │   ├── traffic_light_velocity_node.py
│   │   │   └── traffic_light_nav_filter_node.py
│   │   └── model/
│   │       ├── best2.pt
│   │       ├── best2.onnx
│   │       └── best2.engine
│   └── wheeltec_clean_nav/                  [项目自研 ROS 2 包]
│       ├── behavior_trees/
│       ├── config/
│       ├── launch/
│       ├── maps/
│       ├── rviz/
│       └── wheeltec_clean_nav/
└── deployment_scripts/                      [项目部署脚本]
    ├── start_perception_navigation.sh
    ├── start_chassis_only.sh
    ├── start_navi.sh
    ├── stop_traffic_light.sh
    ├── start_clean_mapping.sh
    ├── save_clean_map.sh
    ├── start_mapping_rviz.sh
    ├── wheeltec_frontend_service.sh
    ├── wheeltec_perception_service.sh
    └── systemd/
        ├── wheeltec-frontend.service
        └── wheeltec-perception.service
```

## 自研与修改内容

### `wheeltec_clean_nav`

本项目新增的导航包，未依赖旧的比赛导航实现。主要职责如下：

| 模块 | 说明 |
| --- | --- |
| `clean_navigation.launch.py` | 分阶段启动 LiDAR、Map Server、AMCL、Nav2、速度过滤器和场地标记。 |
| `clean_mapping.launch.py` | 启动 SLAM Toolbox 和轨迹记录器。 |
| `nav2_params.yaml` | 配置 AMCL、代价地图、Smac Hybrid-A*、RPP 控制器和生命周期节点。 |
| `navigate_to_pose_forward_only.xml` | 单目标前进导航行为树，不包含倒车、旋转和自动移动恢复。 |
| `course_markers.py` | 在 RViz 显示斑马线、方向箭头和建议起点，不影响代价地图。 |
| `generate_course_map.py` | 按比赛场地尺寸生成几何占据地图。 |
| `trajectory_recorder.py` | 建图时记录小车里程计轨迹。 |
| `generate_driven_map.py` | 根据轨迹生成仅允许已行驶区域的辅助地图。 |
| `route_runner.py` | 可选航点执行节点；正式启动默认不启用自动路线。 |

### `turn_on_wheeltec_robot`

该目录保留原厂底盘包结构，并加入项目视觉与集成功能。原厂底盘串口、IMU、EKF 和基础 launch 代码仍归 WHEELTEC 原作者所有。

项目新增或修改的主要文件：

| 文件 | 说明 |
| --- | --- |
| `traffic_light_debug_node.py` | 订阅 RGB/深度图像，执行目标识别、距离判断和交通状态输出。 |
| `traffic_light_velocity_node.py` | 将识别状态转换为测试速度，默认不直接连接底盘。 |
| `traffic_light_nav_filter_node.py` | 将 Nav2 速度与 `GO/SLOW/STOP` 状态融合，限速并输出 `/cmd_vel`。 |
| `traffic_light_nodes.launch.py` | 启动完整识别节点和测试速度节点。 |
| `traffic_light_perception.launch.py` | 仅启动识别，供导航集成流程使用。 |
| `wheeltec_camera.launch.py` | 统一管理 Astra RGB 和深度相机。 |
| `wheeltec_frontend.launch.py` | 启动底盘、IMU、EKF、Rosbridge 和 Web 视频服务。 |

### `deployment_scripts`

部署脚本不属于 ROS 包，安装在 `/home/wheeltec/`。它们负责服务启动顺序、节点状态检查、零速度保护以及统一停止。

## 数据流

```text
Astra RGB/depth
       |
       v
traffic_light_debug_node
       |
       +----> /traffic_light/state (GO / SLOW / STOP)
       |
       +----> /traffic_light/debug_image

Nav2 ----> /nav2/cmd_vel
                    \
                     +--> traffic_light_nav_filter --> /cmd_vel --> chassis
/traffic_light/state /
```

`/cmd_vel` 的正式发布者应只有 `traffic_light_nav_filter`。识别状态超时、导航速度超时或 `STOP` 状态都会输出零速度。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- WHEELTEC mini Ackermann 底盘
- Astra RGB-D 相机
- WHEELTEC M10-P LiDAR
- OpenCV、NumPy、PyYAML、rclpy
- Nav2、SLAM Toolbox、robot_localization
- CUDA 与 TensorRT（使用 `.engine` 时）
- Ultralytics（使用 `.pt` 时）

## 获取与构建

将仓库放在小车工作区：

```bash
cd /home/wheeltec
git clone https://github.com/Zongjue-Liu/wheeltec_ros2.git
cd wheeltec_ros2
```

在已安装原厂依赖的 WHEELTEC 环境中构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_wheeltec_robot wheeltec_clean_nav
source install/setup.bash
```

本仓库不是完整原厂工作区。如果在其他电脑构建，需要另外安装或提供 Astra、LiDAR、Nav2、Rosbridge、Web Video Server、底盘描述包等依赖。

`colcon build` 会重新生成 `build/`、`install/` 和 `log/`。这些目录不属于源码，不应提交到 Git；具体说明见 [`DEPENDENCIES.md`](DEPENDENCIES.md)。

## 部署脚本

将脚本安装到小车用户目录：

```bash
cd /home/wheeltec/wheeltec_ros2
install -m 755 deployment_scripts/*.sh /home/wheeltec/
```

安装 systemd 服务：

```bash
sudo install -m 644 deployment_scripts/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wheeltec-frontend.service
sudo systemctl enable wheeltec-perception.service
```

服务职责：

| 服务 | 说明 |
| --- | --- |
| `wheeltec-frontend.service` | 底盘、IMU、EKF、Rosbridge 和 Web 视频。 |
| `wheeltec-perception.service` | Astra RGB/深度相机和交通目标识别。 |

脚本不保存 sudo 密码。首次执行需要在终端输入系统密码。

## 三阶段启动

将小车放在比赛地图内，并在三个阶段之间等待成功提示。

### 1. 相机与识别

```bash
/home/wheeltec/start_perception_navigation.sh
```

该阶段启动前端服务、Astra RGB/深度相机和识别节点，等待 `/traffic_light/state` 首条消息。TensorRT 首次加载可能需要约一分钟。

### 2. 底盘确认

```bash
/home/wheeltec/start_chassis_only.sh
```

该阶段确认底盘订阅 `/cmd_vel`，释放串口零速度保持进程，并再次发送零速度。

### 3. 导航

```bash
/home/wheeltec/start_navi.sh
```

该阶段启动 LiDAR、Map Server、AMCL、Nav2 和交通状态速度过滤器。默认不发送目标，也不自动执行路线。

## RViz 定位与目标点

启动导航后打开 RViz：

```bash
rviz2 -d /home/wheeltec/wheeltec_ros2/install/wheeltec_clean_nav/share/wheeltec_clean_nav/rviz/clean_navigation.rviz
```

操作顺序：

1. 使用 `2D Pose Estimate` 设置小车在地图中的真实位置。
2. 按住鼠标并沿真实车头方向拖动后松开。
3. 确认 LaserScan 与地图围栏边缘重合。
4. 使用 `2D Goal Pose` 设置目标位置和目标朝向。
5. 确认绿色全局路径和蓝色局部路径合理后再观察车辆运动。

在设置初始位姿前，RViz 可能显示：

```text
Frame [map] does not exist
```

这是 AMCL 尚未建立 `map -> odom_combined` 的正常等待状态。设置正确的初始位姿后即可消失，与是否设置目标点无关。

比赛地图四角结构相似，错误的初始位姿可能使 AMCL 收敛到另一个角落。若 RViz 车体位置与实体位置不一致，不得发送目标，应重新设置初始位姿并检查雷达重合。

## 比赛地图

正式地图位于：

```text
src/wheeltec_clean_nav/maps/WHEELTEC.yaml
src/wheeltec_clean_nav/maps/WHEELTEC.pgm
```

地图参数：

- 实体场地：`3.23 m x 3.55 m`
- 分辨率：`0.01 m/cell`
- ROS 地图白色区域：实体黑色可行道路
- ROS 地图黑色区域：外侧围栏和四块禁行区域
- 建议起点：左下道路，车头朝地图 `+Y`

斑马线和方向箭头通过 `/course_markers` 显示，仅用于 RViz 参考，不参与碰撞判断。

按代码中的场地尺寸重新生成已安装地图：

```bash
ros2 run wheeltec_clean_nav generate_course_map
```

需要写入指定目录时：

```bash
ros2 run wheeltec_clean_nav generate_course_map --output-dir /path/to/maps
```

## 统一停止

任意阶段均可执行：

```bash
/home/wheeltec/stop_traffic_light.sh
```

停止脚本会：

1. 发布多轮零速度。
2. 向底盘串口发送零速度帧。
3. 停止导航、识别、相机和前端服务。
4. 启动串口零速度保持进程。

发生定位错误、路径异常、相机冲突或车辆行为异常时，应立即执行该命令。

## 建图

启动建图：

```bash
/home/wheeltec/start_clean_mapping.sh
```

打开建图 RViz：

```bash
/home/wheeltec/start_mapping_rviz.sh
```

使用手机前端或遥控节点低速绕行固定场地，完成外围闭环和中间道路覆盖后保存：

```bash
/home/wheeltec/save_clean_map.sh
```

建图期间应保留固定围栏、固定障碍物和固定指示牌；人员和其他移动物体应离开 LiDAR 扫描区域。

## 前端连接

前端服务启动后：

```text
Rosbridge: ws://<robot-ip>:9090
Web video: http://<robot-ip>:8080
```

仓库不保存小车 IP、系统密码或其他凭据。

## 模型文件

| 文件 | 用途 |
| --- | --- |
| `best2.pt` | PyTorch/Ultralytics 训练权重。 |
| `best2.onnx` | ONNX 导出模型。 |
| `best2.engine` | 当前比赛小车使用的 TensorRT engine。 |

TensorRT engine 与 GPU 架构、CUDA、TensorRT 版本有关。在其他设备上可能无法直接加载，应使用 `.pt` 或 `.onnx` 重新生成。

## 已知限制

- 初始位姿必须由操作员根据实体位置设置。
- 发送目标前必须确认雷达与地图重合。
- 对称场地可能导致错误定位，不能仅凭地图外观判断。
- 导航参数按当前小车和比赛场地低速调试，不保证直接适用于其他底盘或地图。
- 自动航点路线默认关闭。
- TensorRT engine 不保证跨设备兼容。
- 本仓库不包含完整 WHEELTEC 原厂依赖。

## 权利声明

本仓库用于比赛提交、评审和项目复现，不作为开源软件发布。自研代码和模型采用 `Proprietary` 许可，未经维护者许可，不授予额外的修改、分发或商业使用权。

仓库中保留的 WHEELTEC 原厂代码及第三方组件，其权利归各自原作者或权利人所有。比赛组织方可为评审和复现目的查看、下载和运行本仓库。

维护者：Zongjue Liu
联系邮箱：2215873441@qq.com
