<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
</div>

<p align="center">
  <img src="assets/images/demo1.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo2.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo3.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

> 二编：增加一个map.pcd，并提供测试例子（非搭建gazebo测试）

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成。



## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

---

### 1.1、在一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```
### 1.2、在另一个终端启动自定义确定性仿真器与规划器（Mode 1 二维闭环演示，机身高度保持不变）：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```
---


### 2.1、在一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```
### 2.2、使用 `map.pcd` 运行 Mode 3 跨层开环演示(注意替换自己的路径)：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=/your_map_location/map.pcd \
  reference_path_file:=/your_project_location/src/planner/plan_manage/config/reference_path.map.yaml
```

示例参考路径从 `(-5.5, 5.5, 0.10)` 沿地图坡道上升到
`(-5.5, -4.5, 1.55)`。路径文件中的 `z` 是地面/路线高度，规划器会再加上
`grid_map.body_height`（默认 `0.4 m`），因此目标机身高度约为 `1.95 m`。


RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



### 导航模式说明
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `initial_path` 话题获取全局路径，并在局部范围内进行避障

控制器模式分为 `open_loop`（开环）和 `closed_loop`（闭环）两种。本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map` 和 `pcd_map_file`。

与原作者实现保持一致，`closed_loop` 通过平面 `cmd_vel` 跟踪 `x/y/yaw`，
适用于二维仿真或真机底盘接口；多楼层仿真应使用 `open_loop`，直接按规划得到的
三维 B-spline 发布里程计。Mode 3 默认从 `(-5.5, 5.5, 0.5)` 启动，Mode 1
默认从 `(-19.0, 1.0, 0.25)` 启动，仍可用 `init_x`、`init_y`、`init_z` 覆盖。

Mode 3 默认等待外部节点发布 `/initial_path`。也可以通过
`reference_path_file` 启动仓库内的演示发布器；该发布器会等待首个
`body_pose` 和规划器订阅者就绪后发布一次路径。路径至少需要两个 xyz 点，
相邻点会按三维距离 `0.5 m` 降采样，并始终保留最终点。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

## FAST-LIO 真机复现

真机入口是 `real_fastlio.launch.py`，不会启动地图生成器、运动学仿真器或
Gazebo。数据链路如下：

```text
实物 LiDAR + IMU -> FAST-LIO -> /Odometry + /cloud_registered
                              -> fastlio_pose_adapter
                              -> /scan_planner/body_pose + /scan_planner/lidar_pose
                              -> SCAN-Planner -> /planning/bspline -> /cmd_vel
```

这里直接使用 FAST-LIO 的世界坐标点云 `/cloud_registered`，所以
`grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false`，规划坐标系为
统一的 `map`。这里的 `map` 是 FAST-LIO 每次启动时建立的本次运行世界坐标系，
不表示已经具备历史地图重定位能力。`fastlio_pose_adapter` 将 FAST-LIO 的 IMU 位姿转换为
机器人几何中心位姿，并用相邻位姿估计规划器所需的世界系线速度。

### 1. 安装并启动实物驱动

以 Livox Mid-360S 为例，先安装并编译 `livox_ros_driver2`。构建本工作空间之前必须
source 驱动工作空间，否则 `fast_lio` 找不到 `livox_ros_driver2`：

```bash
source /opt/ros/humble/setup.bash
source /home/w/ws_livox/install/setup.bash
cd /home/w/scanplanner_wab
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

启动实物雷达驱动，并确认 `/livox/lidar`、`/livox/imu` 持续输出：

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### 2. 当前实机 TF 与固定配置

当前实机 `tf_tree.pdf` 表明机器狗运动根坐标系为 `base`，四条腿均位于
`base` 下方。因此真机入口已经固定使用下面的职责划分，不使用仓库中的仿真
`base -> trunk` 模型：

```text
map
├── body                       FAST-LIO：Mid-360S 内部 IMU 位姿
├── base                       fastlio_pose_adapter：机器狗机身位姿
│   ├── livox_frame            adapter 根据外参自动发布的固定安装 TF
│   ├── FL_hip -> ...
│   ├── FR_hip -> ...
│   ├── RL_hip -> ...
│   └── RR_hip -> ...          实机状态发布器
└── sliding_map                SCAN-Planner：局部滑动窗口
```

默认值固定在 `real_fastlio.launch.py`：

```text
world_frame=map
fastlio_imu_frame=body
body_frame=base
lidar_frame=livox_frame
publish_robot_state=false
publish_base_tf=true
publish_lidar_tf=true
```

`publish_robot_state=false` 保留实机已有的 `base -> 四条腿`，不会再启动仿真 Go2
模型。使用 FAST-LIO 定位时，必须关闭 Cartographer、AMCL、RF2O 或旧适配节点对
`map -> base` 的发布，确保该 TF 只有 `fastlio_pose_adapter` 一个发布者。

### 3. 标定参数（真机必须修改）

复制并修改
`FAST_LIO-ROS2/config/mid360_scanplanner.yaml`。其中：

- `mapping.extrinsic_T/R` 是 LiDAR 在 IMU 坐标系下的位置和姿态；
- `common.time_offset_lidar_to_imu` 是标定后的雷达到 IMU 时间偏移；
- 当前保持 `mapping.extrinsic_est_en: true`，与已经在同一台 Mid-360S 上验证稳定的
  `/home/w/FAPP/src/FAST_LIO-ROS2` 一致；获得并验证最终 LiDAR--IMU 标定后，才考虑
  将标定结果同时写入 FAST-LIO 和 adapter 并关闭在线估计；
- 只有硬件时间同步确实不可用时才考虑 `common.time_sync_en: true`。

`body_in_imu_{x,y,z}` 表示 `base` 在 Mid-360S IMU 坐标系中的位置，
`body_rpy_in_imu_{x,y,z}` 表示对应的 roll/pitch/yaw（弧度），即
`T_imu_base`。这六个默认值现在仍为零，因为现有 `tf_tree.pdf` 不包含 Mid-360S
安装位置，不能从 TF 拓扑伪造实机标定值。应把实测值直接写入
`real_fastlio.launch.py` 中这六个 `DeclareLaunchArgument` 的 `default_value`，之后正常
启动时不再从命令行传外参。

`lidar_in_imu_*` 是 `T_imu_livox`，必须与 FAST-LIO 配置中的
`mapping.extrinsic_T/R` 完全一致。adapter 会自动计算
`T_base_livox = inverse(T_imu_base) * T_imu_livox`，并发布固定的
`base -> livox_frame`，不需要再启动一个 `static_transform_publisher`。如果以后真实
URDF 自己发布 `base -> livox_frame`，应把 `publish_lidar_tf` 的默认值改为 `false`，
不能让两个节点同时拥有同一个 child。

错误的机身外参会让碰撞体与实物错位。在完成外参测量和架空检查以前，必须保持
`enable_control=false`。

### 4. 先做不运动的传感器验收

推荐把 FAST-LIO 单独保持运行，这样重启规划器不会重置定位原点：

```bash
# 终端 2：FAST-LIO
source /opt/ros/humble/setup.bash
source /home/w/ws_livox/install/setup.bash
source /home/w/scanplanner_wab/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360_scanplanner.yaml world_frame:=map rviz:=false

# 终端 3：只启动真机规划和输入检查，不发布运动控制
source /opt/ros/humble/setup.bash
source /home/w/ws_livox/install/setup.bash
source /home/w/scanplanner_wab/install/setup.bash
ros2 launch scan_planner real_fastlio.launch.py \
  start_fastlio:=false enable_control:=false
```

终端应周期输出类似以下状态，只有 `ready=true` 才通过输入验收：

```text
FAST-LIO input check: ready=true odom=10Hz/... cloud=10Hz/... frames=(map, map)
```

同时检查：

```bash
ros2 topic echo /scan_planner/body_pose --once
ros2 topic echo /scan_planner/lidar_pose --once
ros2 topic hz /cloud_registered
ros2 topic echo /scan_planner/fastlio_inputs_ready --once
ros2 run tf2_ros tf2_echo map base
ros2 run tf2_ros tf2_echo base livox_frame
```

在 RViz 中确认机器人位姿、原始注册点云、`/grid_map/occupancy_inflate` 完全重合；
抬起或手推机器人时，坐标轴方向、机身高度和障碍物相对位置必须正确。

### 5. 接通实物底盘控制

确认急停可用、机器人架空测试正常后，停止终端 3 并显式开启控制：

```bash
ros2 launch scan_planner real_fastlio.launch.py \
  start_fastlio:=false enable_control:=true \
  cmd_vel_topic:=/your_robot/cmd_vel
```

`cmd_vel_topic` 的消息类型必须是 `geometry_msgs/msg/Twist`；如果实物机器人 SDK 使用
其他指令类型，需要在该话题后接对应的底盘桥接节点。使用 RViz 的 `2D Goal Pose`
发送目标。控制器对定位超时和 FAST-LIO 位姿/点云健康状态做联锁：任一输入失联会
持续发布零速度，不会继续执行旧轨迹。

也可以一条命令同时启动 FAST-LIO 和规划器：

```bash
ros2 launch scan_planner real_fastlio.launch.py \
  start_livox_driver:=true start_fastlio:=true \
  start_scanplanner:=true enable_control:=false
```

`start_livox_driver:=true` 会把 Mid-360S 专用驱动纳入同一个终端和进程组；保持默认的
`false` 时，仍可连接已经单独启动的驱动。Velodyne/Ouster 可通过
`fastlio_config_file` 或 `fastlio_config_path` 选择自己的配置，注册点云和里程计话题
也可用 `registered_cloud_topic`、`fastlio_odom_topic` 覆盖。

### 6. 使用 planner App 启动与查看终端

机器人电脑只保留下面一个常驻入口：

```bash
ros2 launch scan_planner planner_app.launch.py
```

随后可在 `planner` App 中选择“Mid-360S 雷达终端”“FAST-LIO 定位终端”，以及
SCAN-Planner 模式 1/2/3 各自的“安全预览”或“实机控制”。App 会显示对应 launch 的实时输出；
停止当前流程不会关闭 App 桥接。实机控制模式会发布 `/cmd_vel`，必须先完成安全预览、
架空测试和急停检查。

模式 2 必须给常驻入口提供实际航点参数文件，例如
`keypoints_file:=/absolute/path/to/keypoints.yaml`；未配置时 App 会禁用模式 2，绝不会
自动使用仓库的示例坐标。模式 3 可用 `reference_path_file` 启动内置参考路径发布器，
也可以不提供文件，等待外部节点发布 `/initial_path`。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

参考路径演示发布器也可以独立运行：

```bash
ros2 run scan_planner reference_path_publisher.py --ros-args \
  --params-file src/planner/plan_manage/config/reference_path.map.yaml \
  -r body_pose:=/quad_0/body_pose -r initial_path:=/initial_path
```

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
