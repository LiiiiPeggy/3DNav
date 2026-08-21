# 3dnav 工作空间

本目录包含两个**相互独立**的 ROS 2 colcon 工作空间，均基于
[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)（面向路线引导四足长程导航的
空间碰撞感知局部规划器，ROS 2 Humble / C++17 移植版）：

| 目录 | 定位 | 说明 |
| --- | --- | --- |
| `scan_planner/` | **仿真为主** | 完整的原版移植工作空间（含 `build/`、`install/`），运行确定性仿真器 / mockamap / Gazebo Go2 模型 |
| `pct_scan_planner/` | **实机验证准备** | 在 scan_planner 基础上接入 FAST-LIO 真机接口；目录名 `pct_` 表示计划后续接入 PCT 规划器（**尚未接入**） |

两个工作空间各自维护自己的 `src/`，构建、source 时不要混用。

---

## 1. `scan_planner/` — 仿真工作空间

- 源码：`scan_planner/src/SCAN-Planner`，git remote `wuyi2121/SCAN-Planner`，分支 `ros2-community`
- 已构建产物在 `scan_planner/build`、`scan_planner/install`（2026-08-21 构建）
- 配套 `scan_planner/log/` 保存构建日志

### 包含的 ROS 2 包

规划器侧（`src/planner/`）：

- `scan_planner` — 主包：`scan_planner_node`、`scan_replan_fsm`（状态机）、
  `planner_manager`、`open_loop_controller`、`closed_loop_controller`、
  `go2_kinematic_sim`、`go2_gait_publisher`，以及 `launch/`、`config/`、`scripts/`
- `scan_planner_msgs` — 自定义消息 `Bspline`、`DataDisp`
- `plan_env` — 局部滑动窗口占据栅格地图（`grid_map.h`、`raycast.h`）
- `path_searching` — 前端路径搜索（`dyn_a_star.h`）
- `bspline_opt` — 均匀 B 样条轨迹优化（`bspline_optimizer.h`、`uniform_bspline.h`）
- `traj_utils` — 可视化与轨迹工具（`planning_visualization.h`、`polynomial_traj.h`）

仿真器侧（`src/simulator/`）：

- `local_sensing_node` — 传感器仿真（lidar / depth）
- `map_generator`、`mockamap` — 地图生成
- `go2_description` — 宇树 Go2 URDF / Gazebo Fortress 仿真
- `odom_visualization`、`pose_utils`、`waypoint_generator` — 可视化与里程计工具

### 构建

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端；需要 OpenGL 后端时加 `-DUSE_GPU=ON`
（依赖系统安装的 GLFW / GLEW / OpenGL 包，不再自带 x86_64 GLFW 动态库）。

### 快速启动（仿真）

终端 1 启动 RViz：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```

终端 2 启动规划器 + 仿真器（Mode 1，闭环二维演示）：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```

Mode 3 跨层演示（PCD 地图 + 参考路径）：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/绝对路径/map.pcd \
  reference_path_file:=/绝对路径/reference_path.map.yaml
```

### 导航模式与控制器

- `navi_mode:=1` — RViz 2D Goal 选目标点
- `navi_mode:=2` — 按 `fsm.waypoints` 路径点序列（`keypoints_file:=` 指向参数 YAML）
- `navi_mode:=3` — 订阅 `initial_path` 获取全局参考路径，局部避障（可配 `reference_path_file` 内置发布器）

控制器分 `open_loop`（按三维 B 样条直接发布里程计，适合多楼层仿真）与
`closed_loop`（平面 `cmd_vel` 跟踪 x/y/yaw，适合二维仿真或真机底盘接口）。
其它常用参数：`use_pcd_map` / `pcd_map_file`、`init_x/y/z`、`map_size_x/y/z`。

### 配置位置

- 规划器：`src/planner/plan_manage/config/planner.yaml`（参数以点号分隔，如 `grid_map.resolution`）
- 控制器：`src/planner/plan_manage/config/controllers.yaml`
- 仿真器：`src/planner/plan_manage/config/simulator.yaml`

---

## 2. `pct_scan_planner/` — 实机验证工作空间

- 源码：`pct_scan_planner/src/SCAN-Planner-Ros2`，git remote
  `xiaoqi371317/SCAN-Planner-Ros2`，分支 `main`
- 在 scan_planner 基础上新增了 FAST-LIO 真机接口；**目录名 `pct_` 表示后续接入 PCT
  规划器的计划，目前 PCT 尚未接入**，当前实际完成的是 FAST-LIO 真机链路
- ⚠️ 当前仓库有**未提交的本地改动**与新增文件（`real_fastlio.launch.py`、
  `fastlio_pose_adapter.cpp`、`fastlio_input_monitor.py`、`planner_app.launch.py` 等），
  注意区分已提交与本地改动
- 本工作空间在此目录下**尚未构建**（无 `build/`、`install/`），构建方法与 `scan_planner/` 相同

### 新增的真机能力

- `launch/real_fastlio.launch.py` — 真机唯一入口，不启动地图生成器 / 运动学仿真器 / Gazebo
- `src/fastlio_pose_adapter.cpp` — 将 FAST-LIO 的 IMU 里程计转换为机器人几何中心位姿，
  估计线速度，发布 `body_pose` / `lidar_pose` 及 TF
- `scripts/fastlio_input_monitor.py` — 健康检查 FAST-LIO 输入，发布 `/scan_planner/fastlio_inputs_ready`
- `closed_loop_controller.cpp` 新增安全联锁：`odom_timeout`、`require_inputs_ready`，
  任一输入失联时持续发布零速度
- Mode 3 参考路径：`reference_path_utils.h`、`scripts/reference_path_publisher.py`、
  `config/reference_path.map.yaml`
- `launch/planner_app.launch.py` — 基于 `nav2_web` 的常驻入口 App，选择雷达 / FAST-LIO /
  规划模式启动

### 真机数据链路

```text
实物 LiDAR + IMU -> FAST-LIO -> /Odometry + /cloud_registered
                              -> fastlio_pose_adapter
                              -> /scan_planner/body_pose + /scan_planner/lidar_pose
                              -> SCAN-Planner -> /planning/bspline -> /cmd_vel
```

直接使用 FAST-LIO 的世界坐标点云 `/cloud_registered`，因此
`grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false`，规划坐标系统一为 `map`。
注意：此 `map` 是 FAST-LIO 本次运行建立的世界系，**不具备历史地图重定位能力**。

### 真机 TF 结构

```text
map
├── body                       FAST-LIO：Mid-360S 内部 IMU 位姿
├── base                       fastlio_pose_adapter：机器狗机身位姿
│   ├── livox_frame            adapter 根据外参自动发布的固定 TF
│   ├── FL/FR/RL/RR_hip -> ... 实机状态发布器
└── sliding_map                SCAN-Planner 局部滑动窗口
```

`real_fastlio.launch.py` 固定默认值：`world_frame=map`、`fastlio_imu_frame=body`、
`body_frame=base`、`lidar_frame=livox_frame`、`publish_robot_state=false`、
`publish_base_tf=true`、`publish_lidar_tf=true`。使用 FAST-LIO 定位时，必须关闭
Cartographer / AMCL / RF2O 等对 `map -> base` 的发布，确保该 TF 只有 adapter 一个发布者。

### 外参标定（真机必须修改）

- `body_in_imu_{x,y,z}`、`body_rpy_in_imu_{x,y,z}`：`base` 在 Mid-360S IMU 系中的
  位姿（即 `T_imu_base`），当前默认全 0，需把实测值写入 launch 的
  `DeclareLaunchArgument` 默认值
- `lidar_in_imu_*`：`T_imu_livox`，必须与 FAST-LIO 配置 `mapping.extrinsic_T/R` 完全一致；
  adapter 自动计算 `T_base_livox = inverse(T_imu_base) * T_imu_livox`
- 在完成外参测量与架空检查前，必须保持 `enable_control=false`

### 真机启动步骤（流程）

1. 构建：先 source 驱动工作空间（Livox 为例：`source /home/w/ws_livox/install/setup.bash`），
   再构建本工作空间
2. 启动实物驱动（`livox_ros_driver2 msg_MID360s_launch.py`），确认 `/livox/lidar`、`/livox/imu`
   有频率
3. **传感器验收**（不运动）：保持 FAST-LIO 单独运行，然后

   ```bash
   ros2 launch scan_planner real_fastlio.launch.py \
     start_fastlio:=false enable_control:=false
   ```

   直到日志出现 `FAST-LIO input check: ready=true`，并检查
   `/scan_planner/body_pose`、`/scan_planner/lidar_pose`、`/cloud_registered`、
   `/scan_planner/fastlio_inputs_ready`、`tf2_echo map base` 等
4. **接通底盘控制**：确认急停可用、架空测试正常后

   ```bash
   ros2 launch scan_planner real_fastlio.launch.py \
     start_fastlio:=false enable_control:=true \
     cmd_vel_topic:=/your_robot/cmd_vel
   ```

   `cmd_vel_topic` 消息类型必须是 `geometry_msgs/msg/Twist`；若机器人 SDK 用别的指令
   类型，需在话题后接底盘桥接节点。控制器对定位超时 / FAST-LIO 输入做联锁，失联即零速。
5. **常驻入口**（机器人电脑只保留这一个）：

   ```bash
   ros2 launch scan_planner planner_app.launch.py
   ```

   在 App 中选择雷达 / FAST-LIO 终端，以及模式 1/2/3 的“安全预览”或“实机控制”。
   模式 2 必须给常驻入口提供 `keypoints_file:=`；模式 3 可用 `reference_path_file:=`
   启动内置参考路径发布器，也可等待外部节点发布 `/initial_path`。

也可一条命令同时启动驱动 + FAST-LIO + 规划器：

```bash
ros2 launch scan_planner real_fastlio.launch.py \
  start_livox_driver:=true start_fastlio:=true \
  start_scanplanner:=true enable_control:=false
```

真机依赖外部包：`livox_ros_driver2`、`fast_lio`（`FAST_LIO-ROS2`）、`nav2_web`。
`run.launch.py` 中所有真机话题（`body_pose_topic` / `sensor_pose_topic` / `cloud_topic` /
`cmd_vel_topic` / `world_frame` 等）均可覆盖，以适配不同的驱动栈。

---

## 3. 测试

`pct_scan_planner` 的 `plan_manage` 注册了 gtest / pytest / launch 测试：

```bash
# 在对应工作空间根目录（src/ 所在目录）执行
colcon test --packages-select scan_planner
```

覆盖：`test_reference_path_utils`（C++ 参考路径预处理）、
`test_reference_path_publisher_unit`（pytest）、`test_planner_startup`、
`test_reference_path_publisher_launch`、`test_waypoint_parameters`（launch 测试）。

## 4. 工具

- 关键点记录器：`ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml`
  （生成 `fsm.waypoints` 参数 YAML，供 `navi_mode:=2` 使用；`pct_scan_planner` 版可
  用 `--odom` 指定里程计话题）
- 参考路径发布器：`ros2 run scan_planner reference_path_publisher.py --ros-args
  --params-file .../reference_path.map.yaml`

## 5. 致谢与许可

两个仓库均衍生自 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)
（作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang、Tong Qin），Apache License 2.0。
实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 与 Leg-KILO；真实机器人定位基于
Elevator-LIO / FAST-LIO2。分发或派生时请保留各仓库 `LICENSE` 与 `NOTICE`。
