# 3dnav 工作空间

本目录包含**四个相互独立的工作空间**，涵盖从仿真、真机到跨层全局规划的完整研发链路：
两个 ROS 2 colcon 工作空间（基于
[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)，ROS 2 Humble / C++17），
一个 PCT-SCAN-ROS2 一体化导航工程（ROS 2 Humble），以及一个 ROS 1 Noetic 参考项目。
所有上游源码均已**整体纳入（vendored）本仓库**：嵌套的 `.git` 已删除，源码树就是普通
目录，可直接编辑并提交。

| 目录 | 系统 | 定位 | 说明 |
| --- | --- | --- | --- |
| `scan_planner/` | ROS 2 Humble | **仿真为主** | 完整的原版移植工作空间（含 `build/`、`install/`），运行确定性仿真器 / mockamap / Gazebo Go2 模型 |
| `scanplanner_fastlio_ros2/` | ROS 2 Humble | **实机验证准备** | 在 scan_planner 基础上接入 FAST-LIO 真机接口 |
| `pct_scan_ros2/src/PCT-SCAN-ROS2/` | ROS 2 Humble | **PCT+SCAN 一体化导航** | PCT 跨层全局规划 + SCAN 局部 B-spline 的 ROS 2 一体化验证工程（RViz 演示，无 Gazebo） |
| `pct_scan_fastlio_ros1/legbot_3D_Nav/` | **ROS 1 Noetic** | **PCT 全局规划参考** | 完整 A1 三维导航项目（FAST-LIO + PCT + EGO/SCAN 局部 + A1 RL），ROS 1 catkin 工作空间 |

各工作空间各自维护自己的 `src/`，构建、source 时不要混用。

---

## 1. `scan_planner/` — 仿真工作空间

- 源码：`scan_planner/src/SCAN-Planner`，源自 `wuyi2121/SCAN-Planner`（分支 `ros2-community`）
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

## 2. `scanplanner_fastlio_ros2/` — 实机验证工作空间

- 源码：`scanplanner_fastlio_ros2/src/SCAN-Planner-Ros2`，源自 `xiaoqi371317/SCAN-Planner-Ros2`
  （分支 `main`）
- 在 scan_planner 基础上新增了 FAST-LIO 真机接口
- ⚠️ 本地新增的 FAST-LIO 适配文件（`real_fastlio.launch.py`、`fastlio_pose_adapter.cpp`、
  `fastlio_input_monitor.py`、`planner_app.launch.py` 等）**不在上游、是本仓库独有的**，
  已随本仓库提交
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

   在 App 中选择雷达 / FAST-LIO 终端，以及模式 1/2/3 的"安全预览"或"实机控制"。
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

## 3. `pct_scan_ros2/src/PCT-SCAN-ROS2/` — PCT-SCAN-ROS2 一体化导航工程

克隆自 [yagami-light7/PCT-SCAN-ROS2](https://github.com/yagami-light7/PCT-SCAN-ROS2)，
已整体纳入本仓库（嵌套 `.git` 已删除）。本工程将 PCT 跨层全局规划器与 SCAN 局部轨迹规划
合并在 ROS 2 Humble 一体化运行，验证 Building、Plaza、Spiral 三个场景的 RViz 演示。

### 架构与数据流

```text
PCD 点云
  ↓  （构建阶段：tomography 脚本）
PCT tomogram pickle
  ↓  PCT native 多层 A*（跨 tomogram layer）
/pct_path (nav_msgs/Path, reliable + transient_local)
  ↓  pct_scan_bridge（RDP 简化 + 分段线性参考参数化）
SCAN Mode 3 (initial_path_topic:=/pct_path)
  ↓  局部 B-spline
planning/bspline
  ↓  open_loop 三维执行器 / Go2 RViz 动画
```

PCT 负责回答"从哪个楼层、经过哪段楼梯/坡道、走到哪里"；SCAN 在全局路线的局部窗口内
持续生成可跟踪的 B-spline。`pct_scan_bridge` 包从 pickle 读取动态 tomogram 层数，
修正第三方 wrapper 的固定层数 bug，并通过 `/pct_path` 将 PCT 全局结果接入 SCAN Mode 3。

### ROS 2 包结构

- `src/pct/pct_planner` — PCT 全局规划器（ament_cmake + pybind11 native：`a_star`、
  `ele_planner`、`py_map_manager`、`traj_opt`）
- `src/pct/tomography` — tomogram 生成（检查 CUDA，GPU 不可用时改用 NumPy/SciPy）
- `src/integration/pct_scan_bridge` — Python 桥接节点：复用上游 `PCTPlanner` 节点，
  动态层数修正、交互 marker（Start/Goal，Z 轴选楼层），发布 `/pct_path`
- `src/planner/*` — SCAN 局部规划器（plan_manage / bspline_opt / plan_env / path_searching / traj_utils / scan_planner_msgs）
- `src/simulator/*` — 传感器仿真、地图生成、Go2 URDF 与 RViz 动画

### 环境与构建

```bash
# 1. 安装 ROS 2 和系统依赖
sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build git curl \
  python3-colcon-common-extensions python3-rosdep python3-dev python3-pip python3-venv \
  libboost-all-dev libeigen3-dev \
  libarmadillo-dev libglew-dev libglfw3-dev \
  libgl1-mesa-dev libglu1-mesa-dev
rosdep install --from-paths src --ignore-src -r -y

# 2. 编译 PCT 原生依赖（GTSAM 4.2 + OSQP 1.0，装到 .deps/install）
./scripts/setup_pct_dependencies.sh

# 3. 构建工作空间
./scripts/build_workspace.sh
source install/setup.bash
```

### 验收

```bash
./scripts/test_pct_building_astar.sh --force
./scripts/test_pct_plaza_astar.sh --force
# 三个场景都打印 [astar] PASS 才说明 PCT native 库已连通
```

### 快速启动

```bash
# Building（PCD + pickle 随仓库提供）
./scripts/launch_pct_scan_building.sh
# 自动规划默认路线（无需 RViz 交互）
./scripts/launch_pct_scan_building.sh plan_on_startup:=true

# Plaza（PCD + pickle 随仓库提供）
./scripts/launch_pct_scan_plaza.sh
./scripts/launch_pct_scan_plaza.sh plan_on_startup:=true

# Spiral（需先下载资产并生成 tomogram）
./scripts/fetch_spiral_asset.sh
./scripts/setup_tomography_env.sh
./scripts/generate_spiral_tomogram.sh
./scripts/test_pct_spiral_astar.sh --force
./scripts/launch_pct_scan_spiral.sh
```

RViz 打开后选择工具栏中的 `Interact`，拖动绿色 Start 和红色 Goal（跨层时沿蓝色 Z 轴
把 Goal 放到目标楼层），右键任意 marker 选择 `Plan route`。

终端验收（无 RViz）：

```bash
./scripts/launch_pct_scan_building.sh \
  use_rviz:=false plan_on_startup:=true \
  publish_tomogram:=false enable_interactive_markers:=false
```

终端应先看到 PCT 发布 `/pct_path`，然后 SCAN 打印 `Received trajectory N`。

### 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `controller_mode` | `open_loop` | 跨层必须用 open_loop（三维执行器直接采样 B-spline） |
| `reference_path_min_distance` | `0.15` | 三维降采样间距 |
| `reference_path_simplify_tolerance` | `0.1` | RDP 简化容差 |
| `enable_local_sensing` | `false` | Building 场景建议关闭（楼梯稠密点云会触发碰撞膨胀） |
| `collision_radius` / `collision_offset` | `0.12` | PCT 标记窄通道可通行，SCAN 用紧凑足迹 |

### 场景资产

- **Building / Plaza**：PCD 和 pickle 随仓库提供，在 `assets/<scene>/` 下
- **Spiral**：来自 [3D2M-planner](https://github.com/ZJU-FAST-Lab/3D2M-planner)，
  源仓库无明确资产许可证，Git 不分发其资产，需通过脚本本地下载生成

### 与现有链路的关系

`pct_scan_bridge` 复用 `scan_planner` 包的 `run.launch.py`（Mode 3，
`initial_path_topic:=/pct_path`），本工程即 `scanplanner_fastlio_ros2` 计划接入的
PCT 算法的 ROS 2 一体化参考实现。

### 许可

PCT / tomography / bridge 使用 **GPL-2.0-or-later**；SCAN 包继续使用 **Apache-2.0**；
其他 ROS 包保留各自 `package.xml` 声明的许可。完整映射见
[`LICENSE`](LICENSE)、[`NOTICE`](NOTICE)、[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

---

## 4. `pct_scan_fastlio_ros1/legbot_3D_Nav/` — ROS 1 参考项目

`pct_scan_fastlio_ros1/legbot_3D_Nav/` 与上述三个 ROS 2 工作空间完全独立，是 PCT 全局
规划的 ROS 1 Noetic 参考源，也是移植到 ROS 2 Humble 时的蓝本。

- 源自 `github.com/Robot-Nav/legbot_3D_Nav`（**ROS 1 Noetic / Ubuntu 20.04 / Gazebo
  Classic 11** 的 catkin 工作空间，非 colcon），已整体纳入本仓库
- ⚠️ **本机只装了 ROS 2 Humble**（无 `catkin_make`、noetic 依赖缺失），**当前无法在
  本机构建**，仅作移植参考

### 结构与数据流

- `src/PCT_planner/` — **Python 全局规划器**（带 `CATKIN_IGNORE`，不走 catkin）：
  `planner/` 内是 C++/pybind11 模块（a_star、traj_opt、ele_planner、py_map_manager），
  构建需先跑 `planner/build_thirdparty.sh`（gtsam/osqp）再 `planner/build.sh`；
  `tomography/scripts/downsample_pcd.py` 把 PCD 切成多层 tomogram
- `src/FAST_LIO`、`src/Mid360_imu_sim` — 激光里程计与 Mid-360 雷达/IMU 仿真
- `src/planner/`（EGO 局部）与 `src/SCAN-Planner/`（SCAN 局部，catkin wrapper）
- `src/unitree_guide` — A1 的 RL 运动控制器（libtorch），含 unitree_ros_to_real
- `src/legbot_bringup` — 编排入口，`roslaunch legbot_bringup <xxx>.launch`

数据流：PCD → PCT tomogram → 多层 A* → `/pct_path` → reference_path_transform →
EGO/SCAN 局部 B-spline → `scan_a1_cmd_adapter`（三维轨迹投影为 x/y/yaw）→ `/cmd_vel` →
A1 RL policy → Gazebo。`/pct_path` 在 `map` 系、局部规划在 `odom` 系，需
`map_to_odom_static.launch` 提供静态 TF（与 ROS 2 真机「全 map 系」不同）。

详细中文文档：`docs/PROJECT_ANALYSIS_CN.md`、`docs/FAST_LIO_MAPPING_CN.md`。

---

## 5. 测试

`scan_planner` 和 `scanplanner_fastlio_ros2` 的 `plan_manage` 注册了 gtest / pytest /
launch 测试：

```bash
# 在对应工作空间根目录（src/ 所在目录）执行
colcon test --packages-select scan_planner
```

覆盖：`test_reference_path_utils`（C++ 参考路径预处理）、
`test_reference_path_publisher_unit`（pytest）、`test_planner_startup`、
`test_reference_path_publisher_launch`、`test_waypoint_parameters`（launch 测试）。

`pct_scan_ros2` 的验收测试见各场景的 `test_pct_*_astar.sh` 脚本。

## 6. 工具

- 关键点记录器：`ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml`
  （生成 `fsm.waypoints` 参数 YAML，供 `navi_mode:=2` 使用；`scanplanner_fastlio_ros2`
  版可用 `--odom` 指定里程计话题）
- 参考路径发布器：`ros2 run scan_planner reference_path_publisher.py --ros-args
  --params-file .../reference_path.map.yaml`

## 7. 致谢与许可

两个 ROS 2 SCAN-Planner 工作空间均衍生自 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)
（作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang、Tong Qin），Apache License 2.0；
实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 与 Leg-KILO，真实机器人定位基于
Elevator-LIO / FAST-LIO2。

`pct_scan_ros2` 克隆自 [yagami-light7/PCT-SCAN-ROS2](https://github.com/yagami-light7/PCT-SCAN-ROS2)，
PCT planner 来自 [byangw/PCT_planner](https://github.com/byangw/PCT_planner)（GPL-2.0-or-later），
PCT ROS 2 移植来源为 [2473o/PCT_planner](https://github.com/2473o/PCT_planner/tree/feat/ros2pkg)。

`pct_scan_fastlio_ros1` 源自 [Robot-Nav/legbot_3D_Nav](https://github.com/Robot-Nav/legbot_3D_Nav)
（Apache License 2.0），内部含 FAST-LIO、PCT、unitree_guide 等上游组件。

分发或派生时请保留各仓库 `LICENSE` 与 `NOTICE`。
