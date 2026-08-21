# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

`/home/yu/3dnav` 下是**三个相互独立的工作空间**，构建与 `source` 绝不能混用：

| 目录 | 系统 | 定位 |
| --- | --- | --- |
| `scan_planner/` | ROS 2 Humble colcon | 仿真为主，SCAN-Planner 完整移植（有 `build/`、`install/`、`log/`） |
| `pct_scan_planner/` | ROS 2 Humble colcon | 实机验证准备，接入 FAST-LIO 真机接口（尚未构建） |
| `leg_3dnav/legbot_3D_Nav/` | **ROS 1 Noetic catkin** | 完整 A1 三维导航项目，**PCT 全局规划器所在**（第三个、已整体纳入） |

两个 ROS 2 工作空间都基于 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)
（面向路线引导四足长程导航的空间碰撞感知局部规划器，ROS 2 Humble / C++17）。三个目录里
的上游源码都已**整体纳入（vendored）本仓库**：嵌套的 `.git` 已删除，源码树就是普通目录，
可直接编辑并提交，**无需 `git submodule update`**：

- **`scan_planner/`** — 源码在 `scan_planner/src/SCAN-Planner`（源自 `wuyi2121/SCAN-Planner`，
  分支 `ros2-community`）。
- **`pct_scan_planner/`** — 源码在 `pct_scan_planner/src/SCAN-Planner-Ros2`
  （源自 `xiaoqi371317/SCAN-Planner-Ros2`，分支 `main`）。已接入 FAST-LIO 真机接口；
  目录名 `pct_` 是接入 PCT 规划器的计划，**PCT 尚未接入此工作空间**（真正的 PCT 在
  `leg_3dnav` 里）。本地新增的 FAST-LIO 适配文件（`real_fastlio.launch.py`、
  `fastlio_pose_adapter.cpp`、`fastlio_input_monitor.py`、`planner_app.launch.py` 等）
  **不在上游、是本仓库独有的**，已提交进本仓库。

## 常用命令（ROS 2 工作空间）

### 构建（在对应工作空间根目录，即 `src/` 所在层）

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- 默认 CPU 后端；OpenGL 后端加 `-DUSE_GPU=ON`（依赖系统 GLFW/GLEW/OpenGL，仓库不再带 x86_64 GLFW 库）。
- 系统依赖：`libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev`。
- 构建 `pct_scan_planner` 的真机场景前必须**先 source 外部驱动工作空间**（如 Livox：
  `source /home/w/ws_livox/install/setup.bash`），否则 `fast_lio` 找不到 `livox_ros_driver2`。

### 运行

仿真（终端 1 RViz，终端 2 规划器+仿真器）：

```bash
ros2 launch scan_planner rviz.launch.py
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 \
  sensor_type:=lidar controller_mode:=closed_loop use_gpu:=false
```

真机（`pct_scan_planner` 独有）：

```bash
# 传感器验收（不运动）
ros2 launch scan_planner real_fastlio.launch.py start_fastlio:=false enable_control:=false
# 接通底盘控制
ros2 launch scan_planner real_fastlio.launch.py start_fastlio:=false \
  enable_control:=true cmd_vel_topic:=/your_robot/cmd_vel
# 常驻入口 App
ros2 launch scan_planner planner_app.launch.py
```

### 测试

```bash
colcon test --packages-select scan_planner
```

两个 ROS 2 工作空间的 `plan_manage` 都注册了 gtest（`test_reference_path_utils`）、pytest
（`test_reference_path_publisher_unit`）和 launch 测试（`test_planner_startup`、
`test_reference_path_publisher_launch`、`test_waypoint_parameters`）。

## 架构（ROS 2 工作空间）

规划流水线（EGO-Planner 风格）：`scan_planner_node` → `scan_replan_fsm`（状态机，处理
navi_mode 1/2/3 触发重规划）→ `planner_manager`（编排）→ 前端 `path_searching/dyn_a_star`
→ 后端 `bspline_opt`（均匀 B 样条轨迹优化）→ 发布 `planning/bspline`，由
`open_loop_controller` / `closed_loop_controller` 跟踪。

`src/planner/` 下的包（两 ROS 2 工作空间结构相同）：

- `plan_manage` — 主包：`scan_planner_node`、`scan_replan_fsm`、`planner_manager`、
  两个控制器、`go2_kinematic_sim` / `go2_gait_publisher`（仿真）、`launch/`、`config/`、`scripts/`
- `plan_env` — 局部滑动窗口占据栅格地图（`grid_map.h`、`raycast.h`）
- `path_searching` — 前端路径搜索（`dyn_a_star.h`）
- `bspline_opt` — B 样条优化（`bspline_optimizer.h`、`uniform_bspline.h`）
- `traj_utils` — 可视化与轨迹工具
- `scan_planner_msgs` — 自定义消息 `Bspline`、`DataDisp`

`src/simulator/`：`local_sensing_node`、`map_generator`、`mockamap`、`go2_description`、
`odom_visualization`、`pose_utils`、`waypoint_generator`。

参数在 `plan_manage/config/` 下三个 YAML（`planner.yaml`、`controllers.yaml`、
`simulator.yaml`），ROS 2 参数用点号分隔（如 `grid_map.resolution`、`fsm.navi_mode`）。

## 关键差异与注意点（ROS 2）

- **导航模式**：`navi_mode:=1` RViz 目标点 / `2` 预设路径点 / `3` 订阅 `initial_path`
  参考路径做局部避障。模式 2 必须给 `keypoints_file`；`reference_path_file` 只在模式 3 有效。
- **控制器**：`open_loop` 直接按三维 B 样条发布里程计（多楼层仿真）；`closed_loop` 平面
  `cmd_vel` 跟踪 x/y/yaw（二维仿真或真机）。**真机只允许 `closed_loop`**，launch 中有校验。
- **真机话题**：FAST-LIO 输入为 `/Odometry`、`/cloud_registered`，规划器用
  `grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false`，坐标系统一为 `map`
  （FAST-LIO 本次运行的世界系，**无历史地图重定位能力**）。`run.launch.py` 的所有真机
  话题（`body_pose_topic` / `sensor_pose_topic` / `cloud_topic` / `cmd_vel_topic` /
  `world_frame` 等）都可在 launch 参数里覆盖。
- **TF 结构**（真机固定，不用仿真的 `base -> trunk`）：`map -> body`（FAST-LIO IMU）、
  `map -> base`（adapter，唯一发布者，需关闭 Cartographer/AMCL/RF2O 等的同名 TF）、
  `base -> livox_frame`（adapter 按外参自动发布）。真机要 `publish_robot_state=false`。
- **外参标定**：`body_in_imu_*` / `body_rpy_in_imu_*`（`T_imu_base`）目前默认全 0，需实测后
  直接写进 `real_fastlio.launch.py` 的默认值；`lidar_in_imu_*`（`T_imu_livox`）必须与
  FAST-LIO 配置 `mapping.extrinsic_T/R` 一致。adapter 自动计算
  `T_base_livox = inverse(T_imu_base) * T_imu_livox`，无需再起 `static_transform_publisher`。
- **安全联锁**（`closed_loop_controller`）：`odom_timeout`（0.3 s）与
  `require_inputs_ready` 联锁，任一输入失联即发布零速度；`enable_control=false` 时只验收
  不控制。FAST-LIO 应单独常驻运行，重启规划器不会重置定位原点。
- **模式 3 参考路径**：`reference_path_utils.h` 的 `prepareReferenceWaypoints` 会把路径
  按 `min_distance=0.5 m` 降采样、`z` 加 `grid_map.body_height`（`planner.yaml` 中为 0.4；
  路径文件的 `z` 是地面/路线高度，规划后机身高度约为其 + body_height）；发布器
  `reference_path_publisher.py` 等待首个 `body_pose` 且订阅者就绪后才发布一次
  `/initial_path`（TRANSIENT_LOCAL）。

## `leg_3dnav` — 第三个项目（ROS 1，PCT 全局规划器所在）

`leg_3dnav/legbot_3D_Nav/` 与上述两个 ROS 2 工作空间完全独立，是 `pct_` 前缀里 "PCT"
的真正出处：

- 源自 `github.com/Robot-Nav/legbot_3D_Nav` 的 ROS 1 参考实现，已整体纳入本仓库（嵌套
  `.git` 已删除，不再是独立克隆/子模块）。
- **ROS 1 Noetic / Ubuntu 20.04 / Gazebo Classic 11** 的 catkin 工作空间
  （`src/CMakeLists.txt` symlink 到 catkin toplevel.cmake，不是 colcon）。
- ⚠️ **本机只装了 ROS 2 Humble**（`/opt/ros` 下只有 humble，无 `catkin_make`，noetic symlink
  是断的）——**当前无法在本机构建**。它是移植参考源：把 PCT 全局规划 + A1 RL 控制器的 ROS 1
  实现移植到 Humble 时以它为蓝本，不要试图在本机 `catkin build` 它。

### 结构与数据流

- `src/PCT_planner/` — **Python 全局规划器**（带 `CATKIN_IGNORE`，不走 catkin）：
  `planner/` 内是 C++/pybind11 模块（a_star、traj_opt、ele_planner、py_map_manager），
  构建需先跑 `planner/build_thirdparty.sh`（gtsam/osqp）再 `planner/build.sh`；
  `tomography/scripts/downsample_pcd.py` 把 PCD 切成多层 tomogram；产出的 `.pickle` 供规划用。
  Python 依赖见 `requirements.txt`（numpy 1.24.4 / scipy / open3d / cupy-cuda12x，Python 3.8）。
- `src/FAST_LIO`、`src/Mid360_imu_sim` — 激光里程计与 Mid-360 雷达/IMU 仿真。
- `src/planner/`（EGO 局部）与 `src/SCAN-Planner/`（SCAN 局部，catkin wrapper）包结构相同
  （bspline_opt / path_searching / plan_env / plan_manage / traj_utils）。
- `src/unitree_guide` — A1 的 RL 运动控制器（libtorch policy），含 unitree_ros_to_real。
- `src/legbot_bringup` — 编排入口，用 `roslaunch legbot_bringup <xxx>.launch` 启动
  （simulation / fastlio / controller / local_planners / pct_tomography / pct_plan /
  map_to_odom_static / visualization / navigation）。

数据流：PCD → PCT tomogram → 多层 A* → `/pct_path` → reference_path_transform →
EGO/SCAN 局部 B-spline → `scan_a1_cmd_adapter`（把三维轨迹投影成 x/y/yaw）→ `/cmd_vel` →
A1 RL policy → Gazebo。里程计两条路：Gazebo 真值 `/Odometry_gazebo`（state_from_gazebo）
或 FAST-LIO `/fast_lio/odometry_base`。

**坐标系注意**：`/pct_path` 在 `map` 系，局部规划器工作在 `odom` 系，需要
`map_to_odom_static.launch` 提供静态 TF（不是重定位）——这与 ROS 2 真机「全部统一到 `map`」
的约定不同。

详细中文文档：`docs/PROJECT_ANALYSIS_CN.md`（架构与算法）、`docs/FAST_LIO_MAPPING_CN.md`
（建图 → tomogram → 导航的完整操作流程）。
