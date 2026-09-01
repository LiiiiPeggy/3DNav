# PCT-SCAN-ROS2 × Odin SLAM 接口对齐设计（无图导航）

日期：2026-09-01
分支：`realpctscan`
状态：已批准（2026-09-01，方案 A）

## 1. 背景与目标

`pct_scan_ros2/src/PCT-SCAN-ROS2` 现有的真机分支（`run.launch.py` 中 `is_real_world=true`）
写死的是从 FAST-LIO 工作空间移植来的 `/LIO/*` 话题，与实际部署的 **Odin SLAM**
话题不一致。本设计将规划器的真实接口对齐到 Odin，为后续机器狗实机部署打基础。

目标：
- 使用 Odin 实时点云 `/registered_scan`（PointCloud2 XYZI，odom 帧）与定位
  `/state_estimation`（nav_msgs/Odometry，odom 帧）驱动 SCAN 局部规划。
- **无图导航**：不重定位，每次运行 Odin 新建 odom 帧，planner 跟随其作为世界帧。
- 分两阶段：**A** 先打通实时局部避障（SCAN Mode 1，RViz 目标点）；**B** 后加 PCT
  全局参考路线（复用预生成的 tomogram）。
- 控制链：`closed_loop_controller → /cmd_vel`（geometry_msgs/Twist，平面 x/y/yaw）。

## 2. 范围

- **阶段 A（本次实现）**：接口对齐 + 实时局部避障。
- **阶段 B（后置，仅设计大纲）**：存好的 tomogram 提供 PCT 全局参考。
- 明确排除：重定位模式（Odin `3run_relocalization.sh`）、PCT 全局规划器（阶段 B 才接）。

## 3. 现有接口梳理

### 3.1 Odin SLAM 侧（`SLAM/`，独立 colcon 工作空间）

- `SLAM/2run_slam.sh` → `ros2 launch odin_ros_driver odin1_ros2.launch.py`（映射模式，
  `control_command_slam.yaml`，enable_rviz=true）。
- `host_sdk_sample` 节点：`('odin1/odometry', '/state_estimation')` 重映射，发布
  **nav_msgs/Odometry**，odom 帧（odometry 模式 `custom_map_mode=0` 下 odom==map）。
- `registered_scan_adapter_node`：订阅 `/odin1/cloud_slam` + `/state_estimation`
  （做车体相对距离滤波），输出 **/registered_scan**（sensor_msgs/PointCloud2，XYZI），
  保留输入帧（odom）。
- 辅助脚本：`4savemap.sh`（导出 .bin）、`5bin2ply.sh`（.ply）、`6ply2pcd.sh`（.pcd）
  —— 阶段 B 生成 tomogram 用。

### 3.2 PCT-SCAN-ROS2 规划器侧

- `run.launch.py` 真实分支现状：`body_pose=/LIO/odom_vehicle`、`sensor_pose=/LIO/odom_imu`、
  `cloud=/LIO/clouds_lidar`、`cloud_is_world=false`、`need_extrinsic=true` —— **与 Odin 不符**。
- 消费接口（scan_planner_node → SCANReplanFSM / GridMap）：
  - `body_pose`：nav_msgs/Odometry，FSM 取起点与偏航（`odometryCallback`）。
  - `sensor_pose`：nav_msgs/Odometry，GridMap 决定滑动窗口中心（`ray_pos`）。
  - `cloud`：PointCloud2，GridMap 建图；`cloud_is_world=true` 时点按世界帧直接用。
  - `move_base_simple/goal`（Mode 1 目标）、`initial_path`（Mode 3 参考路径）。
- 控制器：`closed_loop_controller` 订阅 `planning/bspline` + `body_pose`，发布 `cmd_vel`，
  不依赖 TF。

## 4. 阶段 A 设计

### 4.1 数据流与坐标系

```text
Odin SLAM (2run_slam.sh, 映射模式)
  ├─ /state_estimation (Odometry, odom帧) ──→ planner body_pose / sensor_pose
  └─ /registered_scan  (PointCloud2, odom帧) ──→ planner cloud

planner (is_real_world=true, closed_loop, navi_mode=1, sensor_type=lidar)
  /registered_scan → grid_map 局部占据地图（cloud_is_world=true 直接用）
  /state_estimation → FSM 起点/偏航 + 滑动窗口中心
  RViz 2D Nav Goal → 局部 A* 避障 → planning/bspline → closed_loop → /cmd_vel → 机器狗
```

**坐标系约定**：odom 帧 = planner 的世界帧。Odin odometry 模式下 odom==map，且
`/registered_scan` 与 `/state_estimation` 同帧，planner 无需 TF，直接以 odom 为世界帧。
与仿真 `/quad_0/cloud`（世界帧）行为一致。

**接口映射**：

| planner 接口 | 参数 | 值 |
| --- | --- | --- |
| body_pose | `body_pose_topic` | `/state_estimation` |
| sensor_pose | `sensor_pose_topic` | `/state_estimation` |
| cloud | `cloud_topic` | `/registered_scan` |
| grid_map.cloud_is_world | — | `true` |
| grid_map.need_extrinsic | — | `false` |
| cmd_vel | `cmd_vel_topic` | `/cmd_vel`（可覆盖） |

### 4.2 `run.launch.py` 改动

新增 launch 参数（默认随 `is_real_world` 分支）：

- `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`cmd_vel_topic`、`world_frame`
  （`world_frame` 默认 `odom`；阶段 A 中 planner 不使用 TF，该参数被接受但保留供
  阶段 B 帧对齐使用）、`publish_robot_state`。

`_setup` 中 `is_real_world=true` 分支：

- `body_pose = /state_estimation`、`sensor_pose = /state_estimation`、
  `cloud = /registered_scan`（均可被对应 launch 参数覆盖）。
- `cloud_is_world = true`、`need_extrinsic = false`。
- 真实分支**不启动** `robot_state_publisher`（真机狗有自己的 TF/URDF，避免同名 TF 冲突；
  与 `scanplanner_fastlio_ros2` 真机约定一致）。
- depth intrinsics 仅在真实分支为 depth 传感器时需要；真实只用 lidar，intrinsics 置空。
- **阶段 A 真实分支仅支持 `sensor_type=lidar`**；`is_real_world=true` 搭配
  `sensor_type=depth` 不在本次支持范围。

### 4.3 新增 `scripts/launch_pct_scan_real.sh`

- 复用 `scripts/_pct_scan_env.sh`（加载 /opt/ros/humble、工作空间 install、`.deps`）。
- 拉起：`run.launch.py is_real_world:=true navi_mode:=1 controller_mode:=closed_loop
  sensor_type:=lidar` + 可选 RViz。
- 参数透传：`use_rviz`、`cmd_vel_topic`、`collision_radius`/`collision_offset`
  （机器狗紧凑足迹，参考 PCT-SCAN 约定 0.12）。

### 4.4 参数与安全

- 起点：Odin 启动即 odom 原点；狗原地启动，FSM 从 `body_pose` 取起点与偏航。
- 速度/加速度沿用 `planner.yaml` / `controllers.yaml` 默认；`max_vel`/`max_acc` 可覆盖。
- 无重定位：每次 Odin 新建 odom 帧，planner 全程跟随 —— 这是"无图"的核心约定。

## 5. 阶段 B 大纲（后置，不在本次实现）

1. **建图导出**：跑一遍 Odin 映射模式，`4savemap.sh` → `5bin2ply.sh` → `6ply2pcd.sh`
   得到环境 PCD。
2. **生成 tomogram**：用 `scripts/generate_tomogram.sh` / `generate_pct_tomogram.py`
   从 PCD 生成 PCT tomogram（pickle），存入 `assets/<scene>/`。
3. **全局接线**：启动 pct_planner + pct_scan_bridge → `/pct_path`；
   planner `navi_mode=3`（`initial_path_topic:=/pct_path`）→ SCAN 沿全局参考做局部避障。
4. **⚠️ 帧对齐风险（核心待定项）**：存好的 tomogram 位于"建图那次会话"的 odom 帧；
   每次新运行 Odin 是新的 odom 原点。无重定位下，仅当狗在**同一位置同朝向**启动时
   两帧一致。需在阶段 B 定方案：固定启动位姿，或为 tomogram 提供粗对齐偏移参数。

## 6. 验证与验收（真机实时）

1. T1 运行 `SLAM/2run_slam.sh`；`ros2 topic hz /registered_scan`、`ros2 topic echo
   /state_estimation` 确认话题有数据且位姿在动。
2. **前提校验**：确认 `/registered_scan.header.frame_id ==
   /state_estimation.header.frame_id`（均为 odom）。若不一致，需加静态 TF 或 remap
   对齐后继续。
3. T2 运行 `./scripts/launch_pct_scan_real.sh`；RViz 中局部地图随点云实时更新。
4. RViz 设置 2D Nav Goal → 看到 `planning/bspline` 发布 → `/cmd_vel` 非零。
5. 先在空旷场地验证运动，再放置障碍物验证局部避障（grid_map 滑动窗口内 A* 绕障）。
6. 阶段 B 验收：`[astar] PASS` 测试 + 沿 `/pct_path` 全局参考的局部跟踪。

## 7. 风险与待定项

| 风险 | 说明 | 缓解 |
| --- | --- | --- |
| 帧不一致 | 两个话题 frame_id 不同则 cloud_is_world 失效 | 运行时先校验，必要时静态 TF |
| QoS 不匹配 | Odin 点云/位姿 QoS 与 planner 订阅不匹配则收不到 | 运行时用 `ros2 topic info` 核对，planner 侧调 QoS |
| 建图/规划性能 | 真机机载 CPU 上 Odin + planner 同时运行 | 关 RViz、降点云频率、`PCT_BUILD_JOBS` 调优 |
| 阶段 B 帧对齐 | 存好 tomogram 与新 odom 帧不对齐 | 阶段 B 专题解决（固定启动位姿 / 偏移参数） |
| TF 冲突 | 真机狗自身 TF 与 planner 冲突 | 真实分支不启动 robot_state_publisher |

## 8. 后续跟进（实现完成后）

- 更新 `CLAUDE.md`：补 pct_scan_ros2 真机接口（Odin 话题映射）与 SLAM 工作空间说明。
- 阶段 B 单独走 brainstorming → spec → 实现流程。
