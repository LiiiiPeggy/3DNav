# PCT-SCAN-ROS2 × Odin SLAM 接口对齐设计（无全局地图导航）

日期：2026-09-01
分支：`realpctscan`
状态：已批准（2026-09-01，方案 A）；同日工程化审查修订（术语、接口验证前置、QoS、
控制链验证、验收测试、文件清单）

## 1. 背景与目标

`pct_scan_ros2/src/PCT-SCAN-ROS2` 现有的真机分支（`run.launch.py` 中 `is_real_world=true`）
写死的是从 FAST-LIO 工作空间移植来的 `/LIO/*` 话题，与实际部署的 **Odin SLAM**
话题不一致。本设计将规划器的真实接口对齐到 Odin，为后续机器狗实机部署打基础。

目标：
- 使用 Odin 实时点云 `/registered_scan`（PointCloud2 XYZI，odom 帧）与定位
  `/state_estimation`（nav_msgs/Odometry，odom 帧）驱动 SCAN 局部规划。
- **无全局地图导航（map-free local navigation）**：不使用预存地图；不进行重定位；
  Odin 每次启动产生新的 odom 坐标系；planner 使用实时点云构建局部占据地图进行避障。
- 分两阶段：**A** 先打通实时局部避障（SCAN Mode 1，RViz 目标点）；**B** 后加 PCT
  全局参考路线（复用预生成的 tomogram）。
- 控制链：`closed_loop_controller → /cmd_vel`（geometry_msgs/Twist，平面 x/y/yaw）。

**术语澄清**：本设计所称"无全局地图导航"强调**不依赖预存全局地图**，而非"没有任何
地图"。局部占据地图始终存在，由实时点云在线构建，用于局部避障。

## 2. 范围

- **阶段 A（本次实现）**：接口对齐 + 实时局部避障。
- **阶段 B（后置，仅设计大纲）**：存好的 tomogram 提供 PCT 全局参考。
- 明确排除：重定位模式（Odin `3run_relocalization.sh`）、PCT 全局规划器（阶段 B 才接）、
  深度传感器（阶段 A 真实分支仅支持 `sensor_type=lidar`）。

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

`cloud_is_world=true` 成立的前提：**`/registered_scan` 点云坐标已位于 odom/world 帧**
（见第 5 节"启动前接口验证"，实现前置条件）。

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
- 无重定位：每次 Odin 新建 odom 帧，planner 全程跟随 —— 这是"无全局地图导航"的核心约定。

### 4.5 sensor_pose 设计说明

**阶段 A**：`sensor_pose = /state_estimation`（与 `body_pose` 同一话题），并显式假设
**lidar–body 外参影响可以忽略**（Odin 定位与注册点云同帧，body 位姿近似等价于
lidar 位姿）。

**阶段 B / 近障碍高精度导航**：应考虑引入 **lidar pose topic**，或
`state_estimation + lidar extrinsic`（`need_extrinsic=true` + 外参标定），以精确
刻画 lidar 光学中心到车体/IMU 的相对位姿。

**注意**：`body_pose`（FSM 起点/偏航）与 `sensor_pose`（地图窗口中心）二者物理意义
不同；阶段 A 因外参可忽略而近似相等，不代表长期可混用。

### 4.6 need_extrinsic 代码检查（实现前置要求）

修改参数**之前**必须核对代码逻辑：

```bash
grep -R "need_extrinsic" src/
```

确认 `need_extrinsic=false` 时：
- **不加载** FAST-LIO 外参（不读取 `lidar_extrinsic`）；
- **不进行**错误的坐标变换（`grid_map.cpp` 中跳过 `pose_r * extrinsic`）；
- **直接**使用 odom 帧点云（`cloud_is_world=true` 时点按世界帧直接用）。

**不要只改 yaml**：需确认 `grid_map.cpp` 的 `sensorPoseCallback` / `cloudCallback`
在该配置下的变换路径符合预期（无外参叠加、无双重变换）。

## 5. 启动前接口验证（实现前置条件）

在启动 planner **之前**必须完成本检查（从验收阶段提前为前置条件）。`cloud_is_world=true`
依赖 `/registered_scan` 已在 odom/world 帧，若两个话题帧不一致，禁止直接使用该配置。

**检查步骤**：

1. 点云帧：

```bash
ros2 topic echo /registered_scan --once
```

确认 `header.frame_id`。

2. 位姿帧：

```bash
ros2 topic echo /state_estimation --once
```

确认 `header.frame_id`。

**要求**：`/registered_scan` 与 `/state_estimation` **必须属于同一坐标系**。

**若 frame_id 不一致**：
- 禁止直接使用 `cloud_is_world=true`；
- 需增加**静态 TF 转换**（发布 `odom → <实际点云帧>` 的静态 TF）；
- 或增加**点云转换节点**（将点云转到位姿所在帧后再送入 planner）。

## 6. ROS2 QoS 兼容性

Odin SLAM 与 planner 之间**必须检查 QoS 是否匹配**，不要假设一定兼容。

**验证命令**：

```bash
ros2 topic info /registered_scan -v
ros2 topic info /state_estimation -v
```

**建议配置**：

- **点云订阅**（`/registered_scan`）：`sensor_data` QoS —— `best_effort` + `volatile`，
  与 Odin 发布端匹配（丢帧容忍，不缓存旧帧）。
- **Odometry**（`/state_estimation`）：根据 Odin 实际 publisher QoS 配置调整
  （可能是 `reliable` + `transient_local` 或 `best_effort`），以 `ros2 topic info -v`
  实测为准。

**不匹配处理**：若两端 QoS 不一致导致收不到数据，优先在 planner 订阅侧调整 QoS
（`rclcpp::SensorDataQoS()` / `rclcpp::QoS` 按发布端配置），必要时加中继节点。

## 7. 阶段 B 大纲（后置，仅接口设计）

1. **建图导出**：跑一遍 Odin 映射模式，`4savemap.sh` → `5bin2ply.sh` → `6ply2pcd.sh`
   得到环境 PCD。
2. **生成 tomogram**：用 `scripts/generate_tomogram.sh` / `generate_pct_tomogram.py`
   从 PCD 生成 PCT tomogram（pickle），存入 `assets/<scene>/`。
3. **全局接线**：启动 pct_planner + pct_scan_bridge → `/pct_path`；
   planner `navi_mode=3`（`initial_path_topic:=/pct_path`）→ SCAN 沿全局参考做局部避障。

**阶段 B 范围限定**：本阶段当前**只记录接口设计**，不实现：
- 重定位（relocalization）；
- map alignment（地图对齐）；
- tomogram registration（tomogram 配准）。

**⚠️ 帧对齐风险（核心待定项）**：存好的 tomogram 位于"建图那次会话"的 odom 帧；
每次新运行 Odin 是新的 odom 原点。无重定位下，仅当狗在**同一位置同朝向**启动时
两帧一致。需在阶段 B 定方案：固定启动位姿，或为 tomogram 提供粗对齐偏移参数
（可利用阶段 A 预留的 `world_frame` 参数）。

## 8. 验证与验收（真机实时）

### 8.1 前置
先完成第 5 节"启动前接口验证"（帧一致性）与第 6 节 QoS 核对，再启动 planner。

### 8.2 cmd_vel 控制链两阶段验证

**第一阶段（不连接机器狗）**：只验证链路
`SCAN planner → planning/bspline → cmd_vel`，不立即连接狗的控制桥。

```bash
ros2 topic echo /cmd_vel
```

确认消息类型为 **geometry_msgs/msg/Twist**，且目标点设置后非零输出。

**第二阶段（接入机器狗）**：第一阶段的 `planning/bspline` 与 `cmd_vel` 均正常后，
再接机器狗控制桥。**避免规划器问题与机器人控制问题混合排查**。

### 8.3 验收测试

- **Test 0 — 空旷环境点到点**：验证
  `/state_estimation → grid_map → planning/bspline → cmd_vel`，空旷场地设置目标点，
  命令随位姿更新、轨迹平滑、无异常跳变。
- **Test 1 — 静态障碍绕行**：验证
  `/registered_scan → grid_map → 局部避障`，放置静态障碍物，确认滑动窗口内 A* 绕行，
  不碰撞。
- **Test 2 — 近距离障碍**：验证 `scan_min_range` / `collision_radius` 等参数对
  近距离障碍的处理（Odin adapter 已做 0.2 m 最小距离滤波，planner 侧
  `collision_radius`/`collision_offset` 按狗紧凑足迹 0.12 配置）。

### 8.4 启动与基础检查

1. T1 运行 `SLAM/2run_slam.sh`；`ros2 topic hz /registered_scan`、`ros2 topic echo
   /state_estimation` 确认话题有数据且位姿在动。
2. 完成 8.1 前置检查。
3. T2 运行 `./scripts/launch_pct_scan_real.sh`；RViz 中局部地图随点云实时更新。
4. RViz 设置 2D Nav Goal → 看到 `planning/bspline` 发布 → `/cmd_vel` 非零
   （先 8.2 第一阶段，再接狗）。
5. 先在空旷场地验证运动，再放置障碍物验证局部避障。

## 9. Expected modification files

**主要修改（`pct_scan_ros2/src/PCT-SCAN-ROS2/`）**：

| 文件 | 改动 |
| --- | --- |
| `src/planner/plan_manage/launch/run.launch.py` | 新增 launch 参数 `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`cmd_vel_topic`、`world_frame`、`publish_robot_state`；真实分支默认值改为 Odin 话题；真实分支不启动 robot_state_publisher |
| `src/planner/plan_manage/config/planner.yaml` | grid_map 参数（`grid_map.*` 命名空间，仓库**无独立 grid_map.yaml**）：`cloud_is_world=true`、`need_extrinsic=false`；碰撞参数按需 |
| `scripts/launch_pct_scan_real.sh` | **新增**：真实启动脚本（复用 `_pct_scan_env.sh`，透传参数） |

**保持不修改**：
- SCAN 核心算法（`scan_replan_fsm` / `planner_manager`）；
- GridMap 算法（`plan_env`）；
- 轨迹优化（`bspline_opt`）；
- 前端路径搜索（`path_searching`）。

## 10. 风险与待定项

| 风险 | 说明 | 缓解 |
| --- | --- | --- |
| 帧不一致 | 两个话题 frame_id 不同则 cloud_is_world 失效 | **启动前必须校验**（第 5 节），必要时静态 TF 或点云转换节点 |
| QoS 不匹配 | Odin 点云/位姿 QoS 与 planner 订阅不匹配则收不到 | 用 `ros2 topic info -v` 核对，planner 侧按发布端调整 QoS |
| 建图/规划性能 | 真机机载 CPU 上 Odin + planner 同时运行 | 关 RViz、降点云频率、`PCT_BUILD_JOBS` 调优 |
| 外参假设 | 阶段 A 忽略 lidar–body 外参，近障碍精度受限 | 阶段 B / 高精度场景引入 lidar extrinsic（4.5） |
| 阶段 B 帧对齐 | 存好 tomogram 与新 odom 帧不对齐 | 阶段 B 专题解决（固定启动位姿 / 偏移参数） |
| TF 冲突 | 真机狗自身 TF 与 planner 冲突 | 真实分支不启动 robot_state_publisher |

## 11. 后续跟进（实现完成后）

- 更新 `CLAUDE.md`：补 pct_scan_ros2 真机接口（Odin 话题映射）与 SLAM 工作空间说明。
- 阶段 B 单独走 brainstorming → spec → 实现流程。
