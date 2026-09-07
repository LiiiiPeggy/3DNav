# 预建图导航模式 代码结构分析（Phase 1）

> 状态：仅检查，未改代码。分支 `realpctscan`。工作根：`pct_scan_ros2/src/PCT-SCAN-ROS2/`。
> 依据用户规范：Phase A（无图导航）必须保持原行为；Phase B（预建图导航）显式开启；
> 不修改 Odin SLAM / `/state_estimation` / `/registered_scan` / 无图默认 yaml / 原启动脚本 / SCAN 核心。

---

## 1. 当前无图导航数据流（已实机验证，禁止回归）

```
Odin SLAM（无图建图定位，外部，不修改）
   ├─ /state_estimation   nav_msgs/Odometry   帧：odom
   └─ /registered_scan    sensor_msgs/PointCloud2  帧：odom
              │
              ▼
   run.launch.py  (is_real_world:=true, navi_mode:=1, controller_mode:=closed_loop, sensor_type:=lidar)
   ├─ scan_planner_node  ← body_pose:=/state_estimation, cloud:=/registered_scan, goal:=/move_base_simple/goal
   │      grid_map：cloud_is_world=true, need_extrinsic=false, grid_map.frame_id = "world"(planner.yaml L27)
   └─ closed_loop_controller ← body_pose:=/state_estimation → /cmd_vel
              │
              ▼
           /cmd_vel → 底盘
```

启动入口：`scripts/launch_pct_scan_real.sh`（只透传 + 固定 4 参数，见 `src/planner/plan_manage/test/test_launch_pct_scan_real.py` 契约）。

关键事实（核实自代码）：
- 话题解析：`src/planner/plan_manage/launch/topic_resolver.py::_compute_topics`（real 分支固定返回 `/state_estimation`、`/registered_scan`）。
- SCAN 局部占据栅格话题：`grid_map/occupancy`、`grid_map/occupancy_inflate`（`src/planner/plan_env/src/grid_map.cpp:168-170`），header.frame = `grid_map.frame_id`。
- `planner.yaml:27` `grid_map.frame_id: world`；`run.launch.py:317` 声明 `world_frame`（默认 `odom`）但**从未使用**（注释：Phase B 帧对齐预留）。grid_map 实际发布帧 = planner.yaml 里的 `world`。
- 参数：`planner.yaml`（`fsm.navi_mode`、`grid_map.*`、`manager.max_vel` 等），`controllers.yaml`。

---

## 2. 新增预建图导航数据流（目标，显式开启）

```
Odin SLAM 重定位模式（外部：SLAM/3run_relocalization.sh + F1q9000.bin，不修改）
   ├─ /state_estimation   nav_msgs/Odometry   帧：odin_map（原点=预建图原点）
   └─ /registered_scan    sensor_msgs/PointCloud2  帧：odin_map
              │
              ▼
   [pct_scan_bridge]  bridge_node  （新 launch：real_prebuilt_scan.launch.py 组装）
       tomogram_path := maps/F19000_map.pickle     （允许覆盖）
       frame_id      := world_frame := odin_map
   ├─ 发布 /pct_tomogram（PointCloud2 代价场，帧 odin_map）
   ├─ 交互 Start/Goal marker（/pct_waypoints）＋ start_mode 适配（见 §4）
   │     Start 默认 = 当前机器人位姿（start_mode=1，读 /state_estimation）
   └─ 规划 → /pct_path（nav_msgs/Path，帧 odin_map，x/y/z=地图地面高）
              │  (SCAN 端 remap initial_path := /pct_path)
              ▼
   SCAN Mode 3（navi_mode:=3, closed_loop, world_frame:=odin_map）
   └─ scan_replan_fsm REFERENCE_PATH 分支订阅 initial_path(nav_msgs/Path)
       → prepareReferenceWaypoints（降采样/RDP + z += grid_map.body_height）
       → 局部 B-spline → closed_loop_controller → /cmd_vel → 底盘
```

新增独立启动：`scripts/launch_pct_scan_real_prebuilt.sh`（完全独立，不改 `launch_pct_scan_real.sh`）。

---

## 3. 需要新增的文件

| 文件 | 职责 |
| --- | --- |
| `src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py` | 组装：pct_bridge.launch.py + scan run.launch.py(real/Mode3) + RViz + start 适配。参数默认：`tomogram_path=maps/F19000_map.pickle`、`world_frame=odin_map`、`start_mode=1`；所有默认在 launch 内声明，shell 不写死，用户可用 `tomogram_path:=…` / `world_frame:=…` 覆盖 |
| `scripts/launch_pct_scan_real_prebuilt.sh` | 实机一键启动（source `_pct_scan_env.sh` + 预检重定位话题 + exec 上面 launch） |
| `src/planner/plan_manage/launch/real_prebuilt.rviz` | RViz 配置：Fixed Frame=`odin_map`；显示 PCT tomogram/`/pct_path`/Start-Goal marker + SCAN occupancy |
| `src/planner/plan_manage/start_pose_adapter.py`（可选新节点） | `/state_estimation`(Odometry) → `/initialpose`(PoseWithCovarianceStamped)；由 `start_mode:=1` 决定是否在 launch 里启动。零侵入（bridge 已订阅 /initialpose → `set_start`） |
| 契约测试（后续阶段） | 参照 `test_launch_pct_scan_real.py` 风格：新脚本/launch/rviz 的 file-contract + 断言不含 simulator/Go2 |

**/pct_path → SCAN 接口结论（已核实，无需自定义消息）：**
- `planner_node.py:141` `path_pub = create_publisher(Path,…)` → `/pct_path` 为 **nav_msgs/Path**；`msg.header.frame_id = self.frame_id`（L355）→ 置 `odin_map` 即可与 SCAN 同帧。
- Path 只有 x/y/z（无 yaw）。SCAN Mode 3（`scan_replan_fsm.cpp:96-98`）以 `nav_msgs/Path` 订阅 `initial_path`，经 `reference_path_utils.h::prepareReferenceWaypoints` 降采样 + RDP + `z += grid_map.body_height`。因此 `/pct_path`（地面高）可直接 remap 为 SCAN `initial_path`，**原则上不需要 adapter**；仅当帧对不上时才需要薄适配（预建图同帧 `odin_map`，预计直达）。

---

## 4. 需要修改的文件（尽量少、加性、不动默认行为）

| 文件 | 改动 | 兼容性说明 |
| --- | --- | --- |
| `src/planner/plan_manage/launch/run.launch.py` | ① 把 `world_frame` 默认从 `odom` 改为 `""`（仍是"未启用"）；② 当 `world_frame` 非空时，向 `scan_planner_node` 参数追加 `planner_overrides["grid_map.frame_id"]=world_frame`；③ 启动日志打印 `[SCAN] Override grid_map.frame_id = odin_map` 或 `Using default` | **唯一共享点**。改动是"死参数转条件参数"：无图脚本不设 `world_frame` → 与现状完全一致（仍用 planner.yaml 的 `world`），不回归 |
| `src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py`（新增）内部 Include `pct_bridge.launch.py` 时传 `frame_id:=world_frame` | 见新增表 | 纯新增 |
| （不改）`planner.yaml`、`launch_pct_scan_real.sh`、`topic_resolver.py`、`grid_map.cpp`、`scan_replan_fsm.cpp`、`closed_loop_controller.cpp` | — | 满足"不改无图默认 yaml / 原启动 / SCAN 核心 / controller" |

**frame 设计要点（第三阶段约束）：** 不改 `planner.yaml:27 grid_map.frame_id`；只在预建图 launch 传 `world_frame:=odin_map`，由 run.launch 动态 override。无图路径永远不传 → 永远走原配置。

---

## 5. Start/Goal 设计（第四阶段）

- 新增 `start_mode`（在 prebuilt launch 声明，默认 `1`）：
  - `1`（默认）：launch 内启动 `start_pose_adapter.py`，把 `/state_estimation`（当前机器人位姿，帧 odin_map）作为 `PoseWithCovarianceStamped` 发到 `/initialpose` → bridge `start_pose_cb → set_start(plan=False)`。**用户只需设 Goal**；设 Goal 时 `set_goal(plan=True)` → 自动出 `/pct_path`。
  - `0`：手动——RViz 拖 Start marker / 发 /initialpose（保留调试用）。
- 不依赖"手动把 Start 拖到机器人位置"作为默认。
- bridge 的 `scene_start_*` 只作兜底初始 marker，不作主流程。

---

## 6. 风险点

1. **run.launch.py 是共享文件**：若改坏，无图模式也回归。缓解：改动纯加性（`if world_frame_text:`），默认 `""`；契约测试断言"无 world_frame 时不注入 grid_map.frame_id"。
2. **帧对齐（odin_map 假说）**：约定重定位后 odin_map 原点=预建图原点、F19000_map.pickle 与其同坐标系。若实测有整体偏移/旋转：不改 SLAM/不改帧标签，在 prebuilt launch 里加可选静态对齐参数（Phase 8 预留），先用 marker 落 Start 兜底。
3. **occpancy/点云帧一致**：预建图要所有显示在 odin_map 重合，需 grid_map.frame_id=odin_map（本方案核心改动）＋ bridge frame_id=odin_map ＋ RViz Fixed=odin_map 三者一致。
4. **SCAN Mode3 平面闭环**：closed_loop 是 x/y/yaw 平面；跨楼层靠机器人自身爬升、SCAN 只投影跟踪。跨层必须先过"同层 5–10m 闭环"（Test 4）再验。
5. **start_mode=1 的时序**：需先收到 `/state_estimation` 再接受 Goal，否则起点用兜底 marker 值。launch 里让 adapter 仅在有 odom 时发，且 bridge 保持 marker 可手动修正。
6. **启动隔离**：任何默认 yaml / 默认参数改动都可能让无图"悄悄"进预建图 → 禁止；预建图只存在于新 launch + 新脚本 + run.launch 条件分支。
7. **接口一致性**：`/pct_path` 帧=bridge `frame_id`；若用户后续 `frame_id` 覆盖为非 odin_map，需同步覆盖 SCAN `world_frame`（launch 已保证二者同源同参）。
8. **QoS**：FSM 订阅 `initial_path` 用 `transient_local + reliable`（`scan_replan_fsm.cpp:98`）；bridge `/pct_path` 发布 QoS 需匹配（默认 `Path` qos 需确认 transient_local），否则晚连的 SCAN 收不到——实机验收需在 SCAN 启动后再规划，或确认 bridge QoS。

---

## 7. 后续阶段（Phase 2–7）执行顺序

1. **run.launch.py**：`world_frame` 条件注入 `grid_map.frame_id`（含日志）+ 契约测试。
2. **real_prebuilt.rviz** + 契约测试。
3. **real_prebuilt_scan.launch.py** + `start_pose_adapter.py`（start_mode）+ 契约测试。
4. **launch_pct_scan_real_prebuilt.sh** + 契约测试。
5. colcon build + `colcon test` + 无头冒烟（tomogram 加载、无帧错误）。
6. 真机 smoke：Test1 tomogram 发布 → Test2 goal→`/pct_path` 非空 → Test3 `/cmd_vel` 闭环 → Test4 同层 5–10m 闭环，最后跨层。
7. 提交前输出：改动文件清单 + 原因 + 无图兼容性说明 + 新启动方式 + 实机步骤。
