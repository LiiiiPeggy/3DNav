# 第四阶段：打通 PCT 全局规划与 SCAN 局部规划

这一阶段完成的是完整导航链路中的算法接口：PCT 在 tomogram 上搜索跨楼层全局
路线，并通过 `/pct_path` 发布 `nav_msgs/msg/Path`；SCAN 在 Mode 3 中接收该路线，
生成严格沿 PCT 折线的全局参考轨迹，再滚动生成局部 B-spline。

```text
tomogram pickle
    ↓
PCT native A*（跨 tomogram layer）
    ↓  /pct_path: nav_msgs/Path
三维降采样 + RDP + 分段线性全局参考
    ↓
SCAN 滚动局部 B-spline
    ↓  planning/bspline
三维 open-loop 演示执行器
```

## 1. 为什么需要 `pct_scan_bridge`

PCT 的不同场景有不同数量的 tomogram layer。第三方 ROS 2 wrapper 曾把 native
planner 的层数写成固定值，C++ A* 因此可能按错误的内存布局建图。
`pct_scan_bridge` 直接从 pickle 读取 `self.n_slice`，再把实际层数传给 native
planner。这个修正同时适用于 Building、Plaza 和 Spiral。

## 2. 先做无 ROS 图形界面的 A* 验收

在仓库根目录构建桥接包：

```bash
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh --packages-select pct_scan_bridge scan_planner
```

然后选择一个场景运行：

```bash
./scripts/test_pct_building_astar.sh --force
./scripts/test_pct_plaza_astar.sh --force
./scripts/test_pct_spiral_astar.sh --force
```

脚本会打印路径点数、经过的 layer、高度增量和路线长度。A* 失败时，先检查端点是否
能吸附到可通行栅格，再检查两个端点是否通过跨层 gateway 连通。

## 3. 为什么不把所有栅格点直接交给 SCAN

PCT 的输出是 tomogram 栅格上的 A* cell 序列。直接保留全部 cell 没有必要，也会
增加后续投影、显示和局部目标采样的开销。

联合启动先按三维距离去重，再执行三维 Ramer-Douglas-Peucker (RDP) 简化。
RDP 会保留楼梯、坡道和明显转角，同时把几何误差限制在配置值内。

相关启动参数是：

```text
reference_path_min_distance:=0.15
reference_path_simplify_tolerance:=0.1
```

整条多楼层路线不能再做一次全局 minimum-snap 拟合，否则曲线可能在弯道和楼层
连接处离开 PCT 走廊。当前全局参考使用分段线性参数化。局部平滑仍由 SCAN 的
B-spline 完成。

## 4. 一条命令启动 PCT + SCAN

```bash
./scripts/launch_pct_scan_building.sh
```

脚本自动完成以下工作：

- source ROS 2 Humble 和本工程 overlay；
- 补充 GTSAM 与 OSQP 的运行库路径；
- 加载 Building PCD 和 tomogram；
- 启动 PCT 交互路点、SCAN Mode 3、运动学机器人和 RViz。

在 RViz 中选择 `Interact`，拖动 `Start` 和 `Goal` 球体。右键任意球体并选择
`Plan route` 后，PCT 发布新路径，SCAN 会立即更新全局参考轨迹。

验证固定起终点且不打开 RViz：

```bash
./scripts/launch_pct_scan_building.sh \
  use_rviz:=false \
  plan_on_startup:=true \
  publish_tomogram:=false \
  enable_interactive_markers:=false
```

关键成功日志为：

```text
Trajectory with 874 poses published on /pct_path
Reference path reduced from 874 poses to 94 trajectory waypoints
Reference path accepted
Received trajectory 1
Received trajectory 2
Received trajectory 3
```

`/pct_path` 两端使用 `reliable + transient_local + depth 1`。因此即使 SCAN 稍晚
完成初始化，也能取得 PCT 保存的最近一条路径，无需重复运行 A*。

## 5. 三维跟踪方式

`closed_loop` 的 `go2_kinematic_sim` 接收平面 `cmd_vel`，只积分 x、y 和 yaw，
因此它不能用于跨层视频。论文场景启动器默认使用 `open_loop`：执行器直接采样
SCAN 输出的三维 B-spline，机身 z 会随坡道上升。

SCAN 的每条局部 B-spline 只覆盖一个 planning horizon。状态机必须在局部段结束时
继续规划下一段，直到全局 PCT 终点；否则会走约一米后错误回到 `WAIT_TARGET`。
该修复及跨层验收见
[`tutorial_phase5_route_tracking.md`](tutorial_phase5_route_tracking.md)。这个执行器
只负责 RViz 演示，不代表真实动力学或低层 MoE policy。
