# 第一阶段：把 PCT 全局路径接入 SCAN

## 1. 本阶段目标

这一阶段只验证导航算法消息链，不启动 Gazebo：

```text
RViz Start/Goal
       │ 右键 Plan route
       ▼
PCT native 3D A* ──/pct_path──▶ SCAN local B-spline
                                      │ /planning/bspline
                                      ▼
                              closed-loop controller
                                      │ /quad_0/cmd_vel
                                      ▼
                               Go2 kinematic model
```

验收条件是：PCT 产生跨层 `nav_msgs/Path`，SCAN 接受参考路径并发布
`scan_planner_msgs/Bspline`，控制器输出速度，Go2 的平面位置发生变化。

## 2. 为什么 PCT 后面仍然需要 SCAN

PCT tomogram 不是普通二维占据栅格。它把不同高度切片中的可通行代价、地面高度
和跨层 gateway 组织成三维搜索空间。PCT 的 A* 负责回答“从哪个楼层、经过哪段
楼梯、最终走到哪里”，输出的是全局地面折线。

SCAN 解决的是另一个尺度的问题：结合局部点云和机器人当前状态，把全局折线的
前向一段变成连续、可跟踪、满足速度和加速度限制的 B-spline，并在局部障碍出现
时重规划。因此本组合采用明确分工：

- PCT：只运行 native 3D A*，保持全局拓扑和楼层选择；
- SCAN：运行局部 B-spline 优化和闭环控制；
- 不启用 PCT 自带 GPMP 优化，避免两套优化器重复平滑并改变端点或楼梯高度。

README 中说的 A* 正是 PCT 内部的离散搜索阶段，不是用传统二维 A* 替代 PCT。

## 3. 三个接口合同

### 3.1 坐标

RViz、PCD 和 SCAN 都使用 `world`。当前 multifloor tomogram 使用 PCT 坐标，和
可视场景相差绕 Z 轴 180°：

```text
x_pct = -x_world
y_pct = -y_world
z_pct =  z_world
```

转换只在 PCT ROS 节点边界发生。交互 marker 和发布的 `/pct_path` 仍然位于
`world`，所以 SCAN 不需要知道 PCT 的内部坐标。

### 3.2 高度

PCT Path 的 `z` 是支撑面/地面高度。SCAN 在接收路径时增加一次
`grid_map.body_height=0.4`，得到机身中心高度。不能在 PCT 节点再加一次，否则
机器狗会悬空约一个机身高度。

### 3.3 QoS

`/pct_path` 两端使用 `reliable + transient_local + depth 1`。这样即使 SCAN 比
PCT 晚启动，也能收到最近一条全局路径；默认 volatile 订阅无法保证这一点。

## 4. 构建

依赖准备完成后，从本仓库直接构建 PCT、bridge 与 SCAN：

```bash
# 在 PCT-SCAN-ROS2 仓库根目录
./scripts/setup_pct_dependencies.sh
./scripts/build_workspace.sh
```

## 5. 启动与交互

脚本会按正确顺序 source ROS 和本仓库 overlay，并补上仓库内 `.deps` 的 PCT
原生库动态链接路径：

```bash
./scripts/launch_pct_scan_phase1.sh
```

这个早期 Multifloor 示例只从本仓库的 `assets/phase1` 读取资产，不访问外部
`pct_scan` 文件夹。当前主线使用 Building，启动入口是
`./scripts/launch_pct_scan_building.sh`。

进入 RViz 后：

1. 确认 Fixed Frame 是 `world`。
2. 选择工具栏的 `Interact`。
3. 拖动绿色 Start 和红色 Goal；拖动 Z 轴可选择不同楼层。
4. 松开 marker 后，它会吸附到附近可通行 tomogram 单元。
5. 右键任意一个 marker，点击 `Plan route`。

拖动时不自动规划是刻意设计：跨楼层 A* 不应在每次鼠标反馈时重复运行，显式
Plan 也让演示的操作语义更清楚。

RViz 中的 `PCT Traversable Tomogram` 是选择起终点时的主要参考。Multifloor
模式不会显示全部 26 个切片，而是只显示包含已验证路线锚点的 F1 layer 8 和 F2
layer 15 主连通域，并限制在各楼面高度上下 0.4 m。marker 的吸附也使用同一掩码，
因此不会落到孤立噪声小岛。蓝色表示较低通行代价，暖色表示代价更高。淡灰色
`Global Map` 只提供建筑轮廓。`Sensor Cloud`、`Occupancy`、`Inflated Occupancy`
和 `Sliding Map Bounds` 默认关闭；需要调试 SCAN 局部感知时再单独开启。

默认跨层示例可用于无界面测试：

```bash
./scripts/launch_pct_scan_phase1.sh use_rviz:=false plan_on_startup:=true
```

## 6. 如何判断成功

终端应依次出现类似日志：

```text
Trajectory with 100 poses published on /pct_path
Reference path accepted
[FSM]: from WAIT_TARGET to GEN_NEW_TRAJ
Received trajectory 1, duration ...
```

另开终端可检查接口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /pct_path --once
ros2 topic echo /planning/bspline --once
ros2 topic hz /quad_0/cmd_vel
```

## 7. 当前阶段的已知边界

当前 Go2 运动学模型只积分平面 `vx/vy/yaw`，没有由碰撞或地形求解机身 `z`。
因此它能证明 PCT→SCAN→控制器接口闭环，也能在一楼平面移动，但跨层演示走到
楼梯后会出现高度状态不一致。第二阶段将增加“沿 SCAN 轨迹/支撑面更新机身高度”
的纯运动学机制，再接程序化步态；这不需要 Gazebo 物理。

## 8. 面试时可以这样讲

“我没有把 PCT 和 SCAN 当成两个竞争的 planner。PCT 在 tomogram 上解决全局
三维拓扑，尤其是跨楼层 gateway；SCAN 消费 PCT 的地面参考路径，并根据局部
点云生成动态可行 B-spline。工程上最容易出错的是坐标变换、高度重复补偿和
ROS 2 QoS，所以我把它们定义成显式接口合同，并用 `/pct_path → bspline → cmd_vel`
的分层冒烟测试逐级验证。”

## 9. 代码阅读入口

- PCT 交互与发布：`pct_planner/pct_planner/planner_node.py`
- PCT native A* 路径转换：`pct_planner/pct_planner/planner_wrapper.py`
- SCAN 接收参考路径：`src/planner/plan_manage/src/scan_replan_fsm.cpp`
- 组合启动：`src/planner/plan_manage/launch/pct_scan_phase1.launch.py`
- 闭环控制：`src/planner/plan_manage/src/closed_loop_controller.cpp`
- 运动学模型：`src/planner/plan_manage/src/go2_kinematic_sim.cpp`
