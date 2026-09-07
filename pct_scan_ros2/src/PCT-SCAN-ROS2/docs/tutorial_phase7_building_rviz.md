# 阶段 7：Building 地图上的 RViz-only PCT + SCAN 导航

这一阶段把项目主线切换为适合录制视频和面试讲解的 Building 演示。系统不启动
Gazebo，也不计算接触动力学：PCT 在 tomogram 上搜索跨楼层路线，SCAN 把全局折线
转换成连续的局部 B-spline，开环控制器沿轨迹更新 Go2 位姿，RViz 同时显示地图、
路线、局部轨迹和机器人模型。

## 1. 数据流与职责边界

```text
building2_9.pickle
        │
        ▼
PCT A* ─────── /pct_path（橙色全局路线）
                         │
                         ▼
                 SCAN Mode 3
                         │
                         ▼
             /planning/bspline（局部轨迹）
                         │
                         ▼
             open_loop_controller
                         │
                         ▼
              /quad_0/body_pose ──► RViz Go2

building2_9.pcd ──────────────────► RViz Global Map
```

这里的 PCD 与 pickle 来自同一个 Building 点云并直接使用 `world` 坐标系，所以不需
要额外的旋转、平移或比例变换。PCT 路径里的 `z` 表示地面高度；SCAN 会按
`grid_map.body_height=0.4 m` 生成机身高度。

## 2. 为什么默认关闭模拟局部感知

SCAN 原本的局部感知器面向自由空间 3D 点云，会用双圆柱对障碍点进行膨胀。
Building 楼梯是稠密的台阶点云：PCT 的地形模型认为它可通行，但 SCAN 默认
`0.25 m` 半径会把窄楼梯走廊膨胀关闭。它还需要持续渲染模拟激光点云，录制视频时
会占用大量 CPU。

因此默认设置 `enable_local_sensing:=false`：PCD 仍作为清晰的 RViz 场景显示，
可通行性由 PCT tomogram 提供，SCAN 继续完成全局路线采样、局部 B-spline 和连续
重规划。这与“无 Gazebo、只做视频演示”的目标一致，但它不是用于验证真实机器人
局部避障安全性的配置。

当前视频演示不要添加 `enable_local_sensing:=true`。如果添加它，稠密台阶会进入
SCAN 的碰撞膨胀层，局部 A* 或 B-spline refinement 连续失败后，状态机会进入
`EMERGENCY_STOP/WAIT_TARGET`，画面上就表现为“机器狗走着走着停下”。启动器现在
会对此打印风险提示。

只有在后续专门研究原始 SCAN 点云碰撞层时才显式开启：

```bash
./scripts/launch_pct_scan_building.sh enable_local_sensing:=true
```

开启后需要进一步统一 PCT 的地形足迹与 SCAN 的碰撞体模型，不能把该选项当作当前
演示的验收入口，也不能用它判断 PCT 全局路线是否正确。

## 3. 构建与资源检查

```bash
cd /mnt/sage_data/workspace/PCT-SCAN-ROS2

./scripts/build_workspace.sh --packages-select pct_scan_bridge scan_planner
```

检查 Building 的 PCT A*，不启动任何 ROS 图形界面：

```bash
./scripts/test_pct_building_astar.sh --force
```

通过时应看到：

```text
shape=(8, 285, 150) resolution=0.100 m
layers=[0, 1, 2, 3, 4, 5, 6]
height_gain=13.50 m
clearance: p05=0.20 m median=0.41 m below_0.20m=38/872
```

## 4. 启动并交互规划

```bash
./scripts/launch_pct_scan_building.sh
```

RViz 的主要显示项是：

- `Global Map`：灰色 Building 原始 PCD；
- `PCT Full Cost Tomogram`：按论文范围显示的完整 PCT 代价层；
- `PCT Global Path`：PCT 发布的橙色跨层全局路线；
- `Robot Path`：Go2 已执行的绿色路径；
- `PCT Start and Goal`：交互式起终点 marker。

操作流程：

1. 选择 RViz 顶部的 `Interact` 工具；
2. 拖动 Start 和 Goal。跨楼层规划时不仅要改变 `x/y`，也要沿蓝色 Z 轴把目标移到
   对应楼层；
3. 右键 Start 或 Goal，点击 `Plan route`；
4. 观察橙色 PCT 路线先出现，随后 SCAN 逐段发布局部轨迹，Go2 沿路线运动。

默认 Start 为 `(5, 5, 0)`，Goal 为 `(-6, -1, 14)`。PCT 会分别吸附到有效的
第 0 层和第 6 层，适合作为固定的跨楼层演示。

如果只想自动跑默认端点：

```bash
./scripts/launch_pct_scan_building.sh plan_on_startup:=true
```

无界面验收命令：

```bash
./scripts/launch_pct_scan_building.sh \
  use_rviz:=false \
  plan_on_startup:=true \
  publish_tomogram:=false \
  enable_interactive_markers:=false
```

## 5. 如何判断链路工作正常

终端日志应依次出现：

```text
Trajectory with 874 poses published on /pct_path
Reference path reduced from 874 poses to 94 trajectory waypoints
Reference path accepted
Received trajectory 1
Local reference window complete (...); continuing along the global path
Received trajectory 2
```

`Received trajectory N` 持续增加表示 SCAN 正在滚动重规划，不是一次性播放 PCT
折线。加入中线偏置后的无界面验收连续生成到第 26 条局部轨迹，机身从
`z=0.40 m` 上升到 `z=4.90 m`，期间没有进入 `EMERGENCY_STOP`；测试随后由定时器
主动结束，而不是规划器自行停止。

## 6. 关键参数为什么这样设置

### PCT 为什么需要中线偏置

原生 PCT A* 使用 8 邻域，并把 tomogram traversability 作为单元代价。不过原生
代码还有一条规则：`step_cost_weight * cell_cost < 5` 时把附加代价清零。默认
`step_cost_weight=0.2`、可通行阈值为 `20`，所以普通可通行单元全部满足
`0.2 * cell_cost <= 4`。结果就是 A* 基本只优化路程，左右两侧同样可通行时可能
贴着楼梯边缘走。

桥接层现在对每个 tomogram 层做二维距离变换，并为离不可通行单元较近的位置加入
软代价。它不会删除原来可通行的单元，因此窄楼梯仍然连通；它只是让多条可行路线
中净空较大的路线更便宜。Building 默认参数是：

- `centerline_bias_enabled=true`；
- `preferred_clearance=0.5 m`；
- `clearance_cost=20.0`；
- `astar_step_cost_weight=1.0`。

基准路线中，距不可通行区域不足 `0.2 m` 的 A* 单元从 `386/769` 减少为
`38/872`，中位净空从约 `0.14 m` 提升为 `0.41 m`。剩余低净空点主要位于跨层
gateway，是楼层切换必须经过的位置。这是“偏向中线”的软约束，不保证每个点都
严格落在楼梯几何中轴线上。

如果自己的交互端点产生的路线仍然偏边，可小幅增大目标净空：

```bash
./scripts/launch_pct_scan_building.sh preferred_clearance:=0.6
```

不要一开始就设置得很大；过强的偏置会让 A* 为追求开阔区域而明显绕路。复现实验
中若要关闭这项桥接增强，可传入 `centerline_bias_enabled:=false`。

### SCAN 跟踪参数

- `reference_path_simplify_tolerance=0.1 m`：与 tomogram 分辨率一致，去掉栅格
  路径的锯齿，同时把最大偏差限制在一个体素内；
- `planning_horizon=0.8 m`：在楼梯和转角上使用较短局部窗口；
- `max_vel=0.5 m/s`：减小折线路径切换方向时的加速度，让画面更平滑；
- `controller_mode=open_loop`：直接执行三维 B-spline，能够改变高度；二维
  `closed_loop` 只跟踪 `x/y/yaw`，不适合跨层演示；
- `use_sim_time=false`：没有 Gazebo `/clock`，所有节点使用系统时间。

Building pickle 由 PCT 虚拟环境的 NumPy 2 写出。启动脚本会自动把该环境的
`site-packages` 放在 `PYTHONPATH` 最前面，避免系统 NumPy 1.x 加载时出现
`ModuleNotFoundError: numpy._core`。

### 为什么论文代价图是蓝—绿—黄—红

论文的 Building 图包含三种不同的渲染，不能直接比较它们的颜色：原始三维模型
使用材质色；点云图按 Z 高度着色；PCT tomogram 图才按 travel cost 着色。本工程
`/pct_tomogram` 发布 pickle 中的完整原始 `traversability` 数据，其数值范围是
`0--50`。论文式显色使用的却不是 `0--50`，而是作者 RViz 配置中的固定
`-20--60` 强度范围和 Rainbow 色表。这个额外留出的上下界会把原始代价 `0`
映射到蓝色区，而不是 Rainbow 色表端点的洋红色。

可按下面的方式理解代价色：

| 原始代价 | RViz 颜色趋势 | 含义 |
| ---: | --- | --- |
| 接近 `0` | 深蓝 | 平坦、净空充足，优先通行 |
| 中等代价 | 青/绿/黄 | 靠近边缘，或坡度、台阶、净空条件变差 |
| 接近 `50` | 橙/红 | 膨胀边界或 barrier，不应通行 |

这里必须区分“显示范围”和“搜索阈值”：

- 点云仍发布 `0 <= cost <= 50`，所以能看到论文图中的完整暖色边界；
- RViz 用 `-20--60` 做颜色归一化，以复现作者配置的蓝—青—绿—黄—橙配色；
- A* 仍只允许 `cost <= 20` 的单元进入搜索；
- `20 < cost <= 50` 只提供环境结构和风险边界，不会变成可走区域。

以前桥接器在发布点云时沿用了 A* 的 `20` 阈值，导致论文中最醒目的黄、红代价环
被提前过滤。仅修改 RViz 色标无法补回这些单元；现在发布器与规划器已经解耦。
当前 Building 数据发布 `95,501` 个可见 tomogram 单元，其中
`48,054` 个代价高于 `20`，`29,254` 个代价等于 barrier `50`。

如果只想截取接近论文 (b3) 的代价图，可在 RViz 中临时关闭 `Global Map`、机器人
和轨迹，仅保留 `PCT Full Cost Tomogram`，再切换到合适的俯视或斜俯视视角。

## 7. 画面不清楚时怎么调

专用 `building.rviz` 使用 `0.10 m` tomogram 方块，与 Building 体素分辨率一致。
如果完整代价场显得拥挤：

1. 临时关闭 `Global Map`，只观察 PCT 层和路线；
2. 将 `PCT Full Cost Tomogram/Alpha` 调低到 `0.5`；
3. 用鼠标中键移动焦点到目标楼层，再滚轮放大；
4. 录制全局镜头时保留全部层，录制机器人特写时只保留 PCD、全局路线和 Robot
   Path。

到此为止，项目已经具备可交互选择跨楼层端点、PCT 全局搜索、SCAN 连续轨迹生成
和 RViz 四足动画这条完整的视频演示链路。
