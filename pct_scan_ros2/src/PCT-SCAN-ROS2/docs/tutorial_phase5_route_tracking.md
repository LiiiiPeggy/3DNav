# 第五阶段：让 SCAN 严格沿 PCT 路线连续上楼

## 1. 现象不是一个单独的控制器调参问题

RViz 中橙色 `/pct_path` 是 PCT 的全局路线，绿色 `/quad_0/path` 是机器人实际
轨迹。旧实现中绿色轨迹会明显切离橙色路线，而且走完第一个局部段后可能停止。
这里实际包含三个问题：

1. 整条跨层路线经过全局 minimum-snap 拟合后发生严重过冲；
2. 状态机把“局部 B-spline 结束”误当成“全局导航结束”；
3. 平面闭环仿真只更新 x、y、yaw，本身不可能改变 z。

## 2. 全局参考负责守住走廊，局部轨迹负责平滑

旧实现会对完整跨层路线做一次全局拟合。弯道较急或楼层连接较近时，拟合曲线可能
离开原始折线。机器人看起来像在跨墙或绕开楼梯，问题其实出在局部规划器收到的
全局参考已经偏离 PCT 走廊。

当前实现把职责拆开：

```text
PCT 折线 ──RDP(0.10 m)──> 三维关键点
                              │
                              ├─ 分段线性全局参考：不抄近路
                              │
                              └─ 局部窗口：SCAN B-spline 平滑与避障
```

分段线性并不意味着机器人动作是折线。它只定义全局“导航走廊中心线”；真正发布给
执行器的仍是 SCAN 优化后的三次 B-spline。Building 默认使用 `0.8 m` 的短窗口，
减少楼梯和转角处的局部捷径。

## 3. 局部完成不等于全局完成

每条 SCAN B-spline 只走大约一个 planning horizon。状态机现在检查全局参考的
`last_progress_time_`：

- 尚未到全局末端：从 `EXEC_TRAJ` 转到 `REPLAN_TRAJ`，继续生成下一窗口；
- 已到全局末端：清除目标并进入 `WAIT_TARGET`。

无界面联调会持续收到递增的 trajectory 编号。编号停在 1，通常表示状态机没有从
局部完成状态继续进入 `REPLAN_TRAJ`。

## 4. 为什么跨层演示使用 open-loop

这里的 `open_loop` 是演示专用的三维轨迹执行器：它直接采样
`planning/bspline`，发布 x、y、z 和由速度方向得到的 yaw。`closed_loop` 则通过
平面 `cmd_vel` 驱动简化运动学模型，只适合二维演示。

这不代表真实四足机器人应当开环控制。真机链路仍应是：SCAN 期望运动 → 速度或
足端命令 → MoE 低层 policy → 关节执行与状态反馈。本项目当前目标是视频展示，
所以论文场景启动器默认使用三维 `open_loop`。

## 5. 验证 Building 跨层跟踪

先运行 PCT A* 测试：

```bash
./scripts/test_pct_building_astar.sh --force
```

测试通过后自动播放默认路线：

```bash
./scripts/launch_pct_scan_building.sh plan_on_startup:=true
```

另开终端观察机身高度：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /quad_0/body_pose --field pose.pose.position.z
```

日志应持续出现递增的 `Received trajectory N`，而不是 trajectory 1 后进入
`WAIT_TARGET`。RViz 中橙色是 PCT 全局路线，绿色是机器人历史轨迹。机器人进入
楼梯后，`/quad_0/body_pose` 的 z 应随局部轨迹变化。
