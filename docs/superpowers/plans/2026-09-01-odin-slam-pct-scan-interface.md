# PCT-SCAN-ROS2 × Odin SLAM 接口对齐（阶段 A）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `pct_scan_ros2/src/PCT-SCAN-ROS2` 从原有 LIO 接口适配到 Odin SLAM：输入 `/registered_scan` + `/state_estimation`，输出 `/cmd_vel`，完成实际四足机器人平台部署前的 map-free local navigation（无全局地图局部导航）接口闭环验证。

**Architecture:** 纯函数独立到无 ROS 依赖的 `launch/topic_resolver.py`（`_compute_topics` + `_should_publish_robot_state`）；`run.launch.py` 只声明参数、调用 resolver、创建 Node；真实分支 Odin 默认话题；`robot_state_publisher` 条件化；速度接口统一冻结为 ROS 标准 `/cmd_vel`。SCAN 核心算法、GridMap、轨迹优化、控制器零修改。

**Tech Stack:** ROS 2 Humble / launch（Python）、pytest（`ament_add_pytest_test`）、bash。

**Spec:** [docs/superpowers/specs/2026-09-01-odin-slam-pct-scan-interface-design.md](../../specs/2026-09-01-odin-slam-pct-scan-interface-design.md)（执行者需同时阅读；本计划按已有实机导航系统经验收敛，接口冻结 `/cmd_vel` 覆盖 spec §9 中 `cmd_vel_topic` 的早期提法）

**重点原则：接口迁移优先，算法零修改；真实链路优先，避免过度工程化。不引入额外抽象层。不引入机器人模型依赖。**

**最终架构（冻结）：**

```text
Odin SLAM
  ├─ /registered_scan   (sensor_msgs/msg/PointCloud2, odom帧)
  └─ /state_estimation  (nav_msgs/msg/Odometry, odom帧)
          ↓
    PCT-SCAN-ROS2 (scan_planner_node + closed_loop_controller)
          ↓
        /cmd_vel (geometry_msgs/msg/Twist)
          ↓
    Robot control interface（具体机器人控制层不属于阶段 A）
```

## Global Constraints

- 真实分支话题映射：`body_pose ← /state_estimation`、`sensor_pose ← /state_estimation`、`cloud ← /registered_scan`。
- 真实分支 `grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false` —— **前提**：`/registered_scan` 点云坐标系已与 planner 世界帧（odom）一致（Task 0 验证）。若帧不一致，需确认 TF 或坐标转换关系，禁止直接 `cloud_is_world=true`；阶段 A **不实现转换节点**。
- **接口冻结**：参考已有实机导航系统经验，采用 ROS 标准速度接口 `geometry_msgs/msg/Twist`、topic `/cmd_vel`，**与机器人型号无关**。统一不区分真实/仿真：

  | Interface | Topic | Message |
  |---|---|---|
  | Velocity command | `/cmd_vel` | `geometry_msgs/msg/Twist` |

- **阶段 A 真实分支仅支持 `sensor_type=lidar`**；`is_real_world=true` 搭配 `sensor_type=depth` 必须抛 `ValueError`。depth 仅 `sensor_type=depth` 时被订阅，阶段 A 不支持。
- **`robot_state_publisher` 条件化**：真实分支**不启动**（阶段 A 只验证 Odin SLAM → planner → `/cmd_vel`，不依赖 URDF / robot_description / 机器人模型 / TF 机器人可视化）；仿真分支保持原仓库行为。
- 仿真分支沿用旧仓库 `/quad_0/*` 话题，**仅用于向后兼容，不属于阶段 A 验证范围，不代表目标机器人接口**。
- `odom` = planner 世界帧；`world_frame` 仅预留，阶段 A 不执行 TF lookup、URDF 加载、机器人模型变换。
- `_compute_topics` 返回**仅 5 个键**：`body_pose, sensor_pose, cloud, cloud_is_world, need_extrinsic`（depth/intrinsics 留给阶段 B，阶段 A 置空）。
- `topic_resolver.py` **无 ROS import、无 LaunchContext、无 Node 创建**，可独立 pytest。
- **不修改**：`scan_replan_fsm`/`planner_manager`、`plan_env`、`path_searching`、`bspline_opt`、`closed_loop_controller`、仿真机器人模型来源。
- 不假设 Odin QoS 兼容（spec §6），Task 0 实测记录。
- 测试用 `PYTHON_EXECUTABLE /usr/bin/python3`（避开 conda python）。
- 只测轻量纯函数与文件契约，**不测 launch 执行 / _setup / Node 结构**；不加 URDF、robot_description、RViz 模型、TF 机器人模型测试。

---

## File Structure

| 文件 | 责任 |
| --- | --- |
| `src/planner/plan_manage/launch/topic_resolver.py` | **新建**：纯函数 `_compute_topics` + `_should_publish_robot_state`（无 ROS import） |
| `src/planner/plan_manage/launch/run.launch.py` | **修改**：`sys.path` 导入 resolver；新增 5 个 launch 参数（`body_pose_topic`/`sensor_pose_topic`/`cloud_topic`/`world_frame`/`publish_robot_state`）；真实分支 Odin 默认话题；`robot_state_publisher` 条件化；`cmd_vel` 统一 `/cmd_vel` |
| `src/planner/plan_manage/test/test_topic_resolver.py` | **新建**：纯函数单测（real lidar / real depth 报错 / should_publish） |
| `scripts/launch_pct_scan_real.sh` | **新建**：真实启动脚本（复用 `_pct_scan_env.sh`，透传 `"$@"`） |
| `src/planner/plan_manage/test/test_launch_pct_scan_real.py` | **新建**：文件契约测试（存在/可执行/关键参数，不执行 launch） |
| `src/planner/plan_manage/CMakeLists.txt` | **修改**：注册上述 2 个 pytest（先 grep 确认 `ament_add_pytest_test` 模式；`launch/` 目录整目录安装已自动带上 topic_resolver.py） |

---

## Task 0: Odin 接口验证（前置，改代码前）

**Files:** 无改动。在 T1 启动 Odin 后执行，记录输出；不通过则**暂停开发**。

- [ ] **Step 1: 启动 Odin**

```bash
cd /home/yu/3DNav/SLAM && ./2run_slam.sh
```

- [ ] **Step 2: 话题频率与类型**

```bash
ros2 topic hz /registered_scan
ros2 topic hz /state_estimation
ros2 topic type /registered_scan      # Expected: sensor_msgs/msg/PointCloud2
ros2 topic type /state_estimation     # Expected: nav_msgs/msg/Odometry
```

Expected：两话题有稳定非零频率，类型正确。

- [ ] **Step 3: 坐标系一致性（关键前提）**

```bash
ros2 topic echo /registered_scan --once
ros2 topic echo /state_estimation --once
```

确认：**`/registered_scan` 点云坐标系已与 planner 世界帧（odom）一致**。优先检查 `header.frame_id` 是否一致；若不一致，需确认 TF 或坐标转换关系。
**不满足时**：禁止 `cloud_is_world=true`，需 TF/点云转换节点（阶段 A 不实现，先停止并汇报）。

- [ ] **Step 4: QoS 记录（不要假设兼容）**

```bash
ros2 topic info /registered_scan -v
ros2 topic info /state_estimation -v
```

Expected：记录两端 publisher QoS（reliability/history/depth）。后续 planner 收不到数据时按此对齐订阅侧。

**提示**：planner 以 `SensorDataQoS`（best_effort/volatile）订阅 `body_pose`（见 scan_replan_fsm.cpp）；若 Odin 以 reliable QoS 发布 `/state_estimation`，planner 会静默收不到数据——无数据时先核对 QoS，再排查其它。

- [ ] **Step 5: 记录结果并提交（只记录，不改代码）**

把结果追加到本文件末尾「Task 0 验证记录」后：

```bash
git add docs/superpowers/plans/2026-09-01-odin-slam-pct-scan-interface.md
git commit -m "docs: 记录 Odin 接口前置验证结果（类型/帧/QoS）"
```

---

## Task 1: `launch/topic_resolver.py` 纯函数模块 + 单测

**Files:**
- Create: `src/planner/plan_manage/launch/topic_resolver.py`
- Create: `src/planner/plan_manage/test/test_topic_resolver.py`

**Interfaces:**
- Produces: `_compute_topics(*, is_real, sensor_type, enable_local_sensing, body_pose_topic="", sensor_pose_topic="", cloud_topic="")` → dict（5 键：`body_pose, sensor_pose, cloud, cloud_is_world, need_extrinsic`）；real+depth 抛 `ValueError`。`_should_publish_robot_state(is_real, publish_robot_state)` → bool。Task 2 通过 `from topic_resolver import _compute_topics, _should_publish_robot_state` 消费。

- [ ] **Step 1: 写失败测试**

创建 `src/planner/plan_manage/test/test_topic_resolver.py`（纯 Python，无 ROS import）：

```python
"""Unit tests for launch/topic_resolver.py (pure functions, no ROS)."""

import importlib.util
import os

import pytest

RESOLVER = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "launch", "topic_resolver.py")
)
SPEC = importlib.util.spec_from_file_location("topic_resolver", RESOLVER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_real_branch_uses_odin_topics():
    t = MODULE._compute_topics(is_real=True, sensor_type="lidar", enable_local_sensing=True)
    assert t == {
        "body_pose": "/state_estimation",
        "sensor_pose": "/state_estimation",
        "cloud": "/registered_scan",
        "cloud_is_world": True,
        "need_extrinsic": False,
    }


def test_real_branch_rejects_depth():
    with pytest.raises(ValueError):
        MODULE._compute_topics(is_real=True, sensor_type="depth", enable_local_sensing=True)


@pytest.mark.parametrize(
    "is_real,publish_robot_state,expected",
    [
        (True, "", False),     # 真实默认关（阶段 A 不需要机器人模型显示）
        (False, "", True),     # 仿真默认开
        (True, "true", True),  # 显式覆盖开
        (False, "false", False),
    ],
)
def test_should_publish_robot_state(is_real, publish_robot_state, expected):
    assert MODULE._should_publish_robot_state(is_real, publish_robot_state) is expected
```

- [ ] **Step 2: 运行测试确认失败**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_topic_resolver.py -v`
Expected: FAIL —— `FileNotFoundError` / `ImportError: No module named 'topic_resolver'`（模块尚不存在）

- [ ] **Step 3: 实现 topic_resolver.py**

创建 `src/planner/plan_manage/launch/topic_resolver.py`：

```python
"""Pure topic/frame-resolution helpers for run.launch.py.

This module deliberately has NO ROS imports so it can be unit-tested
independently of the launch environment.
"""


def _compute_topics(
    *,
    is_real,
    sensor_type,
    enable_local_sensing,
    body_pose_topic="",
    sensor_pose_topic="",
    cloud_topic="",
):
    """Resolve sensor remappings and grid-map frame mode.

    Real branch uses Odin topics directly (cloud_is_world=true, no extrinsic):
    /registered_scan and /state_estimation both live in the odom frame, which
    Phase A treats as the planner's world frame (see Task 0 frame check).

    Simulation branch keeps the legacy repo's /quad_0/* topics ONLY for
    backward compatibility; it is NOT Phase A's verification scope and does
    NOT represent the target robot interface.

    Returns dict with keys: body_pose, sensor_pose, cloud, cloud_is_world,
    need_extrinsic. depth/intrinsics are reserved for Phase B.
    """
    if is_real:
        if sensor_type != "lidar":
            raise ValueError(
                "Phase A real branch supports sensor_type='lidar' only "
                f"(got '{sensor_type}')"
            )
        return {
            "body_pose": body_pose_topic or "/state_estimation",
            "sensor_pose": sensor_pose_topic or "/state_estimation",
            "cloud": cloud_topic or "/registered_scan",
            "cloud_is_world": True,
            "need_extrinsic": False,
        }
    sensor_pose = sensor_pose_topic or (
        "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
    )
    if not enable_local_sensing and not sensor_pose_topic:
        sensor_pose = "/quad_0/body_pose"
    return {
        "body_pose": "/quad_0/body_pose",
        "sensor_pose": sensor_pose,
        "cloud": cloud_topic or "/quad_0/cloud",
        "cloud_is_world": True,
        "need_extrinsic": False,
    }


def _should_publish_robot_state(is_real, publish_robot_state):
    """Real branch defaults to off (Phase A needs no robot model/URDF)."""
    if publish_robot_state == "":
        return not is_real
    return publish_robot_state.strip().lower() in ("1", "true", "yes", "on")
```

- [ ] **Step 4: 运行测试确认通过**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_topic_resolver.py -v`
Expected: PASS（3 个测试全绿，parametrize 展开 4 个用例）

- [ ] **Step 5: 提交**

```bash
git add src/planner/plan_manage/launch/topic_resolver.py src/planner/plan_manage/test/test_topic_resolver.py
git commit -m "feat: 新增 launch/topic_resolver.py 纯函数模块（Odin 话题映射，real 仅 lidar）"
```

---

## Task 2: `run.launch.py` 接线

**Files:**
- Modify: `src/planner/plan_manage/launch/run.launch.py`

**Interfaces:**
- Consumes: `topic_resolver` 的 `_compute_topics` / `_should_publish_robot_state`（Task 1）。
- Produces: 新 launch 参数 `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`world_frame`、`publish_robot_state`。速度接口统一 `/cmd_vel`（**无** `cmd_vel_topic`）。

- [ ] **Step 1: 导入 resolver**

在 `run.launch.py` 文件头部（现有 import 块之后）添加：

```python
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from topic_resolver import _compute_topics, _should_publish_robot_state
```

（`launch/` 目录随 `install(DIRECTORY launch ...)` 整目录安装，symlink-install 下源码/安装侧均能找到 `topic_resolver.py`。）

- [ ] **Step 2: 读取新参数**

在 `_setup` 开头（`enable_local_sensing` 之后）添加：

```python
    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    # world_frame（默认 odom）仅声明，阶段 A 不做 TF lookup / URDF 加载；保留供阶段 B 帧对齐。
```

- [ ] **Step 3: 用 `_compute_topics` 替换 `if is_real: ... else: ...` 整块**

原块（含 depth/intrinsics 赋值）替换为：

```python
    topics = _compute_topics(
        is_real=is_real,
        sensor_type=sensor_type,
        enable_local_sensing=enable_local_sensing,
        body_pose_topic=body_pose_topic,
        sensor_pose_topic=sensor_pose_topic,
        cloud_topic=cloud_topic,
    )
    body_pose = topics["body_pose"]
    sensor_pose = topics["sensor_pose"]
    cloud = topics["cloud"]
    cloud_is_world = topics["cloud_is_world"]
    need_extrinsic = topics["need_extrinsic"]
    # Phase A lidar-only deployment.
    # Depth and intrinsics are reserved for Phase B.
    depth = ""
    intrinsics = {}
```

（depth 置空仅保留 `scan_planner_node` 的 depth remap 字段位；`grid_map` 仅在 `sensor_type=depth` 时创建 depth 订阅（见 `plan_env/src/grid_map.cpp`），阶段 A 用 lidar，该字段不被消费，空值安全。）

- [ ] **Step 4: `cmd_vel` 统一为 `/cmd_vel`（接口冻结）**

`closed_loop_controller` 的 remap：

```python
            remappings=[
                ("body_pose", body_pose),
                ("cmd_vel", "/cmd_vel"),
            ],
```

仿真分支的速度执行节点（现有仿真节点，保持原逻辑；其 body_pose 沿用旧仿真话题）的 remap 同步改为消费统一速度话题：

```python
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/cmd_vel"),
                    ],
```

（不再区分 `"/cmd_vel" if is_real else "/quad_0/cmd_vel"`。）

- [ ] **Step 5: `robot_state_publisher` 条件化**

将当前无条件 `actions.append(Node(package="robot_state_publisher", ...))` 整段包进条件分支，节点 `name=` 改为 `robot_state_publisher`，**节点其余参数（含 robot_description）原样保留、不删减**：

```python
    # 真实分支默认不启动 robot_state_publisher（阶段 A 只验证
    # Odin SLAM → planner → /cmd_vel，不依赖 URDF / robot_description /
    # 机器人模型 / TF 机器人可视化）。
    # 仿真分支保持原仓库行为：节点参数块（robot_description 等）原样保留，
    # 本任务不修改仿真机器人模型来源。
    if _should_publish_robot_state(is_real, publish_robot_state):
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[common],  # 现有 robot_description 参数块原样保留
            )
        )
```

（若原代码该节点带 `robot_description` 等额外参数，将其合并进 `parameters` 列表，不要删减。）

- [ ] **Step 6: 新增 launch 参数声明**

在 `generate_launch_description()` 的 `DeclareLaunchArgument` 列表中追加：

```python
            DeclareLaunchArgument("body_pose_topic", default_value=""),
            DeclareLaunchArgument("sensor_pose_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("publish_robot_state", default_value=""),
```

（**不加** `cmd_vel_topic`，无机器人专属参数。）

- [ ] **Step 7: 人工核对参数真实来源**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
grep -R "cloud_is_world" src/
grep -R "need_extrinsic" src/
```

Expected：`grid_map.cloud_is_world` / `grid_map.need_extrinsic` 唯一真实来源为
`run.launch.py` 的 `planner_overrides`（取自 `_compute_topics`）与 `config/planner.yaml`
默认值（`cloud_is_world: true`、`need_extrinsic: false`、`sensor_type: lidar`，已确认）。无其它覆盖。

- [ ] **Step 8: 回归验证**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest src/planner/plan_manage/test/test_topic_resolver.py -v
```

Expected：Task 1 的测试仍全绿（真实链路逻辑已由纯函数测试覆盖，本任务只接线）。

- [ ] **Step 9: 提交**

```bash
git add src/planner/plan_manage/launch/run.launch.py
git commit -m "feat: run.launch.py 接线 Odin 话题（新增 5 参数，rsp 条件化，cmd_vel 统一 /cmd_vel）"
```

---

## Task 3: `scripts/launch_pct_scan_real.sh` + 文件契约测试

**Files:**
- Create: `scripts/launch_pct_scan_real.sh`
- Test: `src/planner/plan_manage/test/test_launch_pct_scan_real.py`（只测文件契约，不执行 launch）

**Interfaces:**
- Consumes: `scripts/_pct_scan_env.sh`（加载 ROS + install + `.deps`）。
- Produces: 可执行脚本，等价于 `ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=1 controller_mode:=closed_loop sensor_type:=lidar "$@"`。

- [ ] **Step 1: 写失败测试**

创建 `src/planner/plan_manage/test/test_launch_pct_scan_real.py`：

```python
"""File-contract test for the real-machine launch script (no launch execution)."""

import os

# test/ → plan_manage/ → planner/ → src/ → PCT-SCAN-ROS2/scripts/
SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..",
        "scripts", "launch_pct_scan_real.sh",
    )
)


def test_script_exists_and_executable():
    assert os.path.isfile(SCRIPT)
    assert os.access(SCRIPT, os.X_OK)


def test_script_contains_fixed_launch_args():
    with open(SCRIPT, "r", encoding="utf-8") as f:
        content = f.read()
    assert "_pct_scan_env.sh" in content
    assert "is_real_world:=true" in content
    assert "navi_mode:=1" in content
    assert "controller_mode:=closed_loop" in content
    assert "sensor_type:=lidar" in content
    assert "run.launch.py" in content
```

- [ ] **Step 2: 运行测试确认失败**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_launch_pct_scan_real.py -v`
Expected: FAIL —— 脚本不存在（`assert os.path.isfile(SCRIPT)`）

- [ ] **Step 3: 创建脚本**

创建 `scripts/launch_pct_scan_real.sh`（仿 `scripts/launch_pct_scan_building.sh` 结构）：

```bash
#!/usr/bin/env bash
# Launch Odin real-robot local navigation (Phase A, map-free local navigation).
# Prerequisite: Odin SLAM already running via SLAM/2run_slam.sh
#   (publishes /registered_scan and /state_estimation in the odom frame).
# RViz 另开终端：ros2 launch scan_planner rviz.launch.py
# 可覆盖（透传 "$@"）：如 collision_radius:=0.12
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  controller_mode:=closed_loop \
  sensor_type:=lidar \
  "$@"
```

然后 `chmod +x scripts/launch_pct_scan_real.sh`。

- [ ] **Step 4: 运行测试确认通过**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_launch_pct_scan_real.py -v`
Expected: PASS（2 个测试全绿）

- [ ] **Step 5: 提交**

```bash
git add scripts/launch_pct_scan_real.sh src/planner/plan_manage/test/test_launch_pct_scan_real.py
git commit -m "feat: 新增 launch_pct_scan_real.sh 真实启动脚本（Odin 局部导航）"
```

---

## Task 4: 注册 pytest + colcon 回归

**Files:**
- Modify: `src/planner/plan_manage/CMakeLists.txt`（`endif()` 前的 pytest 块内）

- [ ] **Step 1: 先确认测试基建已存在**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
grep -n "ament_cmake_pytest" src/planner/plan_manage/CMakeLists.txt
```

Expected：命中 `find_package(ament_cmake_pytest REQUIRED)` 与现有 `ament_add_pytest_test(...)` 块（已存在，仿照注册）。若缺失则需先加 `find_package(ament_cmake_pytest REQUIRED)`。

- [ ] **Step 2: 注册新测试**

在现有 `ament_add_pytest_test(test_reference_path_publisher_unit ...)` 块之后、`add_launch_test(...)` 之前追加：

```cmake
  ament_add_pytest_test(
    test_topic_resolver
    test/test_topic_resolver.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(
    test_launch_pct_scan_real
    test/test_launch_pct_scan_real.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
```

（沿用现有块的 `PYTHON_EXECUTABLE /usr/bin/python3`，避开 conda python。`launch/` 目录已整目录安装，无需额外 install topic_resolver.py。）

- [ ] **Step 3: 重建并跑全套测试**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select scan_planner
colcon test-result --verbose
```

Expected：原有测试（含 `test_planner_startup` 启动冒烟）+ 新增 2 个 pytest 全部 PASS/无失败。若新增项 FAIL，用 `--verbose` 看断言并修复。

- [ ] **Step 4: 提交**

```bash
git add src/planner/plan_manage/CMakeLists.txt
git commit -m "test: 注册 topic_resolver / real 脚本契约 pytest 到 colcon"
```

---

## Task 5: 真机验收（机器人控制接口）

**Files:** 无代码改动。按序执行，记录每步输出。

> **阶段 A RViz 验收目标**：只检查 `odom/world` frame、`/registered_scan` 点云、local grid map、planning trajectory、planner 状态。**不要求**显示机器人模型、加载 URDF、显示 robot_description。

- [ ] **Step 1: Odin 启动**

```bash
cd /home/yu/3DNav/SLAM && ./2run_slam.sh
```

确认 `/registered_scan`、`/state_estimation` 有稳定频率（若 Task 0 未做，补做类型/帧/QoS 检查）。

- [ ] **Step 2: planner 启动 + 节点检查 + GridMap 更新**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
./scripts/launch_pct_scan_real.sh
```

另开终端：

```bash
ros2 node list          # Expected: 含 scan_planner_node、closed_loop_controller
```

另开终端看 RViz：`ros2 launch scan_planner rviz.launch.py`
Expected：planner 启动无报错；RViz 显示 `odom/world` 帧下的 `/registered_scan` 点云与 local grid map 实时更新，`/state_estimation` 位姿变化时滑动窗口跟随。**不显示机器人模型。**

- [ ] **Step 3: RViz 2D Nav Goal**

设置目标点后，确认轨迹输出 topic 出现数据。

- [ ] **Step 4: 定位并核对轨迹输出 topic**

先发现实际轨迹 topic（不同 fork 名称可能有差异，不硬编码）：

```bash
ros2 topic list | grep -E "bspline|traj|trajectory"
```

再对该 topic 测频率：

```bash
ros2 topic hz <actual_topic>
```

Expected：目标点设置后有稳定输出，轨迹平滑无跳变。

- [ ] **Step 5: /cmd_vel 输出（未接机器人）**

```bash
ros2 topic echo /cmd_vel
```

Expected：`geometry_msgs/msg/Twist`，目标点设置后非零、随收敛归零。**先确认规划器输出正常，再接机器人控制层。**

- [ ] **Step 6: 接入实际机器人控制层**

确认 Step 2–5 全链路正常后，把 `/cmd_vel` 接入实际机器人控制层。空旷场地点动，再逐步验证避障（Test 0 空旷点到点 / Test 1 静态障碍绕行 / Test 2 近距离障碍）。机器人控制层与规划器问题分开排查。

- [ ] **Step 7: 收尾**

验收通过后，更新 `CLAUDE.md`（真机接口、SLAM 工作空间说明），单独提交。阶段 B（PCT 全局 tomogram）另行流程，不在本计划内。

---

## Self-Review

**Spec 覆盖对照：**

| Spec 要求 | 对应任务 |
| --- | --- |
| §4.1 接口映射 / odom 帧约定 | Task 1（`_compute_topics`）+ Task 2（接线） |
| §4.2 launch 参数（5 个话题类 + publish_robot_state；无 cmd_vel_topic） | Task 2 Step 2/6 |
| §4.2 真实分支不启动 robot_state_publisher | Task 2 Step 5（`_should_publish_robot_state`） |
| §4.3 `launch_pct_scan_real.sh` | Task 3 |
| §4.4 起点/速度沿用默认 | Task 5 实测 |
| §4.5 sensor_pose 外参假设 | Task 1 函数注释 |
| §4.6 need_extrinsic 代码核对 | Task 2 Step 7（grep 真实来源） |
| §5 启动前接口验证（帧一致） | **Task 0**（前置，坐标系一致性表述） |
| §6 QoS 兼容性 | Task 0 Step 4 |
| §8.2 cmd_vel 两阶段验证 | Task 5 Step 5/6 |
| §8.3 Test 0/1/2 | Task 5 Step 6 |
| 接口冻结 `/cmd_vel`（ROS 标准接口） | Global Constraints + Task 2 Step 4 |

**最终收敛（本版变更）**：
- Task 2 Step 5 不再出现机器人模型文件/`robot_description` 构造细节，仅说明"现有 rsp 节点参数原样保留，包进条件分支"。
- 仿真分支 `/quad_0/*` 明确标注为旧仓库向后兼容、非阶段 A 验证范围、不代表目标机器人接口。
- depth/intrinsics 置空（`depth=""`、`intrinsics={}`）并注释保留给阶段 B；无任何具体 depth topic 泄漏。
- 帧检查表述改为"坐标系与 planner 世界帧一致"，不绝对要求 frame_id 字符串相同。
- Task 5 Step 4 用 `ros2 topic list | grep` 先定位实际轨迹 topic，不硬编码 `/planning/bspline`。
- `/cmd_vel` 冻结理由保持"ROS 标准速度接口、与机器人型号无关"。

**删除项**：launch/_setup/LaunchContext/Node 测试、`cmd_vel_topic` 参数、planner.yaml pytest 守卫（改人工 grep）、sim 话题细节测试、depth/intrinsics 具体值、机器人模型/URDF/TF/模型文件相关引用。

**占位符扫描：** 无 TBD/TODO；每个代码步骤含完整代码；Task 0/5 为手动验收，含具体命令与预期输出。Task 2 Step 5 的 `<现有节点参数原样保留>` 为对执行者"照抄现有 robot_state_publisher 节点参数、不删减"的明确指令，非占位。

**类型一致性：** `_compute_topics` 返回 5 键在 Task 1 测试与 Task 2 Step 3 逐键一致；`_should_publish_robot_state` 在 Task 1 定义、Task 2 Step 5 消费；node 名 `robot_state_publisher` 与 `run.launch.py` 的 `name=` 一致；`topic_resolver.py` 无 ROS import，测试可独立运行。

**Task 0 验证记录**

（Task 0 完成后在此追加：topic type、frame_id、QoS、hz 实测结果。）
