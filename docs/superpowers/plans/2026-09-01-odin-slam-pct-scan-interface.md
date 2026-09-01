# PCT-SCAN-ROS2 × Odin SLAM 接口对齐（阶段 A）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `pct_scan_ros2/src/PCT-SCAN-ROS2` 规划器接口从 `/LIO/*` 遗留话题对齐到 Odin SLAM 的 `/registered_scan` + `/state_estimation`，实现无全局地图（map-free local navigation）的实时局部避障（阶段 A）。

**Architecture:** 提取纯函数 `_compute_topics()` 与 `_should_publish_robot_state()` 统一真实/仿真话题与坐标帧决策；`run.launch.py` 新增可覆盖 launch 参数（Odin 默认值）；真实分支不启动 `go2_robot_state_publisher`；新增 `launch_pct_scan_real.sh` 一行拉起。SCAN 核心算法、GridMap、轨迹优化、控制器一律零修改。控制链冻结为 Go2 已验证的 `/cmd_vel`（不做额外抽象层）。

**Tech Stack:** ROS 2 Humble / launch（Python）、PyYAML、pytest（`ament_add_pytest_test`）、bash。

**Spec:** [docs/superpowers/specs/2026-09-01-odin-slam-pct-scan-interface-design.md](../../specs/2026-09-01-odin-slam-pct-scan-interface-design.md)（执行者需同时阅读 spec 与计划）

**重点原则：接口迁移优先，算法零修改；验证真实链路优先，避免过度工程化。**

## Global Constraints

- 真实分支话题映射：`body_pose ← /state_estimation`、`sensor_pose ← /state_estimation`、`cloud ← /registered_scan`。
- 真实分支 `grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false` —— **前提**：`/registered_scan` 点云坐标已在 odom/world 帧（Task 0 验证）。若帧不一致，禁止使用 `cloud_is_world=true`，需 TF 转换节点或点云转换适配器（**当前不实现**）。
- **接口冻结**（Go2 + ZBNav 已验证链路，不增加抽象层）：

  | Interface | Topic | Message |
  |---|---|---|
  | Velocity command | `/cmd_vel` | `geometry_msgs/msg/Twist` |

- **阶段 A 真实分支仅支持 `sensor_type=lidar`**；`is_real_world=true` 搭配 `sensor_type=depth` 必须报错。
- 真实分支**不启动** `go2_robot_state_publisher`（真机狗有自己的 TF/URDF）；仿真默认启动。
- odom 帧 = planner 世界帧，阶段 A **不使用 TF**。
- **不修改**：`scan_replan_fsm`/`planner_manager`、`plan_env`（GridMap 算法）、`path_searching`、`bspline_opt`、`closed_loop_controller`。
- 不假设 Odin QoS 兼容（spec §6），Task 0 实测记录。
- 测试用 `PYTHON_EXECUTABLE /usr/bin/python3`（本机 ROS 2 用系统 python，勿用 conda python）。
- 测试只覆盖轻量纯函数与文件契约，**不测试 launch 执行/Node 结构**。

---

## File Structure

| 文件 | 责任 |
| --- | --- |
| `src/planner/plan_manage/launch/run.launch.py` | **修改**：新增 5 个 launch 参数（`body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`world_frame`、`publish_robot_state`）；提取并接线 `_compute_topics()`、`_should_publish_robot_state()`；真实分支 Odin 默认话题；`go2_robot_state_publisher` 条件化；`cmd_vel` 固定 `/cmd_vel`（无参数化） |
| `src/planner/plan_manage/test/test_run_launch_topics.py` | **新建**：纯函数单测（`_compute_topics`、`_should_publish_robot_state`） |
| `scripts/launch_pct_scan_real.sh` | **新建**：真实启动脚本（复用 `_pct_scan_env.sh`，透传 `"$@"`） |
| `src/planner/plan_manage/test/test_launch_pct_scan_real.py` | **新建**：文件契约测试（存在/可执行/关键参数，不执行 launch） |
| `src/planner/plan_manage/CMakeLists.txt` | **修改**：注册上述 2 个 pytest（仿现有块，`PYTHON_EXECUTABLE /usr/bin/python3`） |

---

## Task 0: Odin 接口验证（前置，改代码前）

**Files:** 无改动。在 T1 启动 Odin 后执行，记录输出；不通过则**暂停开发**，先解决帧/QoS 问题。

- [ ] **Step 1: 启动 Odin 并核对话题**

```bash
cd /home/yu/3DNav/SLAM && ./2run_slam.sh
```

新终端：

```bash
ros2 topic hz /registered_scan
ros2 topic hz /state_estimation
```

Expected：两个话题均有稳定非零频率。

- [ ] **Step 2: 帧一致性检查（关键前提）**

```bash
ros2 topic echo /registered_scan --once
ros2 topic echo /state_estimation --once
```

Expected：`/registered_scan.header.frame_id == /state_estimation.header.frame_id`（应为 `odom` 或 Odin 实际帧名），即 `/registered_scan` 已在 odom/world 帧。
**不满足时**：禁止使用 `cloud_is_world=true`。需 TF 转换节点或点云转换适配器（当前不实现，先停止并汇报）。

- [ ] **Step 3: QoS 记录（不要假设兼容）**

```bash
ros2 topic info /registered_scan -v
ros2 topic info /state_estimation -v
```

Expected：记录两端 publisher QoS（reliability/history/depth）。后续若 planner 收不到数据，按 spec §6 在订阅侧对齐。

- [ ] **Step 4: 记录并提交（只记录，不改代码）**

把 Task 0 结果写进本计划文件末尾的「Task 0 验证记录」小节后提交：

```bash
git add docs/superpowers/plans/2026-09-01-odin-slam-pct-scan-interface.md
git commit -m "docs: 记录 Odin 接口前置验证结果（帧/QoS）"
```

---

## Task 1: `_compute_topics()` + `_should_publish_robot_state()` 纯函数 + 单测

**Files:**
- Create: `src/planner/plan_manage/test/test_run_launch_topics.py`
- Modify: `src/planner/plan_manage/launch/run.launch.py`（新增模块级函数，不改 `_setup`）

**Interfaces:**
- Produces: `_compute_topics(*, is_real, sensor_type, enable_local_sensing, body_pose_topic="", sensor_pose_topic="", cloud_topic="")` → dict（键 `body_pose, sensor_pose, cloud, depth, cloud_is_world, need_extrinsic, intrinsics`），real+depth 抛 `ValueError`。`_should_publish_robot_state(is_real, publish_robot_state)` → bool。Task 2 消费二者。

- [ ] **Step 1: 写失败测试**

创建 `src/planner/plan_manage/test/test_run_launch_topics.py`：

```python
"""Light unit tests for pure topic/frame-resolution logic in run.launch.py."""

import importlib.util
import os

import pytest

RUN_LAUNCH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "launch", "run.launch.py")
)
SPEC = importlib.util.spec_from_file_location("run_launch", RUN_LAUNCH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _topics(**kwargs):
    return MODULE._compute_topics(**kwargs)


# --- _compute_topics: 真实分支 ---

def test_real_branch_uses_odin_topics():
    t = _topics(is_real=True, sensor_type="lidar", enable_local_sensing=True)
    assert t["body_pose"] == "/state_estimation"
    assert t["sensor_pose"] == "/state_estimation"
    assert t["cloud"] == "/registered_scan"
    assert t["cloud_is_world"] is True
    assert t["need_extrinsic"] is False


def test_real_branch_respects_topic_overrides():
    t = _topics(
        is_real=True, sensor_type="lidar", enable_local_sensing=True,
        body_pose_topic="/my/odom", cloud_topic="/my/cloud",
    )
    assert t["body_pose"] == "/my/odom"
    assert t["sensor_pose"] == "/state_estimation"  # 未覆盖项保持默认
    assert t["cloud"] == "/my/cloud"


def test_real_branch_rejects_depth():
    with pytest.raises(ValueError):
        _topics(is_real=True, sensor_type="depth", enable_local_sensing=True)


# --- _compute_topics: 仿真分支（保护共享代码路径不被重构破坏） ---

def test_sim_branch_lidar_uses_quad_topics():
    t = _topics(is_real=False, sensor_type="lidar", enable_local_sensing=True)
    assert t["body_pose"] == "/quad_0/body_pose"
    assert t["sensor_pose"] == "/quad_0/lidar_pose"
    assert t["cloud"] == "/quad_0/cloud"
    assert t["cloud_is_world"] is True
    assert t["need_extrinsic"] is False


def test_sim_branch_depth_uses_camera_pose():
    t = _topics(is_real=False, sensor_type="depth", enable_local_sensing=True)
    assert t["sensor_pose"] == "/quad_0/camera_pose"


def test_sim_branch_without_local_sensing_uses_body_pose():
    t = _topics(is_real=False, sensor_type="lidar", enable_local_sensing=False)
    assert t["sensor_pose"] == "/quad_0/body_pose"


# --- _should_publish_robot_state ---

@pytest.mark.parametrize(
    "is_real,publish_robot_state,expected",
    [
        (True, "", False),    # 真实默认关（真机狗自有 TF）
        (False, "", True),    # 仿真默认开
        (True, "true", True),  # 显式覆盖开
        (False, "false", False),
    ],
)
def test_should_publish_robot_state(is_real, publish_robot_state, expected):
    assert MODULE._should_publish_robot_state(is_real, publish_robot_state) is expected
```

- [ ] **Step 2: 运行测试确认失败**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: FAIL —— `AttributeError: module 'run_launch' has no attribute '_compute_topics'`

- [ ] **Step 3: 实现两个纯函数**

在 `run.launch.py` 中、`_setup` 之前（紧接 `_as_bool` 之后）添加：

```python
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
            "depth": "/camera/aligned_depth_to_color/image_raw",
            "cloud_is_world": True,
            "need_extrinsic": False,
            "intrinsics": {},
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
        "depth": "/quad_0/depth",
        "cloud_is_world": True,
        "need_extrinsic": False,
        "intrinsics": {},
    }


def _should_publish_robot_state(is_real, publish_robot_state):
    """Real branch defaults to off (the real dog publishes its own TF)."""
    if publish_robot_state == "":
        return not is_real
    return _as_bool(publish_robot_state)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: PASS（7 个测试全绿）

- [ ] **Step 5: 提交**

```bash
git add src/planner/plan_manage/launch/run.launch.py src/planner/plan_manage/test/test_run_launch_topics.py
git commit -m "feat: 提取 _compute_topics/_should_publish_robot_state 纯函数并单测（Odin 话题映射，real 仅 lidar）"
```

---

## Task 2: 接线 `_setup` + 新增 launch 参数 + 条件化 robot_state_publisher

**Files:**
- Modify: `src/planner/plan_manage/launch/run.launch.py`

**Interfaces:**
- Consumes: `_compute_topics`、`_should_publish_robot_state`（Task 1 签名）。
- Produces: 新 launch 参数 `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`world_frame`、`publish_robot_state`。`cmd_vel` **不**参数化，固定 `/cmd_vel`（真实）/`/quad_0/cmd_vel`（仿真）。

- [ ] **Step 1: 接线 `_setup`**

**1a. 在 `_setup` 开头（`enable_local_sensing` 之后）读取新参数：**

```python
    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    # world_frame（默认 odom）仅声明，阶段 A 不使用 TF；保留供阶段 B 帧对齐。
```

**1b. 用 `_compute_topics` 替换 `if is_real: ... else: ...` 整块（含 intrinsics），改为：**

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
    depth = topics["depth"]
    cloud_is_world = topics["cloud_is_world"]
    need_extrinsic = topics["need_extrinsic"]
    intrinsics = topics["intrinsics"]
```

**1c. `cmd_vel` 保持固定（不引入 `cmd_vel_topic`）** —— `closed_loop_controller` 的 remap 维持现状：

```python
            remappings=[
                ("body_pose", body_pose),
                ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
            ],
```

**1d. `go2_robot_state_publisher` 条件化** —— 当前无条件 append，改为：

```python
    if _should_publish_robot_state(is_real, publish_robot_state):
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                             " use_gazebo:=false"]
                        )
                    },
                ],
            )
        )
```

- [ ] **Step 2: 新增 launch 参数声明**

在 `generate_launch_description()` 的 `DeclareLaunchArgument` 列表中追加：

```python
            DeclareLaunchArgument("body_pose_topic", default_value=""),
            DeclareLaunchArgument("sensor_pose_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("publish_robot_state", default_value=""),
```

（**不加** `cmd_vel_topic` —— 见 Global Constraints 接口冻结表。）

- [ ] **Step 3: 人工核对参数真实来源（替代 yaml 守卫）**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
grep -R "cloud_is_world" src/
grep -R "need_extrinsic" src/
```

Expected：`grid_map.cloud_is_world` / `grid_map.need_extrinsic` 的唯一真实来源为
`run.launch.py` 中 `planner_overrides`（取自 `_compute_topics`）与 `config/planner.yaml`
默认值（`cloud_is_world: true`、`need_extrinsic: false`、`sensor_type: lidar`，已确认）。
确认无其它地方以 `false/true` 覆盖真实分支。

- [ ] **Step 4: 回归验证**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v
```

Expected：Task 1 的 7 个纯函数测试仍全绿（真实链路逻辑已由 Task 1 覆盖，本任务只做接线）。

- [ ] **Step 5: 提交**

```bash
git add src/planner/plan_manage/launch/run.launch.py
git commit -m "feat: run.launch.py 接线 Odin 真实话题（新增 5 参数，robot_state_publisher 条件化，cmd_vel 固定）"
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

## Task 4: 注册 pytest 到 CMakeLists + colcon 回归

**Files:**
- Modify: `src/planner/plan_manage/CMakeLists.txt`（`endif()` 前的 pytest 块内）

- [ ] **Step 1: 注册新测试**

在 `plan_manage/CMakeLists.txt` 现有 `ament_add_pytest_test(test_reference_path_publisher_unit ...)` 块之后、`add_launch_test(...)` 之前，追加：

```cmake
  ament_add_pytest_test(
    test_run_launch_topics
    test/test_run_launch_topics.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(
    test_launch_pct_scan_real
    test/test_launch_pct_scan_real.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
```

（沿用现有块的 `PYTHON_EXECUTABLE /usr/bin/python3`，避开 conda python。）

- [ ] **Step 2: 重建并跑全套测试**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select scan_planner
colcon test-result --verbose
```

Expected：原有测试（含 `test_planner_startup` 启动冒烟）+ 新增 2 个 pytest 全部 PASS/无失败。若新增项 FAIL，用 `--verbose` 看断言并修复。

- [ ] **Step 3: 提交**

```bash
git add src/planner/plan_manage/CMakeLists.txt
git commit -m "test: 注册 run.launch 纯函数 / real 脚本契约 pytest 到 colcon"
```

---

## Task 5: 真机验收（Go2 + ZBNav 链路）

**Files:** 无代码改动。按序执行，记录每步输出。

- [ ] **Step 1: 启动 Odin 并确认话题**

```bash
cd /home/yu/3DNav/SLAM && ./2run_slam.sh
```

新终端：`ros2 topic hz /registered_scan`、`ros2 topic hz /state_estimation`。
Expected：两话题有稳定频率；若之前 Task 0 未做，补做帧一致性与 QoS 检查。

- [ ] **Step 2: 启动 planner 真机分支，确认 GridMap 更新**

```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
./scripts/launch_pct_scan_real.sh
```

另开终端看 RViz：`ros2 launch scan_planner rviz.launch.py`
Expected：planner 启动无报错；RViz 局部占据地图随 `/registered_scan` 实时更新，`/state_estimation` 位姿变化时滑动窗口跟随。

- [ ] **Step 3: RViz 2D Nav Goal，确认 planning/bspline**

Expected：设置目标点后 `ros2 topic hz /planning/bspline` 有输出，轨迹平滑无跳变。

- [ ] **Step 4: 确认 /cmd_vel 输出（未接狗）**

```bash
ros2 topic echo /cmd_vel
```

Expected：`geometry_msgs/msg/Twist`，目标点设置后非零、随收敛归零。**先确认规划器输出正常，再接狗。**

- [ ] **Step 5: 接入 Go2 控制链**

确认 Step 2–4 全链路正常后，把 `/cmd_vel` 接入 Go2（ZBNav 已验证接口）。空旷场地点动，再逐步验证避障。
规划器问题与控制问题分开排查。

- [ ] **Step 6: 收尾**

验收通过后，更新 `CLAUDE.md`（真机接口、SLAM 工作空间说明），单独提交。阶段 B（PCT 全局 tomogram）另行流程，不在本计划内。

---

## Self-Review

**Spec 覆盖对照：**

| Spec 要求 | 对应任务 |
| --- | --- |
| §4.1 接口映射 / odom 帧约定 | Task 1（`_compute_topics`）+ Task 2（接线） |
| §4.2 launch 参数（5 个话题类 + publish_robot_state；无 cmd_vel_topic） | Task 2 Step 1/2 |
| §4.2 真实分支不启动 robot_state_publisher | Task 2 Step 1d（`_should_publish_robot_state`） |
| §4.3 `launch_pct_scan_real.sh` | Task 3 |
| §4.4 起点/速度沿用默认 | Task 5 实测 |
| §4.5 sensor_pose 外参假设 | Task 1 函数注释 + 测试 |
| §4.6 need_extrinsic 代码核对 | Task 2 Step 3（grep 真实来源） |
| §5 启动前接口验证（帧一致） | **Task 0**（前置，改代码前） |
| §6 QoS 兼容性 | Task 0 Step 3 |
| §8.2 cmd_vel 两阶段验证 | Task 5 Step 4/5 |
| §8.3 Test 0/1/2 | Task 5（空旷→避障→近障碍，依实机条件展开） |
| 接口冻结 `/cmd_vel` | Global Constraints + Task 2 Step 1c（不参数化） |

**删除项（依 ZBNav 收敛）**：launch 结构测试（`_setup` 直调 / LaunchContext / Node 集合）、`cmd_vel_topic` 参数、planner.yaml pytest 守卫（改人工 grep 核对）、launch 执行测试。

**占位符扫描：** 无 TBD/TODO；每个代码步骤含完整代码；Task 0/5 为手动验收，含具体命令与预期输出。

**类型一致性：** `_compute_topics`（Task 1 定义、Task 2 消费）签名与返回键一致；`_should_publish_robot_state` 在 Task 1 定义、Task 2 Step 1d 消费；node 名 `go2_robot_state_publisher` 与 `run.launch.py` 的 `name=` 一致。

**Task 0 验证记录**

（Task 0 完成后在此追加：frame_id、QoS、hz 实测结果。）
