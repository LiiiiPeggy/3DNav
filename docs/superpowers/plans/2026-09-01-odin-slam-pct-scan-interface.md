# PCT-SCAN-ROS2 × Odin SLAM 接口对齐（阶段 A）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `pct_scan_ros2/src/PCT-SCAN-ROS2` 规划器的真机接口从 `/LIO/*` 遗留话题对齐到 Odin SLAM 的 `/registered_scan` + `/state_estimation`，实现无全局地图的实时局部避障（阶段 A）。

**Architecture:** 提取纯函数 `_compute_topics()` 统一真实/仿真话题与坐标帧决策；`run.launch.py` 新增可覆盖 launch 参数（Odin 默认值）；真实分支不启动 `robot_state_publisher`；新增 `launch_pct_scan_real.sh` 一行拉起。SCAN 核心算法、GridMap、轨迹优化一律不改。

**Tech Stack:** ROS 2 Humble / launch（Python）、PyYAML、pytest（`ament_add_pytest_test`）、bash。

**Spec:** [docs/superpowers/specs/2026-09-01-odin-slam-pct-scan-interface-design.md](../../specs/2026-09-01-odin-slam-pct-scan-interface-design.md)（计划从 spec 论证，执行者需同时阅读 spec 与计划）

## Global Constraints

从 spec 提取的项目级约束，每个任务的隐含要求，逐字照抄：

- 真实分支话题映射：`body_pose ← /state_estimation`、`sensor_pose ← /state_estimation`、`cloud ← /registered_scan`。
- 真实分支 `grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false`（前提：`/registered_scan` 已在 odom/world 帧，见 spec §5 启动前验证）。
- **阶段 A 真实分支仅支持 `sensor_type=lidar`**；`is_real_world=true` 搭配 `sensor_type=depth` 不在支持范围。
- 真实分支**不启动** `robot_state_publisher`（节点名 `go2_robot_state_publisher`；真机狗有自己的 TF/URDF）。
- 控制链：`closed_loop_controller → /cmd_vel`（geometry_msgs/Twist）。
- odom 帧 = planner 世界帧，阶段 A **不使用 TF**。
- **不修改**：`scan_replan_fsm`/`planner_manager`、`plan_env`（GridMap 算法）、`path_searching`、`bspline_opt`。只允许加 launch 参数与接线。
- 不假设 Odin QoS 兼容（spec §6）；帧不一致时禁止直接 `cloud_is_world=true`（spec §5）。
- 测试用 `PYTHON_EXECUTABLE /usr/bin/python3`（本机 ROS 2 用系统 python，勿用 conda python）。

---

## File Structure

| 文件 | 责任 |
| --- | --- |
| `src/planner/plan_manage/launch/run.launch.py` | **修改**：新增 6 个 launch 参数；提取并接线 `_compute_topics()`；真实分支 Odin 默认话题；`go2_robot_state_publisher` 条件化；`cmd_vel_topic` 参数化 |
| `src/planner/plan_manage/test/test_run_launch_topics.py` | **新建**：`_compute_topics` 纯函数单测 + `_setup` 结构单测（launch 参数声明、真实/仿真分支节点集） |
| `src/planner/plan_manage/config/planner.yaml` | **验证**：grid_map 参数已正确（`cloud_is_world: true`、`need_extrinsic: false`、`sensor_type: lidar`），不改值 |
| `src/planner/plan_manage/test/test_planner_yaml_config.py` | **新建**：守卫上述 yaml 不变的 pytest |
| `scripts/launch_pct_scan_real.sh` | **新建**：真实启动脚本（复用 `_pct_scan_env.sh`，透传 `"$@"`） |
| `src/planner/plan_manage/test/test_launch_pct_scan_real.py` | **新建**：脚本存在/可执行/关键参数内容契约测试 |
| `src/planner/plan_manage/CMakeLists.txt` | **修改**：注册 3 个新 pytest（仿现有 `ament_add_pytest_test` 块，`PYTHON_EXECUTABLE /usr/bin/python3`） |

---

## Task 1: `_compute_topics()` 纯函数 + 单测

**Files:**
- Create: `src/planner/plan_manage/test/test_run_launch_topics.py`
- Modify: `src/planner/plan_manage/launch/run.launch.py`（新增模块级函数，不改 `_setup`）

**Interfaces:**
- Produces: `_compute_topics(*, is_real, sensor_type, enable_local_sensing, body_pose_topic="", sensor_pose_topic="", cloud_topic="")` → dict，键：`body_pose, sensor_pose, cloud, depth, cloud_is_world, need_extrinsic, intrinsics`。Task 2 在 `_setup` 中消费。

- [ ] **Step 1: 写失败测试**

创建 `src/planner/plan_manage/test/test_run_launch_topics.py`：

```python
"""Unit tests for real/simulation sensor-topic resolution in run.launch.py."""

import importlib.util
import os

RUN_LAUNCH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "launch", "run.launch.py")
)
SPEC = importlib.util.spec_from_file_location("run_launch", RUN_LAUNCH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _topics(**kwargs):
    return MODULE._compute_topics(**kwargs)


def test_real_branch_uses_odin_topics():
    t = _topics(is_real=True, sensor_type="lidar", enable_local_sensing=True)
    assert t["body_pose"] == "/state_estimation"
    assert t["sensor_pose"] == "/state_estimation"
    assert t["cloud"] == "/registered_scan"
    assert t["cloud_is_world"] is True
    assert t["need_extrinsic"] is False
    assert t["intrinsics"] == {}


def test_real_branch_respects_topic_overrides():
    t = _topics(
        is_real=True, sensor_type="lidar", enable_local_sensing=True,
        body_pose_topic="/my/odom", cloud_topic="/my/cloud",
    )
    assert t["body_pose"] == "/my/odom"
    assert t["sensor_pose"] == "/state_estimation"  # 未覆盖项保持默认
    assert t["cloud"] == "/my/cloud"


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
```

- [ ] **Step 2: 运行测试确认失败**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: FAIL —— `AttributeError: module 'run_launch' has no attribute '_compute_topics'`

- [ ] **Step 3: 实现 `_compute_topics`**

在 `run.launch.py` 中、`_setup` 之前（紧接 `_as_bool` 定义之后）添加：

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
    Phase A treats as the planner's world frame.
    """
    if is_real:
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
```

- [ ] **Step 4: 运行测试确认通过**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: PASS（5 个测试全绿）

- [ ] **Step 5: 提交**

```bash
git add src/planner/plan_manage/launch/run.launch.py src/planner/plan_manage/test/test_run_launch_topics.py
git commit -m "feat: 提取 _compute_topics 纯函数并单测（Odin 真机话题映射）"
```

---

## Task 2: 接线 `_setup` + 新增 launch 参数 + 条件化 robot_state_publisher

**Files:**
- Modify: `src/planner/plan_manage/launch/run.launch.py`
- Modify: `src/planner/plan_manage/test/test_run_launch_topics.py`（追加结构测试）

**Interfaces:**
- Consumes: `_compute_topics`（Task 1 的签名）。
- Produces: 新的 launch 参数 `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`cmd_vel_topic`、`world_frame`、`publish_robot_state`。

- [ ] **Step 1: 写失败测试（追加到 test_run_launch_topics.py 末尾）**

```python
from launch import LaunchContext

# _setup 实际执行（.perform）的全部 LaunchConfiguration 及其默认值。
# 缺任一键会 raise，测试即强制清单完整。
ALL_DEFAULTS = {
    "is_real_world": "false",
    "navi_mode": "1",
    "sensor_type": "lidar",
    "controller_mode": "closed_loop",
    "keypoints_file": "",
    "reference_path_file": "",
    "initial_path_topic": "/initial_path",
    "reference_path_min_distance": "0.5",
    "reference_path_simplify_tolerance": "0.0",
    "planning_horizon": "3.5",
    "max_vel": "",
    "max_acc": "",
    "collision_radius": "",
    "collision_offset": "",
    "inflation_z_up": "",
    "inflation_z_down": "",
    "enable_local_sensing": "true",
    "init_x": "",
    "init_y": "",
    "init_z": "",
    "use_sim_time": "false",
    "body_pose_topic": "",
    "sensor_pose_topic": "",
    "cloud_topic": "",
    "cmd_vel_topic": "",
    "publish_robot_state": "",
}


def _render(value, ctx):
    if isinstance(value, (list, tuple)):
        return "".join(_render(v, ctx) for v in value)
    return value.perform(ctx) if hasattr(value, "perform") else str(value)


def _setup_nodes(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(ALL_DEFAULTS)
    ctx.launch_configurations.update(overrides)
    actions = MODULE._setup(ctx)
    return {_render(a.name, ctx) for a in actions if hasattr(a, "name")}


def test_launch_declares_real_topic_args():
    from launch.actions import DeclareLaunchArgument

    ld = MODULE.generate_launch_description()
    declared = {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert {
        "body_pose_topic", "sensor_pose_topic", "cloud_topic",
        "cmd_vel_topic", "world_frame", "publish_robot_state",
    } <= declared


def test_setup_real_branch_node_set():
    nodes = _setup_nodes(is_real_world="true", controller_mode="closed_loop")
    assert "scan_planner_node" in nodes
    assert "closed_loop_controller" in nodes
    assert "go2_robot_state_publisher" not in nodes   # 真机不发布狗模型 TF
    assert "go2_kinematic_sim" not in nodes            # 仿真专用
    assert "go2_gait_publisher" not in nodes           # 仿真专用
    assert "open_loop_controller" not in nodes


def test_setup_real_branch_explicit_robot_state_override():
    nodes = _setup_nodes(
        is_real_world="true", controller_mode="closed_loop",
        publish_robot_state="true",
    )
    assert "go2_robot_state_publisher" in nodes


def test_setup_sim_branch_has_sim_nodes():
    nodes = _setup_nodes(is_real_world="false", controller_mode="closed_loop")
    assert "go2_robot_state_publisher" in nodes
    assert "go2_kinematic_sim" in nodes
    assert "go2_gait_publisher" in nodes
```

- [ ] **Step 2: 运行测试确认失败**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: FAIL —— 新参数未声明 / `_setup` 仍走 `/LIO/*`（`_setup_nodes` 断言 node 名集合不符）

- [ ] **Step 3: 实现**

**3a. 在 `_setup` 开头（`enable_local_sensing` 之后）读取新参数：**

```python
    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    # world_frame（默认 odom）仅声明，阶段 A 不使用 TF；保留供阶段 B 帧对齐。
```

**3b. 用 `_compute_topics` 替换 `if is_real: ... else: ...` 整块（含 intrinsics），改为：**

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

**3c. `cmd_vel_topic` 参数化** —— `closed_loop_controller` 的 remap 改为：

```python
            remappings=[
                ("body_pose", body_pose),
                ("cmd_vel", cmd_vel_topic or ("/cmd_vel" if is_real else "/quad_0/cmd_vel")),
            ],
```

**3d. `go2_robot_state_publisher` 条件化** —— 当前它无条件 append，改为：

```python
    include_robot_state = (not is_real) if publish_robot_state == "" else _as_bool(publish_robot_state)
    if include_robot_state:
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

**3e. 在 `generate_launch_description()` 的 DeclareLaunchArgument 列表中追加：**

```python
            DeclareLaunchArgument("body_pose_topic", default_value=""),
            DeclareLaunchArgument("sensor_pose_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("cmd_vel_topic", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("publish_robot_state", default_value=""),
```

- [ ] **Step 4: 运行测试确认通过**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_run_launch_topics.py -v`
Expected: PASS（Task 1 的 5 个 + 本任务 5 个共 10 个全绿）
注意：若 `ctx.launch_configurations` 报缺键，把缺失键补进 `ALL_DEFAULTS`（以 `_setup` 实际 `.perform()` 的为准）。

- [ ] **Step 5: 提交**

```bash
git add src/planner/plan_manage/launch/run.launch.py src/planner/plan_manage/test/test_run_launch_topics.py
git commit -m "feat: run.launch.py 接线 Odin 真机话题，新增可覆盖 launch 参数并条件化 robot_state_publisher"
```

---

## Task 3: planner.yaml grid_map 守卫测试

**Files:**
- Test: `src/planner/plan_manage/test/test_planner_yaml_config.py`
- Verify: `src/planner/plan_manage/config/planner.yaml`（已含正确值，**不改值**；如被误改则本测试拦截）

**Interfaces:**
- Produces: 守卫不变量 `grid_map.cloud_is_world=true`、`grid_map.need_extrinsic=false`、`grid_map.sensor_type=lidar`。

- [ ] **Step 1: 写测试**

创建 `src/planner/plan_manage/test/test_planner_yaml_config.py`：

```python
"""Guard the real-world grid_map defaults expected by the Odin interface."""

import os

import yaml


def test_planner_yaml_real_grid_map_defaults():
    path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    grid_map = data["grid_map"]
    assert grid_map["sensor_type"] == "lidar"
    assert grid_map["cloud_is_world"] is True
    assert grid_map["need_extrinsic"] is False
```

- [ ] **Step 2: 运行测试确认通过（现状即满足）**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest src/planner/plan_manage/test/test_planner_yaml_config.py -v`
Expected: PASS（现有 yaml 已含正确值）。若 FAIL，说明 planner.yaml 被改动过，按 spec §4.1 恢复 `cloud_is_world: true`、`need_extrinsic: false`。

- [ ] **Step 3: 提交**

```bash
git add src/planner/plan_manage/test/test_planner_yaml_config.py
git commit -m "test: planner.yaml grid_map 真机默认值守卫测试"
```

---

## Task 4: `scripts/launch_pct_scan_real.sh` + 契约测试

**Files:**
- Create: `scripts/launch_pct_scan_real.sh`
- Test: `src/planner/plan_manage/test/test_launch_pct_scan_real.py`

**Interfaces:**
- Consumes: `scripts/_pct_scan_env.sh`（加载 ROS + install + `.deps`）。
- Produces: 可执行脚本，等价于 `ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=1 controller_mode:=closed_loop sensor_type:=lidar "$@"`。

- [ ] **Step 1: 写失败测试**

创建 `src/planner/plan_manage/test/test_launch_pct_scan_real.py`：

```python
"""Contract test for the real-machine launch script."""

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


def test_script_launches_real_branch_with_odin_args():
    with open(SCRIPT, "r", encoding="utf-8") as f:
        content = f.read()
    assert "_pct_scan_env.sh" in content
    assert "is_real_world:=true" in content
    assert "controller_mode:=closed_loop" in content
    assert "navi_mode:=1" in content
    assert "sensor_type:=lidar" in content
    assert 'run.launch.py' in content
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
# 可覆盖：cmd_vel_topic:=/dog/cmd_vel、collision_radius:=0.12 等（透传 "$@"）。
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

## Task 5: 注册 pytest 到 CMakeLists + colcon 测试

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
    test_planner_yaml_config
    test/test_planner_yaml_config.py
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
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select scan_planner
colcon test-result --verbose
```

Expected: 原有测试 + 新增 3 个 pytest 全部 PASS/无失败。若 `colcon test-result` 报新增项 FAIL，用 `--verbose` 看具体断言并修复。

- [ ] **Step 3: 提交**

```bash
git add src/planner/plan_manage/CMakeLists.txt
git commit -m "test: 注册 run.launch / planner.yaml / real 脚本 pytest 到 colcon"
```

---

## Task 6: 真机接口验证与验收（手动，spec §4.6/§5/§6/§8）

**Files:** 无代码改动。逐项执行并记录输出。

- [ ] **Step 1: need_extrinsic 代码路径核对（spec §4.6）**

```bash
cd pct_scan_ros2/src/PCT-SCAN-ROS2
grep -R "need_extrinsic" src/
```

Expected: 找到 `plan_env/src/grid_map.cpp` 中 `if (mp_.need_extrinsic_)` 的 3 处分支（约 L801/L850/L908）。人工确认：`need_extrinsic=false` 时 `sensorPoseCallback`/`cloudCallback` 跳过 `pose_r * extrinsic`，`cloud_is_world=true` 时点按世界帧直接用 —— **无外参叠加、无双重变换**。

- [ ] **Step 2: 启动 Odin 并核对话题（spec §5/§6）**

T1 终端：

```bash
cd SLAM && ./2run_slam.sh
```

新终端核对：

```bash
ros2 topic hz /registered_scan          # 应有稳定频率（非 0）
ros2 topic echo /state_estimation --once
ros2 topic echo /registered_scan --once
ros2 topic info /registered_scan -v     # 记下 QoS（Expected: 与规划器订阅匹配）
ros2 topic info /state_estimation -v
```

Expected：两话题均有数据；**`/registered_scan.header.frame_id == /state_estimation.header.frame_id`**（应同为 `odom` 或 Odin 实际帧名）。若不一致 → **禁止用 `cloud_is_world=true`**，先加静态 TF 或点云转换节点再继续。QoS 不匹配时按 spec §6 在 planner 订阅侧调整。

- [ ] **Step 3: 启动 planner 真机分支**

T2 终端（确保 T1 的 Odin 已就绪）：

```bash
cd pct_scan_ros2/src/PCT-SCAN-ROS2
./scripts/launch_pct_scan_real.sh
```

另开终端看 RViz：`ros2 launch scan_planner rviz.launch.py`

Expected：planner 启动无报错；RViz 中局部占据地图随 `/registered_scan` 实时更新；`ros2 topic echo /state_estimation` 位姿变化时滑动窗口跟随。

- [ ] **Step 4: cmd_vel 两阶段验证（spec §8.2）**

第一阶段（**不连狗**）：

```bash
ros2 topic echo /cmd_vel
```

Expected：`geometry_msgs/msg/Twist`；在 RViz 设 2D Nav Goal 后 `planning/bspline` 发布且 `/cmd_vel` 非零、随目标收敛。**此阶段确认规划器输出正常。**

第二阶段：确认第一阶段全链路正常后，才把 `/cmd_vel` 接入机器狗控制桥。规划器问题与控制问题分开排查。

- [ ] **Step 5: 验收测试（spec §8.3）**

- **Test 0 空旷点到点**：开阔场地，RViz 设目标点，验证 `/state_estimation → grid_map → planning/bspline → cmd_vel` 平滑趋近、无跳变。
- **Test 1 静态障碍绕行**：放静态障碍，验证 `/registered_scan → grid_map → 局部避障`（滑动窗口内 A* 绕行、不碰撞）。
- **Test 2 近距离障碍**：靠障碍行进，验证 `scan_min_range`（Odin adapter 0.2 m）/ `collision_radius`/`collision_offset`（狗紧凑足迹 0.12）表现，无危险贴近。

- [ ] **Step 6: 记录结果并收尾**

验收通过后：
1. 按仓库约定更新 `CLAUDE.md`（真机接口、SLAM 工作空间说明）—— 单独提交。
2. 阶段 B（PCT 全局 tomogram）另行 brainstorming → spec → 实现，不在此计划内。

---

## Self-Review

**Spec 覆盖对照：**

| Spec 要求 | 对应任务 |
| --- | --- |
| §4.1 接口映射 / odom 帧约定 | Task 1（`_compute_topics`）+ Task 2（接线） |
| §4.2 launch 参数（5 个话题 + world_frame + publish_robot_state） | Task 2 Step 3a/3e |
| §4.2 真实分支不启动 robot_state_publisher | Task 2 Step 3d + 测试 |
| §4.3 `launch_pct_scan_real.sh` | Task 4 |
| §4.4 起点/速度参数沿用默认 | Task 2（不改 yaml 速度值）+ Task 6 实测 |
| §4.5 sensor_pose 外参假设 | Task 1 函数注释 + 测试断言 sensor_pose=body_pose 同源 |
| §4.6 need_extrinsic 代码检查 | Task 6 Step 1 |
| §5 启动前接口验证（帧一致） | Task 6 Step 2 |
| §6 QoS 兼容性 | Task 6 Step 2 |
| §8.2 cmd_vel 两阶段验证 | Task 6 Step 4 |
| §8.3 Test 0/1/2 | Task 6 Step 5 |
| §9 Expected modification files（run.launch.py / planner.yaml / real 脚本；核心不改） | Task 1/2/3/4，Global Constraints 圈定不改范围 |
| 阶段 B 不实现 | 计划范围明确排除，Task 6 Step 6 收尾 |

**占位符扫描：** 无 TBD/TODO；每个代码步骤含完整代码；Task 6 为手动验收，含具体命令与预期输出。

**类型一致性：** `_compute_topics` 在 Task 1 定义、Task 2 消费，签名一致（关键字参数 `is_real/sensor_type/enable_local_sensing/body_pose_topic/sensor_pose_topic/cloud_topic`，返回 dict 的 7 个键在 Task 1 测试与 Task 2 Step 3b 中逐键一致）。node 名断言统一用 `go2_robot_state_publisher`（与 `run.launch.py` 的 `name=` 一致）。
