# Real-Robot Prebuilt-Map 3D Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a prebuilt-map 3D navigation mode on the real robot: RViz shows the F19000 PCT tomogram with interactive Start/Goal markers, the user plans a global PCT route, and SCAN Mode 3 tracks it in closed loop to `/cmd_vel`, while Odin SLAM runs in relocalization mode against a stored map.

**Architecture:** A single real launch (the real analogue of the sim `building_pct_scan.launch.py`) composes three independent pieces that already exist and were each verified separately: `pct_scan_bridge` (PCT tomogram A* + Start/Goal markers → `/pct_path`), SCAN's real `run.launch.py` in Mode 3 (`/initial_path := /pct_path`, closed loop), and RViz. Coordinate alignment is one shared frame `odin_map` (identity to the tomogram); the only frame work is teaching `run.launch.py` to forward its `world_frame` argument into `grid_map.frame_id`, which the code already earmarked for "Phase B". SLAM is **not modified**; it is an external prerequisite that publishes the same two topics as Phase A but in `odin_map`.

**Tech Stack:** ROS 2 Humble, ament_python / ament_cmake, `launch`/`launch_ros` Python, `pct_scan_bridge` (Python bridge node), `scan_planner` (C++ SCAN), rviz2, pytest (file-contract tests, matching the repo's `test_launch_pct_scan_real.py` pattern).

**Spec:** This plan implements the feature request: "Phase A flat nav is tested; add a prebuilt-map 3D navigation mode that visualizes `pct_scan_ros2/src/PCT-SCAN-ROS2/maps/F19000_map.pickle` in RViz2 and lets the user select points to publish. At this time SLAM runs relocalization mode `SLAM/3run_relocalization.sh` (with `F1q9000.bin` placed under the robot's `SLAM/src/odin_ros_driver/map`). Do not modify anything under SLAM."

## Global Constraints

- Do NOT modify any file under `SLAM/` (external Odin SLAM on the robot). Our code only *consumes* its topics.
- Relocalized odometry origin = prebuilt-map origin; `/state_estimation` frame id = `odin_map` (confirmed by user reading SLAM code). Treat tomogram coords as identity with `odin_map`; do not add a transform until an on-robot alignment check proves otherwise.
- Phase A Odin topics remain `/state_estimation` (`nav_msgs/Odometry`) and `/registered_scan` (`sensor_msgs/PointCloud2`) — only their frame changes to `odin_map`.
- Real branch supports `controller_mode=closed_loop` and `sensor_type=lidar` only (enforced by `topic_resolver._compute_topics`).
- Closed-loop controller is planar (x/y/yaw). Real stair climbing emerges from the robot's own locomotion while it tracks the horizontal projection; the plan verifies a same-floor long goal first and treats cross-floor as a follow-up acceptance on hardware.
- Language of user-facing docs/comments: Chinese (matches repo). Code identifiers English.
- Run `./scripts/build_workspace.sh` (full colcon) and `colcon test` after code tasks; Python launch/config changes need no C++ recompile but the package re-install must happen (build_workspace covers it).

## File Structure

- Modify `src/planner/plan_manage/launch/run.launch.py` — forward `world_frame` → `grid_map.frame_id` (the Phase-B wiring its comment already promises).
- Create `src/planner/plan_manage/launch/real_prebuilt.rviz` — RViz config: Fixed Frame `odin_map`, PCT displays (tomogram/global path/Start-Goal markers), SCAN local displays (occupancy/inflate/sliding bounds/goal).
- Create `src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py` — composes `pct_bridge.launch.py` + scan `run.launch.py` (real, Mode 3) + RViz.
- Create `scripts/launch_pct_scan_real_3d.sh` — env + Odin-relocalization preflight + exec of the new launch (mirrors `launch_pct_scan_real.sh`).
- Create `src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py` — file-contract tests (mirror `test_launch_pct_scan_real.py`).
- Create `src/planner/plan_manage/test/test_run_launch_world_frame.py` — file-contract test that `run.launch.py` wires `world_frame` to `grid_map.frame_id`.
- Create `docs/tutorial_real_prebuilt_3d_nav.md` — runbook incl. relocalization prerequisite, alignment smoke, plan & track verification.

---

### Task 1: Teach `run.launch.py` to forward `world_frame` into `grid_map.frame_id`

**Files:**
- Modify: `src/planner/plan_manage/launch/run.launch.py` (read `world_frame`, add conditional override; change the `DeclareLaunchArgument` default to `""`)
- Test: `src/planner/plan_manage/test/test_run_launch_world_frame.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: launch arg `world_frame` (default `""`). When non-empty, `scan_planner_node` receives parameter `grid_map.frame_id=<value>` so SCAN publishes occupancy/bbox in that frame. When empty (unchanged behaviour today) no override is emitted and `planner.yaml`'s `grid_map.frame_id: world` still applies.

- [ ] **Step 1: Write the failing file-contract test**

Create `src/planner/plan_manage/test/test_run_launch_world_frame.py`:

```python
"""File-contract test: run.launch.py must forward world_frame to grid_map.frame_id.

Phase B (prebuilt-map, frame odin_map) needs SCAN's occupancy published in the
real frame. This cannot be executed headlessly, so assert on the source, matching
test_launch_pct_scan_real.py's style.
"""
import os

RUN = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "launch", "run.launch.py"))


def test_declares_world_frame_default_empty():
    with open(RUN, encoding="utf-8") as f:
        content = f.read()
    assert 'DeclareLaunchArgument("world_frame", default_value="")' in content


def test_world_frame_read_from_context():
    with open(RUN, encoding="utf-8") as f:
        content = f.read()
    assert 'LaunchConfiguration("world_frame").perform(context)' in content


def test_world_frame_conditionally_sets_grid_map_frame_id():
    with open(RUN, encoding="utf-8") as f:
        content = f.read()
    assert 'if world_frame_text:' in content
    assert 'planner_overrides["grid_map.frame_id"] = world_frame_text' in content
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/planner/plan_manage/test/test_run_launch_world_frame.py -v`
Expected: FAIL on `test_world_frame_conditionally_sets_grid_map_frame_id` (assertion not found) and on the default-value test.

- [ ] **Step 3: Edit `run.launch.py`**

(a) Near where `body_pose_topic`, `cloud_topic`, `world_frame` are first read (`_setup`, after the `publish_robot_state` read around line 34), add a read:

```python
    world_frame_text = LaunchConfiguration("world_frame").perform(context)
```

(b) Inside `_setup`, right after the `planner_overrides = { ... }` literal is built (after `"grid_map.need_extrinsic": need_extrinsic,` and before the `if max_vel_text:` block), insert:

```python
    # Phase B (prebuilt-map / relocalization): SCAN publishes its local map in
    # this frame. Empty keeps the planner.yaml default (legacy "world").
    if world_frame_text:
        planner_overrides["grid_map.frame_id"] = world_frame_text
```

(c) Change the argument declaration at the bottom (currently `default_value="odom"`) to:

```python
            DeclareLaunchArgument("world_frame", default_value=""),
```

(d) Update the stale comment on that line (was "阶段 A 不进行 TF lookup…") to note it is now consumed as `grid_map.frame_id` when non-empty.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/planner/plan_manage/test/test_run_launch_world_frame.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Sanity-compile the launch module**

Run: `python3 -m py_compile src/planner/plan_manage/launch/run.launch.py`
Expected: exit 0.

- [ ] **Step 6: Commit**

```bash
cd /home/yu/3DNav
git add pct_scan_ros2/src/PCT-SCAN-ROS2/src/planner/plan_manage/launch/run.launch.py \
        pct_scan_ros2/src/PCT-SCAN-ROS2/src/planner/plan_manage/test/test_run_launch_world_frame.py
git commit -m "feat(scan): run.launch 转发 world_frame 到 grid_map.frame_id（Phase B 帧对齐）"
```

---

### Task 2: Add a real-mode RViz config for the prebuilt 3D nav

**Files:**
- Create: `src/planner/plan_manage/launch/real_prebuilt.rviz`
- Test: `src/planner/plan_manage/test/test_real_prebuilt_rviz.py`

**Interfaces:**
- Consumes: nothing.
- Produces: RViz config file at `src/planner/plan_manage/launch/real_prebuilt.rviz` — Fixed Frame `odin_map`; displays: Ground Grid, TF, `PCT Full Cost Tomogram` (`/pct_tomogram`), `PCT Global Path` (`/pct_path`), `PCT Start and Goal` (InteractiveMarkers namespace `/pct_waypoints`), `Occupancy` (`/grid_map/occupancy`), `Inflated Occupancy` (`/grid_map/occupancy_inflate`), `Sliding Map Bounds` (`/grid_map/sliding_map_bbox`), `Goal` (`/goal_point`). Later tasks reference this file by source path.

- [ ] **Step 1: Write the failing file-contract test**

Create `src/planner/plan_manage/test/test_real_prebuilt_rviz.py`:

```python
"""File-contract test for the real prebuilt-nav RViz config."""
import os

RV = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "launch", "real_prebuilt.rviz"))


def test_file_exists():
    assert os.path.isfile(RV)


def test_fixed_frame_is_odin_map():
    with open(RV, encoding="utf-8") as f:
        assert "Fixed Frame: odin_map" in f.read()


def test_pct_and_scan_displays_present():
    with open(RV, encoding="utf-8") as f:
        content = f.read()
    for display in ("PCT Full Cost Tomogram", "PCT Global Path",
                    "PCT Start and Goal", "Name: Occupancy"):
        assert display in content
    assert "Value: /pct_tomogram" in content
    assert "Value: /pct_path" in content
    assert "Value: /grid_map/occupancy" in content
    assert "Interactive Markers Namespace: /pct_waypoints" in content
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/planner/plan_manage/test/test_real_prebuilt_rviz.py -v`
Expected: FAIL (`test_file_exists`, file missing).

- [ ] **Step 3: Create `real_prebuilt.rviz`**

Copy `src/planner/plan_manage/launch/building.rviz` and edit it:
1. Replace `Fixed Frame: world` with `Fixed Frame: odin_map` (single occurrence in the Global Options block).
2. Delete the display blocks whose `Name` is one of: `Go2`, `Global Map`, `Sensor Cloud`, `Robot Path` (they depend on a URDF/`/quad_0` sim sources and spam errors in real mode).
3. Keep: `Ground Grid`, `PCT Global Path`, `PCT Start and Goal`, `PCT Full Cost Tomogram`, `Occupancy`, `Inflated Occupancy`, `Sliding Map Bounds`, `Goal`, `TF`, plus the default tools/views.

Recommended mechanical route (produces a valid YAML the same way the existing `f19000.rviz` was pruned):

```python
# run once from the repo root to write the file
import yaml, os
src = "src/planner/plan_manage/launch/building.rviz"
cfg = yaml.safe_load(open(src, encoding="utf-8"))
vm = cfg["Visualization Manager"]
vm["Global Options"]["Fixed Frame"] = "odin_map"
drop = {"Go2", "Global Map", "Sensor Cloud", "Robot Path"}
vm["Displays"] = [d for d in vm["Displays"]
                  if isinstance(d, dict) and d.get("Name") not in drop]
out = "src/planner/plan_manage/launch/real_prebuilt.rviz"
yaml.safe_dump(cfg, open(out, "w", encoding="utf-8"), sort_keys=False, width=1000)
print("wrote", out, "displays:", [d["Name"] for d in vm["Displays"]])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/planner/plan_manage/test/test_real_prebuilt_rviz.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/yu/3DNav
git add pct_scan_ros2/src/PCT-SCAN-ROS2/src/planner/plan_manage/launch/real_prebuilt.rviz \
        pct_scan_ros2/src/PCT-SCAN-ROS2/src/planner/plan_manage/test/test_real_prebuilt_rviz.py
git commit -m "feat(rviz): 实机预建图导航 RViz 配置（Fixed Frame odin_map）"
```

---

### Task 3: Compose the real prebuilt-map launch (bridge + SCAN Mode 3 + RViz)

**Files:**
- Create: `src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py`
- Test: `src/integration/pct_scan_bridge/test/test_real_prebuilt_scan_launch.py`

**Interfaces:**
- Consumes:
  - `src/integration/pct_scan_bridge/launch/pct_bridge.launch.py` args: `tomogram_path`, `rsg_root`, `scene_name`, `frame_id`, `coord_mode`, `path_topic`, `publish_tomogram`, `enable_interactive_markers`, `plan_on_startup`, `plan_on_marker_release`, `use_scene_waypoint_overrides`, `scene_start_x/y/z`, `scene_goal_x/y/z`.
  - `scan_planner/launch/run.launch.py` args: `is_real_world`, `navi_mode`, `controller_mode`, `sensor_type`, `initial_path_topic`, `world_frame`, `collision_radius`, `planning_horizon`, `max_vel`, `use_rviz`-equivalent handled here.
- Produces: `ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py` with launch args `tomogram_path` (default `$PCT_SCAN_ROS2_ROOT/maps/F19000_map.pickle`), `world_frame` (default `odin_map`), `use_rviz` (default `true`), plus pass-through `controller_mode`/`planning_horizon`/`collision_radius`/`max_vel`.

- [ ] **Step 1: Write the failing file-contract test**

Create `src/integration/pct_scan_bridge/test/test_real_prebuilt_scan_launch.py`:

```python
"""File-contract test for the real prebuilt-map launch (mirrors building_pct_scan)."""
import os

LAUNCH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "launch",
                                      "real_prebuilt_scan.launch.py"))


def test_file_exists():
    assert os.path.isfile(LAUNCH)


def test_real_and_mode3_wiring_present():
    with open(LAUNCH, encoding="utf-8") as f:
        content = f.read()
    assert 'is_real_world": "true"' in content or 'is_real_world="true"' in content
    assert '"navi_mode": "3"' in content or 'navi_mode="3"' in content
    assert '"controller_mode": "closed_loop"' in content
    assert '"sensor_type": "lidar"' in content
    assert '"initial_path_topic": "/pct_path"' in content


def test_odin_map_and_bridge_wiring_present():
    with open(LAUNCH, encoding="utf-8") as f:
        content = f.read()
    assert '"world_frame": "odin_map"' in content
    assert 'pct_bridge.launch.py' in content
    assert 'tomogram_path' in content


def test_rviz_uses_real_prebuilt_config():
    with open(LAUNCH, encoding="utf-8") as f:
        content = f.read()
    assert 'real_prebuilt.rviz' in content
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/integration/pct_scan_bridge/test/test_real_prebuilt_scan_launch.py -v`
Expected: FAIL (`test_file_exists`).

- [ ] **Step 3: Create `real_prebuilt_scan.launch.py`**

Model it directly on `src/integration/pct_scan_bridge/launch/building_pct_scan.launch.py`. Differences: real mode, Mode 3, `odin_map`, no `use_pcd_map`, no `simulator`, RViz uses the new config. Full content:

```python
"""Launch real-robot prebuilt-map (PCT global + SCAN Mode 3) navigation.

Prerequisite: Odin SLAM running in RELOCALIZATION mode
(SLAM/3run_relocalization.sh, map F1q9000.bin) publishing
/state_estimation (nav_msgs/Odometry) and /registered_scan (PointCloud2),
both in frame odin_map (== F19000_map.pickle coordinates).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = os.environ.get("PCT_SCAN_ROS2_ROOT", "")
    if not project_root:
        raise RuntimeError(
            "PCT_SCAN_ROS2_ROOT is not set. Start this demo through "
            "scripts/launch_pct_scan_real_3d.sh."
        )

    default_tomogram = os.path.join(project_root, "maps", "F19000_map.pickle")
    default_rviz = os.path.join(
        project_root, "src", "planner", "plan_manage", "launch",
        "real_prebuilt.rviz")

    bridge_share = get_package_share_directory("pct_scan_bridge")
    scan_share = get_package_share_directory("scan_planner")

    declarations = [
        DeclareLaunchArgument("tomogram_path", default_value=default_tomogram),
        DeclareLaunchArgument("world_frame", default_value="odin_map"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
        DeclareLaunchArgument("planning_horizon", default_value="3.5"),
        DeclareLaunchArgument("max_vel", default_value="0.5"),
        DeclareLaunchArgument("collision_radius", default_value=""),
        DeclareLaunchArgument("plan_on_startup", default_value="false"),
        DeclareLaunchArgument("plan_on_marker_release", default_value="true"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "pct_bridge.launch.py")
        ),
        launch_arguments={
            "tomogram_path": LaunchConfiguration("tomogram_path"),
            "rsg_root": project_root,
            "scene_name": "custom",
            "coord_mode": "identity",
            "frame_id": LaunchConfiguration("world_frame"),
            "path_topic": "/pct_path",
            "publish_tomogram": "true",
            "plan_on_startup": LaunchConfiguration("plan_on_startup"),
            "enable_interactive_markers": "true",
            "plan_on_marker_release": LaunchConfiguration(
                "plan_on_marker_release"),
            "snap_search_radius_cells": "15",
            "restrict_endpoints_to_main_components": "false",
            "endpoint_height_tolerance": "0.4",
        }.items(),
    )

    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "true",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": LaunchConfiguration("controller_mode"),
            "world_frame": LaunchConfiguration("world_frame"),
            "initial_path_topic": "/pct_path",
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "max_vel": LaunchConfiguration("max_vel"),
            "collision_radius": LaunchConfiguration("collision_radius"),
            "use_sim_time": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    reloc_note = LogInfo(
        msg=(
            "[real-prebuilt] 前置：请确认 Odin 已以重定位模式运行 "
            "(SLAM/3run_relocalization.sh, F1q9000.bin)，且 /state_estimation "
            "frame_id=odin_map。RViz 里拖 Start/Goal marker → 右键 Plan route → "
            "/pct_path → SCAN Mode 3 跟踪 → /cmd_vel。"
        )
    )

    return LaunchDescription([*declarations, reloc_note, bridge, scan, rviz])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/integration/pct_scan_bridge/test/test_real_prebuilt_scan_launch.py -v`
Expected: PASS.

- [ ] **Step 5: Sanity-compile the launch module**

Run: `python3 -m py_compile src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py`
Expected: exit 0.

- [ ] **Step 6: Commit**

```bash
cd /home/yu/3DNav
git add pct_scan_ros2/src/PCT-SCAN-ROS2/src/integration/pct_scan_bridge/launch/real_prebuilt_scan.launch.py \
        pct_scan_ros2/src/PCT-SCAN-ROS2/src/integration/pct_scan_bridge/test/test_real_prebuilt_scan_launch.py
git commit -m "feat(bridge): 实机预建图 3D 导航 launch（PCT global + SCAN Mode3 + RViz, frame odin_map）"
```

---

### Task 4: Quick-start script for the real prebuilt 3D nav

**Files:**
- Create: `scripts/launch_pct_scan_real_3d.sh`
- Test: `src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py`

**Interfaces:**
- Consumes: `_pct_scan_env.sh`, `real_prebuilt_scan.launch.py`.
- Produces: executable `./scripts/launch_pct_scan_real_3d.sh [tomogram_path:=…] [max_vel:=…]` that sources the shared env, preflights Odin relocalization topics (`/state_estimation`, `/registered_scan`), prints a usage/next-step block, and `exec`s the launch.

- [ ] **Step 1: Write the failing file-contract test**

Create `src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py` (mirror `test_launch_pct_scan_real.py`):

```python
"""File-contract test for the real prebuilt-map launch script (no execution)."""
import os

SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "scripts", "launch_pct_scan_real_3d.sh"))


def test_script_exists_and_executable():
    assert os.path.isfile(SCRIPT)
    assert os.access(SCRIPT, os.X_OK)


def test_script_launches_real_prebuilt_scan():
    with open(SCRIPT, encoding="utf-8") as f:
        content = f.read()
    assert "_pct_scan_env.sh" in content
    assert "real_prebuilt_scan.launch.py" in content
    assert 'tomogram_path:=' in content


def test_script_preflights_reloc_topics():
    with open(SCRIPT, encoding="utf-8") as f:
        content = f.read()
    assert "/state_estimation" in content
    assert "/registered_scan" in content
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py -v`
Expected: FAIL.

- [ ] **Step 3: Create `scripts/launch_pct_scan_real_3d.sh`**

```bash
#!/usr/bin/env bash
# Launch real-robot prebuilt-map 3D navigation (PCT global + SCAN Mode 3).
# Prerequisite: Odin SLAM in RELOCALIZATION mode
#   (SLAM/3run_relocalization.sh; F1q9000.bin under SLAM/src/odin_ros_driver/map)
#   publishing /state_estimation + /registered_scan in frame odin_map.
# Overrides pass through ("$@"): e.g. max_vel:=0.3, tomogram_path:=/path/x.pickle
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"

echo "[pct-scan-real-3d] 预检 Odin 重定位话题 ..."
for tp in /state_estimation /registered_scan; do
  rate="$(timeout 3 ros2 topic hz "$tp" 2>/dev/null | grep -m1 'average rate' || true)"
  echo "  $tp -> ${rate:-未收到消息（请确认 SLAM/3run_relocalization.sh 已启动）}"
done

echo "[pct-scan-real-3d] RViz：Fixed Frame=odin_map；拖 Start/Goal marker → 右键 Plan route"
echo "[pct-scan-real-3d] 监控：ros2 topic echo /cmd_vel  /  /pct_path  /  /planning/bspline"

exec ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py \
  tomogram_path:="$PCT_SCAN_ROS2_ROOT/maps/F19000_map.pickle" \
  world_frame:=odin_map "$@"
```

Then `chmod +x scripts/launch_pct_scan_real_3d.sh`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/yu/3DNav
git add pct_scan_ros2/src/PCT-SCAN-ROS2/scripts/launch_pct_scan_real_3d.sh \
        pct_scan_ros2/src/PCT-SCAN-ROS2/src/planner/plan_manage/test/test_launch_pct_scan_real_3d.py
git commit -m "feat(scan): 实机预建图 3D 导航启动脚本 launch_pct_scan_real_3d.sh"
```

---

### Task 5: Full build + offline test sweep

**Files:**
- none new.

**Interfaces:**
- Consumes: all files from Tasks 1–4.

- [ ] **Step 1: Full colcon build**

Run from the workspace root:
```bash
cd /home/yu/3DNav/pct_scan_ros2/src/PCT-SCAN-ROS2
./scripts/build_workspace.sh
```
Expected: exit 0, `install/setup.bash` refreshed (run.launch.py/launch files reinstalled).

- [ ] **Step 2: Run the whole workspace test suite**

Run:
```bash
colcon test && colcon test-result --verbose
```
Expected: the new tests `test_run_launch_world_frame`, `test_real_prebuilt_rviz`, `test_real_prebuilt_scan_launch`, `test_launch_pct_scan_real_3d` PASS alongside the existing suite (no regressions in `test_topic_resolver`, `test_launch_pct_scan_real`).

- [ ] **Step 3: Headless launch smoke (no Odin required)**

Run (expect it to fail fast only on missing Odin topics, NOT on arg parsing / missing files):
```bash
source install/setup.bash
export PCT_SCAN_ROS2_ROOT="$PWD"
timeout 20 ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py use_rviz:=false \
  tomogram_path:="$PWD/maps/F19000_map.pickle" 2>&1 | head -40
```
Expected: logs show tomogram load attempt for `F19000_map.pickle`, `grid_map.frame_id` effective frame log, and no "Invalid scene name"/remap-parse errors. Missing Odin topics are fine at this stage.

- [ ] **Step 4: Commit any incidental fixes**

If Step 1–3 surfaced bugs, fix them in the relevant task file, re-run tests, and commit.

---

### Task 6: Runbook + on-robot manual acceptance

**Files:**
- Create: `docs/tutorial_real_prebuilt_3d_nav.md`

**Interfaces:**
- Consumes: Tasks 1–5 artifacts; SLAM relocalization on robot (external).

- [ ] **Step 1: Write the runbook**

Create `docs/tutorial_real_prebuilt_3d_nav.md` (Chinese) containing, at minimum, these sections:

1. **目标与数据流** — tomogram F19000 → PCT A\* → `/pct_path` → SCAN Mode 3 → B-spline → closed_loop → `/cmd_vel`; frames all `odin_map`.
2. **前置（机器狗）**
   - Odin 以重定位模式运行：`SLAM/3run_relocalization.sh`（SLAM 内部，勿改动）；确认 `SLAM/src/odin_ros_driver/map/F1q9000.bin` 在位。
   - 确认话题在 `odin_map` 帧：`ros2 topic echo /state_estimation --once`（看 `header.frame_id`）。
   - 全量构建：`./scripts/build_workspace.sh`（把 Task 1–4 的 launch/rviz 安装进 `install/`）。
3. **启动**
   - 终端 A：`./scripts/launch_pct_scan_real_3d.sh`（自动开 bridge + SCAN + RViz）。
   - 等价手动：`ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py tomogram_path:=$PCT_SCAN_ROS2_ROOT/maps/F19000_map.pickle world_frame:=odin_map`
4. **RViz 操作** — Fixed Frame=`odin_map`；确认 PCT tomogram 点云与真实/里程计重合（对齐冒烟）；把 **Start marker 拖到机器人当前所在格**，**Goal marker 拖到目标**，右键任一 marker → **Plan route**；`/pct_path` 出现后 SCAN 自动跟踪，`/cmd_vel` 输出。
5. **验证步骤（按顺序，每步有通过判据）**
   - V1 帧对齐：机器人静止时，RViz 里 odom/里程计原点应落在 tomogram 对应位置（与建图一致）。若整体平移/旋转偏，**不要改 SLAM**；记录偏差并在启动参数里加静态对齐（未来任务）或手动把 Start 放到机器人真实格。
   - V2 同层长目标：发一个与本层可达、无障碍 3–10 m 目标，观察 `/pct_path` 生成、SCAN 跟踪、`/cmd_vel` 非零、位置收敛到目标（`/state_estimation` 靠近 goal），无急停。
   - V3 贴墙/窄道：选一个贴近障碍的目标，确认局部避障不把机器人推出走廊（对比 Phase A 已验证行为）。
   - V4 跨层（若 F19000 有真实上层/楼梯）：把 Goal 放上层，确认 PCT 走 gateway 出跨层 `/pct_path`；SCAN 平面跟踪 XY 投影、机器人自身爬升（RL）。**注意** closed_loop 是平面控制器，跨层先小步验证。
6. **风险与排查** — depth 空 remap bug 已修；`grid_map.frame_id` 未生效时检查 `world_frame` 参数是否透传；占用地 frame 若不是 odin_map 则 RViz 里不重合，检查 scan_planner_node 参数 `grid_map.frame_id`。

- [ ] **Step 2: Runbook sanity review**

Re-read the runbook against Task 3's launch args and the RViz config display names to make sure every command/topic/frame matches a real artifact. Fix any drift.

- [ ] **Step 3: Commit**

```bash
cd /home/yu/3DNav
git add pct_scan_ros2/src/PCT-SCAN-ROS2/docs/tutorial_real_prebuilt_3d_nav.md
git commit -m "docs: 实机预建图 3D 导航 runbook（Odin 重定位前置 + RViz + 验证）"
```

- [ ] **Step 4: On-robot manual acceptance (no autotest; run by user)**

Execute runbook V1→V4 on the robot with a human in the loop and an e-stop. Record results back into `docs/tutorial_real_prebuilt_3d_nav.md` under a "实测记录" section. This step is complete only when V1–V4 all pass on hardware.

---

## Self-Review

**Spec coverage:**
- "预建图 3D 导航模式 / 全链路驱动" → Task 3 (+4 script) compose bridge + SCAN Mode 3 + closed loop to `/cmd_vel`.
- "RViz2 可视化 F19000_map.pickle 并可选点" → Task 2 RViz config shows the tomogram + Start/Goal markers; Task 3/6 wire it; interaction = existing markers + Plan route (user's choice).
- "SLAM 启动重定位模式 SLAM/3run_relocalization.sh + F1q9000.bin" → external prerequisite; documented in Task 6 runbook; preflight in Task 4; SLAM untouched (Global Constraint).
- "frame_id odin_map / origin = prebuilt map" → Task 1 wires `world_frame`→`grid_map.frame_id`; Task 3 defaults `world_frame=odin_map` and bridge `frame_id=odin_map`; runbook V1 verifies alignment.
- "手动设起点 / 沿用 Start/Goal 交互 marker" → Task 3 enables markers + plan_on_marker_release; runbook V-step instructs dragging Start to robot cell.

**Placeholder scan:** all code/commands concrete; no TBD steps. Manual robot acceptance is explicitly the one non-automatable step and carries concrete pass criteria.

**Type consistency:** launch arg names (`world_frame`, `tomogram_path`, `navi_mode`, `initial_path_topic`, `controller_mode`, `sensor_type`, `is_real_world`) match existing `run.launch.py`/`pct_bridge.launch.py` declarations; RViz display names/topics match `building.rviz`/planner pubs (`/pct_tomogram`, `/pct_path`, `/pct_waypoints`, `/grid_map/occupancy`); test-file assertions match the exact strings emitted by Task code.
