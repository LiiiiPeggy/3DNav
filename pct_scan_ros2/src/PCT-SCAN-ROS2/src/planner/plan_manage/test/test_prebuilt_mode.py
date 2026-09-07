"""File-contract tests for Phase B (prebuilt-map) real navigation.

No launch is executed: these assert the intended isolation and wiring of the
new Phase B files, in the style of test_launch_pct_scan_real.py.
"""

import os

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))

RUN_LAUNCH = os.path.join(
    ROOT, "src", "planner", "plan_manage", "launch", "run.launch.py")
REAL_SCRIPT = os.path.join(ROOT, "scripts", "launch_pct_scan_real_prebuilt.sh")
PREBUILT_LAUNCH = os.path.join(
    ROOT, "src", "integration", "pct_scan_bridge", "launch",
    "real_prebuilt_scan.launch.py")
REAL_RViz = os.path.join(
    ROOT, "src", "planner", "plan_manage", "launch", "real_prebuilt.rviz")


def test_run_launch_world_frame_is_additive_and_off_by_default():
    with open(RUN_LAUNCH, encoding="utf-8") as f:
        content = f.read()
    # default "" -> no override -> Phase A (map-free) behaviour unchanged.
    assert 'DeclareLaunchArgument("world_frame", default_value="")' in content
    assert 'world_frame_text = LaunchConfiguration("world_frame").perform(context)' in content
    assert 'if world_frame_text:' in content
    assert 'planner_overrides["grid_map.frame_id"] = world_frame_text' in content


def test_prebuilt_script_exists_executable_and_not_hardcoding_tomogram():
    assert os.path.isfile(REAL_SCRIPT)
    assert os.access(REAL_SCRIPT, os.X_OK)
    with open(REAL_SCRIPT, encoding="utf-8") as f:
        content = f.read()
    assert "_pct_scan_env.sh" in content
    assert "real_prebuilt_scan.launch.py" in content
    # shell must NOT bake tomogram_path into the launch command, so user
    # overrides win (the launch file itself defaults to F19000_map.pickle).
    exec_line = content.split("exec ros2 launch", 1)[1]
    assert "tomogram_path" not in exec_line
    assert "/state_estimation" in content and "/registered_scan" in content


def test_prebuilt_launch_wiring():
    with open(PREBUILT_LAUNCH, encoding="utf-8") as f:
        content = f.read()
    # explicit opt-in, Mode 3 closed loop, PCT path as initial path, odin_map.
    assert '"is_real_world": "true"' in content
    assert '"navi_mode": "3"' in content
    assert 'DeclareLaunchArgument("controller_mode", default_value="closed_loop")' in content
    assert '"sensor_type": "lidar"' in content
    assert '"initial_path_topic": "/pct_path"' in content
    assert 'DeclareLaunchArgument("world_frame", default_value="odin_map")' in content
    assert 'DeclareLaunchArgument("start_mode", default_value="1")' in content
    assert '"tomogram_path": LaunchConfiguration("tomogram_path")' in content
    assert "real_prebuilt.rviz" in content


def test_prebuilt_rviz_fixed_frame():
    assert os.path.isfile(REAL_RViz)
    with open(REAL_RViz, encoding="utf-8") as f:
        content = f.read()
    assert "Fixed Frame: odin_map" in content
    assert "Value: /pct_tomogram" in content
    assert "Value: /pct_path" in content
    assert "Value: /grid_map/occupancy" in content
