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
