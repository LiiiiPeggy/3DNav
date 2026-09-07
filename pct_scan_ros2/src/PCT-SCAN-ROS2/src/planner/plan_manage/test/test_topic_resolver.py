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
    with pytest.raises(ValueError, match="lidar"):
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
