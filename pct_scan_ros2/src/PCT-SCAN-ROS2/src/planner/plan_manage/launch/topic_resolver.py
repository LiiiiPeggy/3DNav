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
