"""Launch real-robot PREBUILT-map navigation (PCT global + SCAN Mode 3).

Phase B entry point — explicit opt-in, fully isolated from the Phase A
(map-free) launch `scripts/launch_pct_scan_real.sh`.

Prerequisite: Odin SLAM running in RELOCALIZATION mode
(SLAM/3run_relocalization.sh, map F1q9000.bin), publishing
/state_estimation (nav_msgs/Odometry) and /registered_scan (PointCloud2),
both in frame `odin_map` == the F19000_map.pickle coordinate frame.

Layout in this frame (odin_map):
    F19000_map.pickle -> PCT A* -> /pct_path -> SCAN Mode 3 -> /cmd_vel
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
            "PCT_SCAN_ROS2_ROOT is not set. Start this mode through "
            "scripts/launch_pct_scan_real_prebuilt.sh."
        )

    default_tomogram = os.path.join(project_root, "maps", "F19000_map.pickle")
    default_rviz = os.path.join(
        project_root, "src", "planner", "plan_manage", "launch",
        "real_prebuilt.rviz",
    )
    scan_share = get_package_share_directory("scan_planner")

    declarations = [
        DeclareLaunchArgument("tomogram_path", default_value=default_tomogram),
        DeclareLaunchArgument("world_frame", default_value="odin_map"),
        DeclareLaunchArgument("start_mode", default_value="1"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
        DeclareLaunchArgument("planning_horizon", default_value="3.5"),
        DeclareLaunchArgument("max_vel", default_value=""),
        DeclareLaunchArgument("collision_radius", default_value=""),
        DeclareLaunchArgument("plan_on_startup", default_value="false"),
        DeclareLaunchArgument("plan_on_marker_release", default_value="false"),
    ]

    # PCT global planner + Start/Goal + /pct_path. Created directly (not via the
    # shared pct_bridge.launch.py) so Phase B can own frame/start_mode without
    # touching the Phase A / sim launch trees.
    bridge = Node(
        package="pct_scan_bridge",
        executable="bridge_node",
        name="pct_scan_bridge",
        output="screen",
        parameters=[{
            "rsg_root": project_root,
            "scene_name": "custom",
            "tomogram_path": LaunchConfiguration("tomogram_path"),
            "frame_id": LaunchConfiguration("world_frame"),
            "coord_mode": "identity",
            "optimize_path": False,
            "path_topic": "/pct_path",
            "publish_tomogram": True,
            "tomogram_topic": "/pct_tomogram",
            "plan_on_startup": LaunchConfiguration("plan_on_startup"),
            "enable_interactive_markers": True,
            "plan_on_marker_release": LaunchConfiguration(
                "plan_on_marker_release"
            ),
            "snap_search_radius_cells": 15,
            "restrict_endpoints_to_main_components": False,
            "endpoint_height_tolerance": 0.4,
            "centerline_bias_enabled": True,
            "preferred_clearance": 0.5,
            "clearance_cost": 20.0,
            "astar_step_cost_weight": 1.0,
            "tomogram_visual_min_cost": 0.0,
            "tomogram_visual_max_cost": 50.0,
            "use_scene_waypoint_overrides": False,
            # Phase B：默认以当前机器人位姿为 Start（start_mode=1），手动=0。
            "start_mode": LaunchConfiguration("start_mode"),
            "start_pose_odom_topic": "/state_estimation",
        }],
    )

    # SCAN local Mode 3 tracking the PCT global path in closed loop.
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

    note = LogInfo(
        msg=(
            "[real-prebuilt] 前置：Odin 已以重定位模式运行（3run_relocalization.sh, "
            "F1q9000.bin），/state_estimation 帧=odin_map。RViz: Fixed Frame=odin_map；"
            "Start 自动=当前位姿（start_mode=1），拖/设 Goal → /pct_path → SCAN Mode3 → /cmd_vel。"
        )
    )

    return LaunchDescription([*declarations, note, bridge, scan, rviz])
