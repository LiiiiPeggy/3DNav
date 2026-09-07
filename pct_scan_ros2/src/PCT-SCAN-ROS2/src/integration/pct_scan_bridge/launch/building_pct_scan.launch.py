"""Launch the Building PCT + SCAN demo without Gazebo."""

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
            "scripts/launch_pct_scan_building.sh."
        )

    building_root = os.path.join(project_root, "assets", "building")
    default_tomogram = os.path.join(
        building_root, "tomogram", "building2_9_ros2.pickle"
    )
    default_pcd = os.path.join(building_root, "pcd", "building2_9.pcd")

    bridge_share = get_package_share_directory("pct_scan_bridge")
    scan_share = get_package_share_directory("scan_planner")

    declarations = [
        DeclareLaunchArgument("tomogram_path", default_value=default_tomogram),
        DeclareLaunchArgument("pcd_map_file", default_value=default_pcd),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gpu", default_value="false"),
        DeclareLaunchArgument("enable_local_sensing", default_value="false"),
        DeclareLaunchArgument("controller_mode", default_value="open_loop"),
        DeclareLaunchArgument("plan_on_startup", default_value="false"),
        DeclareLaunchArgument("publish_tomogram", default_value="true"),
        DeclareLaunchArgument("enable_interactive_markers", default_value="true"),
        DeclareLaunchArgument("centerline_bias_enabled", default_value="true"),
        DeclareLaunchArgument("preferred_clearance", default_value="0.5"),
        DeclareLaunchArgument("clearance_cost", default_value="20.0"),
        DeclareLaunchArgument("astar_step_cost_weight", default_value="1.0"),
        DeclareLaunchArgument("tomogram_visual_min_cost", default_value="0.0"),
        DeclareLaunchArgument("tomogram_visual_max_cost", default_value="50.0"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "pct_bridge.launch.py")
        ),
        launch_arguments={
            "tomogram_path": LaunchConfiguration("tomogram_path"),
            "rsg_root": building_root,
            "scene_name": "building",
            "coord_mode": "identity",
            "path_topic": "/pct_path",
            "publish_tomogram": LaunchConfiguration("publish_tomogram"),
            "plan_on_startup": LaunchConfiguration("plan_on_startup"),
            "enable_interactive_markers": LaunchConfiguration(
                "enable_interactive_markers"
            ),
            "plan_on_marker_release": "false",
            "snap_search_radius_cells": "12",
            "restrict_endpoints_to_main_components": "false",
            "endpoint_height_tolerance": "0.4",
            "centerline_bias_enabled": LaunchConfiguration(
                "centerline_bias_enabled"
            ),
            "preferred_clearance": LaunchConfiguration("preferred_clearance"),
            "clearance_cost": LaunchConfiguration("clearance_cost"),
            "astar_step_cost_weight": LaunchConfiguration(
                "astar_step_cost_weight"
            ),
            "tomogram_visual_min_cost": LaunchConfiguration(
                "tomogram_visual_min_cost"
            ),
            "tomogram_visual_max_cost": LaunchConfiguration(
                "tomogram_visual_max_cost"
            ),
        }.items(),
    )

    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "false",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": LaunchConfiguration("controller_mode"),
            "initial_path_topic": "/pct_path",
            "reference_path_min_distance": "0.15",
            "reference_path_simplify_tolerance": "0.1",
            "planning_horizon": "0.8",
            "max_vel": "0.5",
            # PCT marks the narrow stair corridor traversable.  Use a compact
            # visual-demo footprint so SCAN does not inflate its risers shut.
            "collision_radius": "0.12",
            "collision_offset": "0.12",
            "inflation_z_up": "0.05",
            "inflation_z_down": "0.05",
            "use_gpu": LaunchConfiguration("use_gpu"),
            "enable_local_sensing": LaunchConfiguration("enable_local_sensing"),
            "use_pcd_map": "true",
            "pcd_map_file": LaunchConfiguration("pcd_map_file"),
            "map_size_x": "40.0",
            "map_size_y": "30.0",
            "map_size_z": "22.0",
            # PCT paths describe floor height. SCAN body poses are 0.4 m above it.
            "init_x": "5.034396",
            "init_y": "4.966283",
            "init_z": "0.4",
            "use_sim_time": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(scan_share, "rviz", "building.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    sensing_warning = LogInfo(
        condition=IfCondition(LaunchConfiguration("enable_local_sensing")),
        msg=(
            "[PCT_SCAN_ROS2] enable_local_sensing=true is experimental in "
            "Building: dense stair PCD points can trigger SCAN collision "
            "inflation and EMERGENCY_STOP. Use false for the RViz-only video."
        ),
    )

    return LaunchDescription(
        [*declarations, sensing_warning, bridge, scan, rviz]
    )
