"""Generic RViz-only PCT + SCAN launch for a prepared paper scene."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("pct_scan_bridge")
    scan_share = get_package_share_directory("scan_planner")

    defaults = {
        "scene_name": "plaza",
        "tomogram_path": "",
        "pcd_map_file": "",
        "scene_start_x": "0.0",
        "scene_start_y": "0.0",
        "scene_start_z": "0.0",
        "scene_goal_x": "23.0",
        "scene_goal_y": "10.0",
        "scene_goal_z": "0.0",
        "init_x": "0.0",
        "init_y": "0.0",
        "init_z": "0.4",
        "map_size_x": "65.0",
        "map_size_y": "65.0",
        "map_size_z": "10.0",
        "planning_horizon": "0.8",
        "max_vel": "0.6",
        "snap_search_radius_cells": "12",
        "use_rviz": "true",
        "use_gpu": "false",
        "enable_local_sensing": "false",
        "controller_mode": "open_loop",
        "plan_on_startup": "false",
        "publish_tomogram": "true",
        "enable_interactive_markers": "true",
        "centerline_bias_enabled": "true",
        "preferred_clearance": "0.5",
        "clearance_cost": "20.0",
        "astar_step_cost_weight": "1.0",
        "tomogram_visual_min_cost": "0.0",
        "tomogram_visual_max_cost": "50.0",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "pct_bridge.launch.py")
        ),
        launch_arguments={
            "tomogram_path": LaunchConfiguration("tomogram_path"),
            "rsg_root": "/tmp",
            "scene_name": LaunchConfiguration("scene_name"),
            "coord_mode": "identity",
            "path_topic": "/pct_path",
            "publish_tomogram": LaunchConfiguration("publish_tomogram"),
            "plan_on_startup": LaunchConfiguration("plan_on_startup"),
            "enable_interactive_markers": LaunchConfiguration(
                "enable_interactive_markers"
            ),
            "plan_on_marker_release": "false",
            "snap_search_radius_cells": LaunchConfiguration(
                "snap_search_radius_cells"
            ),
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
            "use_scene_waypoint_overrides": "true",
            "scene_start_x": LaunchConfiguration("scene_start_x"),
            "scene_start_y": LaunchConfiguration("scene_start_y"),
            "scene_start_z": LaunchConfiguration("scene_start_z"),
            "scene_goal_x": LaunchConfiguration("scene_goal_x"),
            "scene_goal_y": LaunchConfiguration("scene_goal_y"),
            "scene_goal_z": LaunchConfiguration("scene_goal_z"),
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
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "max_vel": LaunchConfiguration("max_vel"),
            "collision_radius": "0.12",
            "collision_offset": "0.12",
            "inflation_z_up": "0.05",
            "inflation_z_down": "0.05",
            "use_gpu": LaunchConfiguration("use_gpu"),
            "enable_local_sensing": LaunchConfiguration("enable_local_sensing"),
            "use_pcd_map": "true",
            "pcd_map_file": LaunchConfiguration("pcd_map_file"),
            "map_size_x": LaunchConfiguration("map_size_x"),
            "map_size_y": LaunchConfiguration("map_size_y"),
            "map_size_z": LaunchConfiguration("map_size_z"),
            "init_x": LaunchConfiguration("init_x"),
            "init_y": LaunchConfiguration("init_y"),
            "init_z": LaunchConfiguration("init_z"),
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
    return LaunchDescription([*declarations, bridge, scan, rviz])
