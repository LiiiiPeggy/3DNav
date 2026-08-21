"""Launch the PCT-SCAN-ROS2 global-planner bridge for a selected scene."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("tomogram_path"),
        DeclareLaunchArgument("rsg_root", default_value="/tmp"),
        DeclareLaunchArgument("scene_name", default_value="building"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("coord_mode", default_value="identity"),
        DeclareLaunchArgument("path_topic", default_value="/pct_path"),
        DeclareLaunchArgument("publish_tomogram", default_value="true"),
        DeclareLaunchArgument("tomogram_topic", default_value="/pct_tomogram"),
        DeclareLaunchArgument("plan_on_startup", default_value="false"),
        DeclareLaunchArgument("enable_interactive_markers", default_value="true"),
        DeclareLaunchArgument("plan_on_marker_release", default_value="false"),
        DeclareLaunchArgument("snap_search_radius_cells", default_value="10"),
        DeclareLaunchArgument(
            "restrict_endpoints_to_main_components", default_value="false"
        ),
        DeclareLaunchArgument("endpoint_height_tolerance", default_value="0.4"),
        DeclareLaunchArgument("centerline_bias_enabled", default_value="true"),
        DeclareLaunchArgument("preferred_clearance", default_value="0.5"),
        DeclareLaunchArgument("clearance_cost", default_value="20.0"),
        DeclareLaunchArgument("astar_step_cost_weight", default_value="1.0"),
        DeclareLaunchArgument("tomogram_visual_min_cost", default_value="0.0"),
        DeclareLaunchArgument("tomogram_visual_max_cost", default_value="50.0"),
        DeclareLaunchArgument(
            "use_scene_waypoint_overrides", default_value="false"
        ),
        DeclareLaunchArgument("scene_start_x", default_value="0.0"),
        DeclareLaunchArgument("scene_start_y", default_value="0.0"),
        DeclareLaunchArgument("scene_start_z", default_value="0.0"),
        DeclareLaunchArgument("scene_goal_x", default_value="0.0"),
        DeclareLaunchArgument("scene_goal_y", default_value="0.0"),
        DeclareLaunchArgument("scene_goal_z", default_value="0.0"),
    ]

    node = Node(
        package="pct_scan_bridge",
        executable="bridge_node",
        name="pct_scan_bridge",
        output="screen",
        parameters=[
            {
                "rsg_root": LaunchConfiguration("rsg_root"),
                "scene_name": LaunchConfiguration("scene_name"),
                "tomogram_path": LaunchConfiguration("tomogram_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "coord_mode": LaunchConfiguration("coord_mode"),
                "optimize_path": False,
                "path_topic": LaunchConfiguration("path_topic"),
                "publish_tomogram": LaunchConfiguration("publish_tomogram"),
                "tomogram_topic": LaunchConfiguration("tomogram_topic"),
                "plan_on_startup": LaunchConfiguration("plan_on_startup"),
                "enable_interactive_markers": LaunchConfiguration(
                    "enable_interactive_markers"
                ),
                "plan_on_marker_release": LaunchConfiguration(
                    "plan_on_marker_release"
                ),
                "snap_search_radius_cells": LaunchConfiguration(
                    "snap_search_radius_cells"
                ),
                "restrict_endpoints_to_main_components": LaunchConfiguration(
                    "restrict_endpoints_to_main_components"
                ),
                "endpoint_height_tolerance": LaunchConfiguration(
                    "endpoint_height_tolerance"
                ),
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
                "use_scene_waypoint_overrides": LaunchConfiguration(
                    "use_scene_waypoint_overrides"
                ),
                "scene_start_x": LaunchConfiguration("scene_start_x"),
                "scene_start_y": LaunchConfiguration("scene_start_y"),
                "scene_start_z": LaunchConfiguration("scene_start_z"),
                "scene_goal_x": LaunchConfiguration("scene_goal_x"),
                "scene_goal_y": LaunchConfiguration("scene_goal_y"),
                "scene_goal_z": LaunchConfiguration("scene_goal_z"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
