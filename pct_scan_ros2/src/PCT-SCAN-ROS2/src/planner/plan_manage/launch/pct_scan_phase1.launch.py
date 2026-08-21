"""Phase 1: interactive PCT global path feeding SCAN's closed-loop stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    asset_root = LaunchConfiguration("asset_root").perform(context)
    tomogram_path = LaunchConfiguration("tomogram_path").perform(context)
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)

    if not tomogram_path:
        tomogram_path = os.path.join(
            asset_root, "source", "scene", "multifloor", "mutifloor_upstream.pickle"
        )
    if not pcd_map_file:
        pcd_map_file = os.path.join(
            asset_root, "source", "scene", "multifloor", "pcd", "collision_map.pcd"
        )

    for description, path in (
        ("PCT tomogram", tomogram_path),
        ("SCAN collision PCD", pcd_map_file),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{description} does not exist: {path}")

    pct_share = get_package_share_directory("pct_planner")
    scan_share = get_package_share_directory("scan_planner")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pct_share, "launch", "planner.launch.py")
            ),
            launch_arguments={
                "scene_name": "Multifloor",
                "tomogram_path": tomogram_path,
                "frame_id": "world",
                "coord_mode": "sim_to_pct_180deg",
                "path_topic": "/pct_path",
                "restrict_endpoints_to_main_components": "true",
                "endpoint_height_tolerance": "0.4",
                "plan_on_startup": LaunchConfiguration("plan_on_startup"),
                "plan_on_marker_release": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_share, "launch", "run.launch.py")
            ),
            launch_arguments={
                "is_real_world": "false",
                "navi_mode": "3",
                "sensor_type": "lidar",
                "controller_mode": "closed_loop",
                "initial_path_topic": "/pct_path",
                "use_gpu": LaunchConfiguration("use_gpu"),
                "use_pcd_map": "true",
                "pcd_map_file": pcd_map_file,
                "map_size_x": "50.0",
                "map_size_y": "50.0",
                "map_size_z": "10.0",
                # PCT path z is ground height. SCAN adds its configured 0.4 m
                # body height once, so the kinematic base starts at z=0.227271.
                "init_x": "-3.503225",
                "init_y": "6.663706",
                "init_z": "0.227271",
                "use_sim_time": "false",
            }.items(),
        ),
    ]

    if _as_bool(LaunchConfiguration("use_rviz").perform(context)):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(scan_share, "rviz", "default.rviz")],
                parameters=[{"use_sim_time": False}],
            )
        )

    return actions


def generate_launch_description():
    default_asset_root = os.environ.get("PCT_SCAN_ASSET_ROOT", "")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "asset_root",
                default_value=default_asset_root,
                description="Root of the repository that owns the multifloor assets",
            ),
            DeclareLaunchArgument(
                "tomogram_path",
                default_value="",
                description="Override path to mutifloor_upstream.pickle",
            ),
            DeclareLaunchArgument(
                "pcd_map_file",
                default_value="",
                description="Override path to collision_map.pcd",
            ),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "plan_on_startup",
                default_value="false",
                description="Plan the default route automatically for headless smoke tests",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
