"""Bring up FAST-LIO and SCAN-Planner for a physical robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _vector(context, prefix):
    return [_float(context, f"{prefix}_{axis}") for axis in ("x", "y", "z")]


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    scan_launch = os.path.join(scan_share, "launch", "run.launch.py")

    start_livox_driver = _as_bool(
        LaunchConfiguration("start_livox_driver").perform(context)
    )
    start_fastlio = _as_bool(LaunchConfiguration("start_fastlio").perform(context))
    start_scanplanner = _as_bool(
        LaunchConfiguration("start_scanplanner").perform(context)
    )
    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    start_mobile_3d = _as_bool(
        LaunchConfiguration("start_mobile_3d").perform(context)
    )
    enable_control = _as_bool(LaunchConfiguration("enable_control").perform(context))
    world_frame = LaunchConfiguration("world_frame").perform(context)
    fastlio_imu_frame = LaunchConfiguration("fastlio_imu_frame").perform(context)
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic").perform(context)
    registered_cloud_topic = LaunchConfiguration("registered_cloud_topic").perform(
        context
    )
    body_pose_topic = "/scan_planner/body_pose"
    sensor_pose_topic = "/scan_planner/lidar_pose"
    ready_topic = "/scan_planner/fastlio_inputs_ready"

    actions = []
    if start_livox_driver:
        livox_share = get_package_share_directory("livox_ros_driver2")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        livox_share,
                        "launch_ROS2",
                        "msg_MID360s_launch.py",
                    )
                )
            )
        )
    if start_mobile_3d:
        nav2_web_share = get_package_share_directory("nav2_web")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_web_share, "launch", "scanplanner_3d.launch.py")
                ),
                launch_arguments={
                    "http_port": LaunchConfiguration("mobile_http_port"),
                    "ws_port": LaunchConfiguration("mobile_ws_port"),
                    "fixed_frame": world_frame,
                    "base_frame": LaunchConfiguration("body_frame"),
                    "point_limit": LaunchConfiguration("mobile_point_limit"),
                    "cloud_rate": LaunchConfiguration("mobile_cloud_rate"),
                }.items(),
            )
        )
    if start_fastlio:
        fastlio_share = get_package_share_directory("fast_lio")
        fastlio_launch = os.path.join(fastlio_share, "launch", "mapping.launch.py")
        config_path = LaunchConfiguration("fastlio_config_path").perform(context)
        if not config_path:
            config_path = os.path.join(fastlio_share, "config")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "use_sim_time": "false",
                    "config_path": config_path,
                    "config_file": LaunchConfiguration("fastlio_config_file"),
                    "world_frame": world_frame,
                    "imu_frame": fastlio_imu_frame,
                    "rviz": "true" if start_rviz else "false",
                }.items(),
            )
        )
    elif start_rviz:
        fastlio_share = get_package_share_directory("fast_lio")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(fastlio_share, "rviz", "fastlio.rviz")],
                parameters=[{"use_sim_time": False}],
            )
        )

    if start_scanplanner:
        actions.extend(
            [
            Node(
                package="scan_planner",
                executable="fastlio_pose_adapter",
                name="fastlio_pose_adapter",
                output="screen",
                parameters=[
                    {
                        "body_translation_in_imu": _vector(context, "body_in_imu"),
                        "body_rpy_in_imu": _vector(context, "body_rpy_in_imu"),
                        "sensor_translation_in_imu": _vector(context, "lidar_in_imu"),
                        "sensor_rpy_in_imu": _vector(context, "lidar_rpy_in_imu"),
                        "body_frame": LaunchConfiguration("body_frame").perform(context),
                        "sensor_frame": LaunchConfiguration("lidar_frame").perform(context),
                        "publish_tf": _as_bool(
                            LaunchConfiguration("publish_base_tf").perform(context)
                        ),
                        "publish_sensor_tf": _as_bool(
                            LaunchConfiguration("publish_lidar_tf").perform(context)
                        ),
                        "estimate_velocity": True,
                    }
                ],
                remappings=[
                    ("fastlio_odom", fastlio_odom_topic),
                    ("body_pose", body_pose_topic),
                    ("sensor_pose", sensor_pose_topic),
                ],
            ),
            Node(
                package="scan_planner",
                executable="fastlio_input_monitor.py",
                name="fastlio_input_monitor",
                output="screen",
                parameters=[
                    {
                        "expected_frame": world_frame,
                        "max_message_age": 0.3,
                        "max_stamp_skew": 0.25,
                        "min_odom_hz": 5.0,
                        "min_cloud_hz": 3.0,
                    }
                ],
                remappings=[
                    ("body_pose", body_pose_topic),
                    ("cloud", registered_cloud_topic),
                    ("inputs_ready", ready_topic),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(scan_launch),
                launch_arguments={
                    "is_real_world": "true",
                    "use_sim_time": "false",
                    "sensor_type": "lidar",
                    "navi_mode": LaunchConfiguration("navi_mode"),
                    "controller_mode": "closed_loop",
                    "start_controller": "true" if enable_control else "false",
                    "require_inputs_ready": "true",
                    "publish_robot_state": LaunchConfiguration("publish_robot_state"),
                    "body_pose_topic": body_pose_topic,
                    "sensor_pose_topic": sensor_pose_topic,
                    "cloud_topic": registered_cloud_topic,
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "world_frame": world_frame,
                    "cloud_is_world": "true",
                    "need_extrinsic": "false",
                    "keypoints_file": LaunchConfiguration("keypoints_file"),
                    "reference_path_file": LaunchConfiguration("reference_path_file"),
                }.items(),
            ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_livox_driver",
                default_value="false",
                description="Start the Livox Mid-360S driver in this launch process",
            ),
            DeclareLaunchArgument("start_fastlio", default_value="true"),
            DeclareLaunchArgument(
                "start_scanplanner",
                default_value="true",
                description="Start the pose adapter, input monitor, and SCAN-Planner",
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_mobile_3d",
                default_value="false",
                description="Start the SCAN 3D phone WebSocket bridge",
            ),
            DeclareLaunchArgument("mobile_http_port", default_value="8081"),
            DeclareLaunchArgument("mobile_ws_port", default_value="8891"),
            DeclareLaunchArgument("mobile_point_limit", default_value="40000"),
            DeclareLaunchArgument("mobile_cloud_rate", default_value="5.0"),
            DeclareLaunchArgument(
                "enable_control",
                default_value="false",
                description="Allow the closed-loop controller to publish real cmd_vel",
            ),
            DeclareLaunchArgument("fastlio_config_path", default_value=""),
            DeclareLaunchArgument(
                "fastlio_config_file", default_value="mid360_scanplanner.yaml"
            ),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument(
                "registered_cloud_topic", default_value="/cloud_registered"
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "world_frame",
                default_value="map",
                description="Common fixed frame for FAST-LIO and SCAN-Planner",
            ),
            DeclareLaunchArgument(
                "fastlio_imu_frame",
                default_value="body",
                description="FAST-LIO IMU child frame; change it if the real robot already owns 'body'",
            ),
            DeclareLaunchArgument(
                "body_frame",
                default_value="base",
                description="Physical robot moving root; the current real dog TF tree uses base",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="livox_frame",
                description="Livox driver frame and child frame of the fixed base-to-lidar TF",
            ),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("reference_path_file", default_value=""),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="false",
                description="Start the repository's simulated Go2 URDF publisher; keep false when the real robot already publishes its model",
            ),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="true",
                description="Publish world_frame -> body_frame from FAST-LIO; disable only if another node owns the exact same child TF",
            ),
            DeclareLaunchArgument(
                "publish_lidar_tf",
                default_value="true",
                description="Derive and publish fixed body_frame -> lidar_frame from the two IMU-referenced extrinsics; disable if the real URDF already owns lidar_frame",
            ),
            DeclareLaunchArgument("body_in_imu_x", default_value="0.0"),
            DeclareLaunchArgument("body_in_imu_y", default_value="0.0"),
            DeclareLaunchArgument("body_in_imu_z", default_value="0.0"),
            DeclareLaunchArgument("body_rpy_in_imu_x", default_value="0.0"),
            DeclareLaunchArgument("body_rpy_in_imu_y", default_value="0.0"),
            DeclareLaunchArgument("body_rpy_in_imu_z", default_value="0.0"),
            DeclareLaunchArgument("lidar_in_imu_x", default_value="-0.011"),
            DeclareLaunchArgument("lidar_in_imu_y", default_value="-0.02329"),
            DeclareLaunchArgument("lidar_in_imu_z", default_value="0.04412"),
            DeclareLaunchArgument("lidar_rpy_in_imu_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_rpy_in_imu_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_rpy_in_imu_z", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
