"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from topic_resolver import _compute_topics, _should_publish_robot_state


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    enable_local_sensing = _as_bool(
        LaunchConfiguration("enable_local_sensing").perform(context)
    )
    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    # world_frame（默认 odom）仅声明，阶段 A 不做 TF lookup / URDF 加载；保留供阶段 B 帧对齐。
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    initial_path_topic = LaunchConfiguration("initial_path_topic").perform(context)
    reference_path_min_distance = float(
        LaunchConfiguration("reference_path_min_distance").perform(context)
    )
    reference_path_simplify_tolerance = float(
        LaunchConfiguration("reference_path_simplify_tolerance").perform(context)
    )
    planning_horizon = float(LaunchConfiguration("planning_horizon").perform(context))
    max_vel_text = LaunchConfiguration("max_vel").perform(context)
    max_acc_text = LaunchConfiguration("max_acc").perform(context)
    collision_radius_text = LaunchConfiguration("collision_radius").perform(context)
    collision_offset_text = LaunchConfiguration("collision_offset").perform(context)
    inflation_z_up_text = LaunchConfiguration("inflation_z_up").perform(context)
    inflation_z_down_text = LaunchConfiguration("inflation_z_down").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    mode_default_init = (-19.0, 1.0, 0.25) if navi_mode == 1 else (-5.5, 5.5, 0.5)
    initial_position = []
    for name, default in zip(("init_x", "init_y", "init_z"), mode_default_init):
        value = LaunchConfiguration(name).perform(context)
        initial_position.append(default if value == "" else float(value))
    init_x, init_y, init_z = initial_position

    topics = _compute_topics(
        is_real=is_real,
        sensor_type=sensor_type,
        enable_local_sensing=enable_local_sensing,
        body_pose_topic=body_pose_topic,
        sensor_pose_topic=sensor_pose_topic,
        cloud_topic=cloud_topic,
    )
    body_pose = topics["body_pose"]
    sensor_pose = topics["sensor_pose"]
    cloud = topics["cloud"]
    cloud_is_world = topics["cloud_is_world"]
    need_extrinsic = topics["need_extrinsic"]
    # Phase A lidar-only deployment.
    # Depth and intrinsics are reserved for Phase B.
    depth = ""
    intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "fsm.reference_path_min_distance": reference_path_min_distance,
        "fsm.reference_path_simplify_tolerance": reference_path_simplify_tolerance,
        "fsm.planning_horizon": planning_horizon,
        "manager.planning_horizon": planning_horizon,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    if max_vel_text:
        max_vel = float(max_vel_text)
        if max_vel <= 0.0:
            raise RuntimeError("max_vel must be positive")
        planner_overrides["manager.max_vel"] = max_vel
        planner_overrides["optimization.max_vel"] = max_vel
    if max_acc_text:
        max_acc = float(max_acc_text)
        if max_acc <= 0.0:
            raise RuntimeError("max_acc must be positive")
        planner_overrides["manager.max_acc"] = max_acc
        planner_overrides["optimization.max_acc"] = max_acc
    for text, parameter in (
        (collision_radius_text, "grid_map.double_cylinder_radius"),
        (collision_offset_text, "grid_map.double_cylinder_offset"),
        (inflation_z_up_text, "grid_map.obstacles_inflation_z_up"),
        (inflation_z_down_text, "grid_map.obstacles_inflation_z_down"),
    ):
        if text:
            value = float(text)
            if value < 0.0:
                raise RuntimeError(f"{parameter} must be non-negative")
            planner_overrides[parameter] = value
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", initial_path_topic),
            ],
        )
    ]
    # 真实分支默认不启动 robot_state_publisher（阶段 A 只验证
    # Odin SLAM → planner → /cmd_vel，不依赖 URDF / robot_description /
    # 机器人模型 / TF 机器人可视化）。
    # 仿真分支保持原仓库行为：节点参数块（robot_description 等）原样保留，
    # 本任务不修改仿真机器人模型来源。
    if _should_publish_robot_state(is_real, publish_robot_state):
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                             " use_gazebo:=false"]
                        )
                    },
                ],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[
                    controllers_yaml,
                    common,
                    {"init_x": init_x, "init_y": init_y, "init_z": init_z},
                ],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": init_x,
                            "init_y": init_y,
                            "init_z": init_z,
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/cmd_vel"),
                    ],
                )
            )

    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("initial_path", initial_path_topic),
                ],
            )
        )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "enable_local_sensing",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("reference_path_file", default_value=""),
            DeclareLaunchArgument("initial_path_topic", default_value="/initial_path"),
            DeclareLaunchArgument("reference_path_min_distance", default_value="0.5"),
            DeclareLaunchArgument("reference_path_simplify_tolerance", default_value="0.0"),
            DeclareLaunchArgument("planning_horizon", default_value="3.5"),
            DeclareLaunchArgument("max_vel", default_value=""),
            DeclareLaunchArgument("max_acc", default_value=""),
            DeclareLaunchArgument("collision_radius", default_value=""),
            DeclareLaunchArgument("collision_offset", default_value=""),
            DeclareLaunchArgument("inflation_z_up", default_value=""),
            DeclareLaunchArgument("inflation_z_down", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("enable_local_sensing", default_value="true"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value=""),
            DeclareLaunchArgument("init_y", default_value=""),
            DeclareLaunchArgument("init_z", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("body_pose_topic", default_value=""),
            DeclareLaunchArgument("sensor_pose_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("publish_robot_state", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
