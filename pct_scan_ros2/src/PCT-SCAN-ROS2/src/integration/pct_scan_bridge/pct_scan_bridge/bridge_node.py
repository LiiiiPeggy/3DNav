"""ROS 2 bridge between PCT global planning and SCAN local planning."""

from __future__ import annotations

import numpy as np
from nav_msgs.msg import Odometry

from pct_planner import planner_node as upstream_planner_node

from .dynamic_planner import DynamicLayerTomogramPlanner
from .tomogram_visualization import build_full_cost_tomogram


BUILDING_START = np.array([5.0, 5.0, 0.0], dtype=np.float32)
BUILDING_GOAL = np.array([-6.0, -1.0, 14.0], dtype=np.float32)
SPIRAL_DEMO_GOAL = np.array([3.2, -22.2, 4.0], dtype=np.float32)


class PCTScanBridge(upstream_planner_node.PCTPlanner):
    """Reuse PCT's ROS interaction layer while owning integration fixes here."""

    def __init__(self):
        # PCTPlanner constructs the class through this module-level symbol.
        # Override it only inside this bridge process; the dependency checkout
        # and its installed files stay untouched.
        upstream_planner_node.TomogramPlanner = DynamicLayerTomogramPlanner
        super().__init__()
        self._configure_auto_start()

    def _configure_auto_start(self):
        """start_mode=1（实机预建图默认）：把当前机器人位姿作为 PCT 起点。

        start_mode=1 订阅 /state_estimation，把 Start marker/起点持续吸到当前位姿
        （plan=False，不会触发重规划）；用户只需设置 Goal。start_mode=0 保持手动
        （marker / /initialpose），与原行为一致。
        """
        self._auto_start_enabled = False
        self._auto_start_last = None
        self._in_auto_start = False
        self.declare_parameter("start_mode", 0)
        self.declare_parameter("start_pose_odom_topic", "/state_estimation")
        start_mode = (
            self.get_parameter("start_mode").get_parameter_value().integer_value
        )
        if start_mode != 1:
            return
        topic = (
            self.get_parameter("start_pose_odom_topic")
            .get_parameter_value()
            .string_value
        )
        if not topic:
            return
        self._auto_start_enabled = True
        self._start_odom_sub = self.create_subscription(
            Odometry, topic, self._state_estimation_cb, 10
        )
        self.get_logger().info(
            f"start_mode=1: PCT Start 自动取自 {topic}（当前机器人位姿）; 设 Goal 后冻结"
        )

    def _state_estimation_cb(self, msg):
        if not self._auto_start_enabled:
            return
        pose = msg.pose.pose.position
        current = np.array([pose.x, pose.y, pose.z], dtype=np.float32)
        if self._auto_start_last is not None and (
            np.linalg.norm(current - self._auto_start_last) < 0.15
        ):
            return
        self._auto_start_last = current
        self._in_auto_start = True
        try:
            self.set_start(
                float(pose.x), float(pose.y), float(pose.z),
                plan=False, update_marker=True,
            )
        except Exception as error:  # planner 未就绪时静默等待下一帧
            self.get_logger().debug(f"auto-start skipped: {error}")
        finally:
            self._in_auto_start = False

    def set_start(self, x, y, z, plan=True, update_marker=True):
        # 手动移动 Start（marker / /initialpose）会关闭自动跟随。
        if not self._in_auto_start and getattr(self, "_auto_start_enabled", False):
            self._auto_start_enabled = False
        super().set_start(x, y, z, plan=plan, update_marker=update_marker)

    def set_goal(self, x, y, z, plan=True, update_marker=True):
        # 一旦用户设 Goal，冻结自动 Start，避免跟踪中起点漂移。
        if getattr(self, "_auto_start_enabled", False):
            self._auto_start_enabled = False
        super().set_goal(x, y, z, plan=plan, update_marker=update_marker)

    def configure_scene(self, scene_name):
        # The upstream node calls configure_scene() after rclpy.Node has been
        # initialized but before it constructs and loads TomogramPlanner.  That
        # makes this the earliest safe point to expose bridge-owned A* tuning
        # as normal ROS parameters.
        self.declare_parameter("centerline_bias_enabled", True)
        self.declare_parameter("preferred_clearance", 0.5)
        self.declare_parameter("clearance_cost", 20.0)
        self.declare_parameter("astar_step_cost_weight", 1.0)
        self.declare_parameter("tomogram_visual_min_cost", 0.0)
        self.declare_parameter("tomogram_visual_max_cost", 50.0)
        self.tomogram_visual_min_cost = (
            self.get_parameter("tomogram_visual_min_cost")
            .get_parameter_value()
            .double_value
        )
        self.tomogram_visual_max_cost = (
            self.get_parameter("tomogram_visual_max_cost")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.centerline_bias_enabled = (
            self.get_parameter("centerline_bias_enabled")
            .get_parameter_value()
            .bool_value
        )
        DynamicLayerTomogramPlanner.preferred_clearance = (
            self.get_parameter("preferred_clearance")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.clearance_cost = (
            self.get_parameter("clearance_cost").get_parameter_value().double_value
        )
        DynamicLayerTomogramPlanner.astar_step_cost_weight = (
            self.get_parameter("astar_step_cost_weight")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            "PCT A* centerline bias: "
            f"enabled={DynamicLayerTomogramPlanner.centerline_bias_enabled}, "
            f"preferred_clearance="
            f"{DynamicLayerTomogramPlanner.preferred_clearance:.2f} m, "
            f"clearance_cost={DynamicLayerTomogramPlanner.clearance_cost:.1f}, "
            f"step_weight={DynamicLayerTomogramPlanner.astar_step_cost_weight:.1f}"
        )
        if scene_name.lower() == "building":
            self.tomo_file = "building2_9"
            self.start_pos = BUILDING_START.copy()
            self.end_pos = BUILDING_GOAL.copy()
        else:
            super().configure_scene(scene_name)
            if scene_name.lower() == "spiral":
                # The upstream endpoint returns to the ground after traversing
                # the entire 426 m spiral.  This shorter two-level route is a
                # practical, deterministic RViz demo while markers still allow
                # selecting any higher platform.
                self.end_pos = SPIRAL_DEMO_GOAL.copy()

        self.declare_parameter("use_scene_waypoint_overrides", False)
        values = {}
        for waypoint_name, waypoint in (
            ("start", self.start_pos),
            ("goal", self.end_pos),
        ):
            for axis, default in zip("xyz", waypoint):
                parameter_name = f"scene_{waypoint_name}_{axis}"
                self.declare_parameter(parameter_name, float(default))
                values[parameter_name] = (
                    self.get_parameter(parameter_name)
                    .get_parameter_value()
                    .double_value
                )
        if (
            self.get_parameter("use_scene_waypoint_overrides")
            .get_parameter_value()
            .bool_value
        ):
            self.start_pos = np.array(
                [values[f"scene_start_{axis}"] for axis in "xyz"],
                dtype=np.float32,
            )
            self.end_pos = np.array(
                [values[f"scene_goal_{axis}"] for axis in "xyz"],
                dtype=np.float32,
            )

    def publish_tomogram(self):
        """Publish the paper's complete 0--50 PCT travel-cost field."""
        planner_points, costs = build_full_cost_tomogram(
            np.where(self.planner.elev_g_valid, self.planner.elev_g, np.nan),
            self.planner.trav,
            self.planner.resolution,
            self.planner.center,
            self.planner.offset,
            self.planner.slice_dh,
            z_offset=self.tomogram_visual_z_offset,
            min_cost=self.tomogram_visual_min_cost,
            max_cost=self.tomogram_visual_max_cost,
        )
        world_points = self.planner_to_world(planner_points)
        cloud_points = np.column_stack((world_points, costs)).astype(np.float32)

        header = upstream_planner_node.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.tomogram_pub.publish(
            upstream_planner_node.pc2.create_cloud(
                header,
                upstream_planner_node.TOMOGRAM_POINT_FIELDS,
                cloud_points,
            )
        )
        self.get_logger().info(
            f"Published {cloud_points.shape[0]} full-cost tomogram cells on "
            f"{self.tomogram_topic}; visualization range="
            f"[{self.tomogram_visual_min_cost:.1f}, "
            f"{self.tomogram_visual_max_cost:.1f}]"
        )


def main(args=None):
    upstream_planner_node.rclpy.init(args=args)
    node = PCTScanBridge()
    try:
        upstream_planner_node.rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        upstream_planner_node.rclpy.try_shutdown()
