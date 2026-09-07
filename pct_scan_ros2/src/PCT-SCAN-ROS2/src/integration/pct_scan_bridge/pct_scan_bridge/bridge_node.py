"""ROS 2 bridge between PCT global planning and SCAN local planning."""

from __future__ import annotations

import numpy as np

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
