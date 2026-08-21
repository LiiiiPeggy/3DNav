#!/usr/bin/env python3
"""Check the live FAST-LIO inputs used by the real-robot planner."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class FastlioInputMonitor(Node):
    def __init__(self):
        super().__init__("fastlio_input_monitor")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("max_message_age", 0.5)
        self.declare_parameter("max_stamp_skew", 0.25)
        self.declare_parameter("min_odom_hz", 5.0)
        self.declare_parameter("min_cloud_hz", 3.0)
        self.expected_frame = self.get_parameter("expected_frame").value
        self.max_message_age = float(self.get_parameter("max_message_age").value)
        self.max_stamp_skew = float(self.get_parameter("max_stamp_skew").value)
        self.min_odom_hz = float(self.get_parameter("min_odom_hz").value)
        self.min_cloud_hz = float(self.get_parameter("min_cloud_hz").value)
        if self.max_message_age <= 0.0 or self.max_stamp_skew < 0.0:
            raise ValueError("message age must be positive and stamp skew non-negative")

        self.last_odom_arrival = None
        self.last_cloud_arrival = None
        self.odom_hz = 0.0
        self.cloud_hz = 0.0
        self.last_odom_stamp = None
        self.last_cloud_stamp = None
        self.odom_frame = ""
        self.cloud_frame = ""
        self.invalid_odom = False
        self.invalid_cloud = True
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, "cloud", self.cloud_callback, qos_profile_sensor_data
        )
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(Bool, "inputs_ready", ready_qos)
        self.timer = self.create_timer(0.1, self.check_inputs)

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        quaternion_norm = math.sqrt(
            orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2
        )
        self.invalid_odom = (
            not all(math.isfinite(value) for value in values)
            or quaternion_norm < 1e-6
        )
        arrival = self.get_clock().now()
        if self.last_odom_arrival is not None:
            interval = (arrival - self.last_odom_arrival).nanoseconds * 1e-9
            if interval > 1e-4:
                measured_hz = 1.0 / interval
                self.odom_hz = (
                    measured_hz
                    if self.odom_hz == 0.0
                    else 0.3 * measured_hz + 0.7 * self.odom_hz
                )
        self.last_odom_arrival = arrival
        self.last_odom_stamp = self.stamp_seconds(message.header.stamp)
        self.odom_frame = message.header.frame_id

    def cloud_callback(self, message):
        arrival = self.get_clock().now()
        if self.last_cloud_arrival is not None:
            interval = (arrival - self.last_cloud_arrival).nanoseconds * 1e-9
            if interval > 1e-4:
                measured_hz = 1.0 / interval
                self.cloud_hz = (
                    measured_hz
                    if self.cloud_hz == 0.0
                    else 0.3 * measured_hz + 0.7 * self.cloud_hz
                )
        self.last_cloud_arrival = arrival
        self.last_cloud_stamp = self.stamp_seconds(message.header.stamp)
        self.cloud_frame = message.header.frame_id
        self.invalid_cloud = message.width * message.height == 0 or not message.data

    def check_inputs(self):
        now = self.get_clock().now()
        odom_age = (
            math.inf
            if self.last_odom_arrival is None
            else (now - self.last_odom_arrival).nanoseconds * 1e-9
        )
        cloud_age = (
            math.inf
            if self.last_cloud_arrival is None
            else (now - self.last_cloud_arrival).nanoseconds * 1e-9
        )
        stamp_skew = (
            math.inf
            if self.last_odom_stamp is None or self.last_cloud_stamp is None
            else abs(self.last_odom_stamp - self.last_cloud_stamp)
        )
        frame_ok = not self.expected_frame or (
            self.odom_frame == self.expected_frame and self.cloud_frame == self.expected_frame
        )
        ready = (
            not self.invalid_odom
            and not self.invalid_cloud
            and odom_age <= self.max_message_age
            and cloud_age <= self.max_message_age
            and self.odom_hz >= self.min_odom_hz
            and self.cloud_hz >= self.min_cloud_hz
            and stamp_skew <= self.max_stamp_skew
            and frame_ok
        )
        self.ready_pub.publish(Bool(data=ready))
        detail = (
            f"ready={str(ready).lower()} odom={self.odom_hz:.1f}Hz/{odom_age:.2f}s "
            f"cloud={self.cloud_hz:.1f}Hz/{cloud_age:.2f}s skew={stamp_skew:.3f}s "
            f"frames=({self.odom_frame or '-'}, {self.cloud_frame or '-'})"
        )
        if ready:
            self.get_logger().info(
                f"FAST-LIO input check: {detail}", throttle_duration_sec=5.0
            )
        else:
            self.get_logger().warning(
                f"FAST-LIO input check: {detail}", throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = FastlioInputMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
