#!/usr/bin/env python3
"""
Convert PointXYZRGB point cloud to PointXYZI.

Odin outputs /odin1/cloud_slam as PointXYZRGB (has 'rgb' field).
CMU planner expects PointXYZI (needs 'intensity' field).
This node strips 'rgb' and republishes with intensity=0 (compatible format).

Usage:
  ros2 run odin_ros_driver xyzrgb_to_xyzi.py \
    --ros-args -p input_topic:=/odin1/cloud_slam -p output_topic:=/odin1/cloud_slam_xyzi
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class XyzrgbToXyziNode(Node):
    def __init__(self):
        super().__init__('xyzrgb_to_xyzi')

        self.declare_parameter('input_topic', '/odin1/cloud_slam')
        self.declare_parameter('output_topic', '/odin1/cloud_slam_xyzi')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10)

        self.get_logger().info(f'XYZRGB→XYZI: {input_topic} → {output_topic}')

    def callback(self, msg):
        # Build output fields: x, y, z, intensity
        out_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = out_fields
        out.point_step = 16  # 4 floats × 4 bytes
        out.row_step = out.point_step * out.width
        out.is_bigendian = False
        out.is_dense = msg.is_dense

        # Parse input point cloud
        in_data = np.frombuffer(msg.data, dtype=np.uint8)
        point_count = msg.width * msg.height  # unordered cloud: height=1

        # Find field offsets in input message
        offsets = {f.name: f.offset for f in msg.fields}
        in_step = msg.point_step

        out_data = bytearray(point_count * out.point_step)
        for i in range(point_count):
            base = i * in_step
            x = struct.unpack_from('f', in_data, base + offsets.get('x', 0))[0]
            y = struct.unpack_from('f', in_data, base + offsets.get('y', 4))[0]
            z = struct.unpack_from('f', in_data, base + offsets.get('z', 8))[0]
            out_base = i * out.point_step
            struct.pack_into('ffff', out_data, out_base, x, y, z, 0.0)

        out.data = bytes(out_data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = XyzrgbToXyziNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
