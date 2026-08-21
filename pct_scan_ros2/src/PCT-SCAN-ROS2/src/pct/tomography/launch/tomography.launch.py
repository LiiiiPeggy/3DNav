"""Launch the vendored PCT tomography node with explicit asset paths.

Modified for relocatable PCT-SCAN-ROS2 resources in 2026.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = os.environ.get("PCT_SCAN_ROS2_ROOT", os.getcwd())
    default_resource_root = os.path.join(project_root, "assets", "plaza")

    rsg_root_arg = DeclareLaunchArgument(
        'rsg_root',
        default_value=default_resource_root,
        description='Root directory for resources (containing pcd, rviz, tomogram folders)'
    )

    scene_name_arg = DeclareLaunchArgument(
        'scene_name',
        default_value='plaza',
        description='Name of the scene to load (e.g., plaza, building)'
    )

    # Node configuration
    tomography_node = Node(
        package='tomography',
        executable='tomography_node',
        name='tomography_node',
        output='screen',
        parameters=[{
            'rsg_root': LaunchConfiguration('rsg_root'),
            'scene_name': LaunchConfiguration('scene_name')
        }]
    )

    return LaunchDescription([
        rsg_root_arg,
        scene_name_arg,
        tomography_node
    ])
