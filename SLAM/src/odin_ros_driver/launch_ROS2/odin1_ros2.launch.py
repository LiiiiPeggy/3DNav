
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')
    
    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )
    
    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )

    # ################################
    # Python: declare RViz-on and relocalization map publish options
    # ################################
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Start the Odin RViz'
    )
    publish_overall_map_arg = DeclareLaunchArgument(
        'publish_overall_map',
        default_value='false',
        description='Publish relocalization map PCD on /overall_map'
    )
    overall_map_pcd_arg = DeclareLaunchArgument(
        'overall_map_pcd',
        default_value='',
        description='Path to the map PCD published on /overall_map'
    )

    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
       # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }],
        remappings=[
            ('odin1/odometry', '/state_estimation'),
        ]
    )

    # Adapter node: XYZRGB→XYZI conversion + vehicle-relative range filtering
    registered_scan_adapter_node = Node(
        package='odin_ros_driver',
        executable='registered_scan_adapter_node',
        name='registered_scan_adapter_node',
        output='screen',
        parameters=[{
            'scan_min_range': 0.2,
            'input_topic': '/odin1/cloud_slam',
            'output_topic': '/registered_scan',
            'state_topic': '/state_estimation',
        }]
    )

    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f) 
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # Cloud reprojection node
    reprojection_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f) 
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path 
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',  
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # Image overlay node - overlays reprojected points on camera image
    overlay_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',  
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    # Create RViz2 node - loads specified configuration file
    # ################################
    # Python: make Odin RViz optional via enable_rviz
    # ################################
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # ################################
    # Python: optionally publish relocalization map on /overall_map
    # ################################
    overall_map_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='overall_map_publisher',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('publish_overall_map')
        ),
        parameters=[{
            'file_name': LaunchConfiguration('overall_map_pcd'),
            'tf_frame': 'odin_map',
            'publishing_period_ms': 10000,
        }],
        remappings=[
            ('cloud_pcd', '/overall_map'),
        ]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)  # Add RViz configuration argument

    # ################################
    # Python: declare RViz and map-publish options before dependent nodes
    # ################################
    ld.add_action(enable_rviz_arg)
    ld.add_action(publish_overall_map_arg)
    ld.add_action(overall_map_pcd_arg)

    ld.add_action(host_sdk_node)
    ld.add_action(registered_scan_adapter_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(image_overlay_node)
    ld.add_action(rviz_node)  # Add RViz node
    ld.add_action(overall_map_node)

    return ld
