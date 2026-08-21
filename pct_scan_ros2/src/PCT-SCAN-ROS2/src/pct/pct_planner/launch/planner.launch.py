"""Launch the ROS 2 PCT planner.

Modified for relocatable PCT-SCAN-ROS2 integration in 2026.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    project_root = os.environ.get("PCT_SCAN_ROS2_ROOT", os.getcwd())
    default_resource_root = os.path.join(project_root, "assets", "plaza")

    # Example argument - adjust based on what planner_node actually needs
    rsg_root_arg = DeclareLaunchArgument(
        'rsg_root',
        default_value=default_resource_root,
        description='Root directory for resources'
    )

    scene_name_arg = DeclareLaunchArgument(
        'scene_name',
        default_value='Plaza',
        description='Name of the scene to load (e.g., Plaza, Building, Spiral)'
    )
    tomo_file_arg = DeclareLaunchArgument(
        'tomo_file',
        default_value='',
        description='Optional tomogram file override without .pickle suffix'
    )
    tomogram_path_arg = DeclareLaunchArgument(
        'tomogram_path',
        default_value='',
        description='Optional absolute path to a generated tomogram pickle'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame used by paths and interactive markers'
    )
    coord_mode_arg = DeclareLaunchArgument(
        'coord_mode',
        default_value='identity',
        description='Coordinate conversion: identity or sim_to_pct_180deg'
    )
    optimize_path_arg = DeclareLaunchArgument(
        'optimize_path',
        default_value='false',
        description='Run PCT GPMP optimization; keep false when SCAN performs local optimization'
    )
    publish_tomogram_arg = DeclareLaunchArgument(
        'publish_tomogram',
        default_value='true',
        description='Publish traversable tomogram cells for RViz'
    )
    tomogram_topic_arg = DeclareLaunchArgument(
        'tomogram_topic',
        default_value='/pct_tomogram',
        description='Traversable tomogram visualization topic'
    )
    restrict_endpoints_arg = DeclareLaunchArgument(
        'restrict_endpoints_to_main_components',
        default_value='false',
        description='Restrict marker snapping and visualization to seeded floor components'
    )
    endpoint_height_tolerance_arg = DeclareLaunchArgument(
        'endpoint_height_tolerance',
        default_value='0.4',
        description='Vertical band around each seeded endpoint floor, in metres'
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/pct_path',
        description='Path output topic'
    )
    start_topic_arg = DeclareLaunchArgument(
        'start_topic',
        default_value='/initialpose',
        description='RViz start pose topic'
    )
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic',
        default_value='/goal_pose',
        description='RViz goal pose topic'
    )
    clicked_point_topic_arg = DeclareLaunchArgument(
        'clicked_point_topic',
        default_value='/clicked_point',
        description='RViz Publish Point topic'
    )
    plan_on_startup_arg = DeclareLaunchArgument(
        'plan_on_startup',
        default_value='false',
        description='Run the built-in scene example immediately at startup'
    )
    clicked_point_mode_arg = DeclareLaunchArgument(
        'clicked_point_mode',
        default_value='alternate',
        description='How /clicked_point is consumed: alternate, start, or goal'
    )
    enable_interactive_markers_arg = DeclareLaunchArgument(
        'enable_interactive_markers',
        default_value='true',
        description='Enable draggable RViz interactive start and goal markers'
    )
    interactive_marker_namespace_arg = DeclareLaunchArgument(
        'interactive_marker_namespace',
        default_value='/pct_waypoints',
        description='Interactive marker server namespace'
    )
    snap_search_radius_cells_arg = DeclareLaunchArgument(
        'snap_search_radius_cells',
        default_value='10',
        description='Grid-cell search radius for snapping dragged markers to traversable tomogram cells'
    )
    snap_to_traversable_layer_arg = DeclareLaunchArgument(
        'snap_to_traversable_layer',
        default_value='true',
        description='Snap incoming waypoints to traversable tomogram layers'
    )
    plan_on_marker_release_arg = DeclareLaunchArgument(
        'plan_on_marker_release',
        default_value='false',
        description='Replan when a dragged interactive marker is released'
    )

    # Node configuration for pct_planner
    planner_node = Node(
        package='pct_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'rsg_root': LaunchConfiguration('rsg_root'),
            'scene_name': LaunchConfiguration('scene_name'),
            'tomo_file': LaunchConfiguration('tomo_file'),
            'tomogram_path': LaunchConfiguration('tomogram_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'coord_mode': LaunchConfiguration('coord_mode'),
            'optimize_path': LaunchConfiguration('optimize_path'),
            'publish_tomogram': LaunchConfiguration('publish_tomogram'),
            'tomogram_topic': LaunchConfiguration('tomogram_topic'),
            'restrict_endpoints_to_main_components': LaunchConfiguration(
                'restrict_endpoints_to_main_components'
            ),
            'endpoint_height_tolerance': LaunchConfiguration('endpoint_height_tolerance'),
            'path_topic': LaunchConfiguration('path_topic'),
            'start_topic': LaunchConfiguration('start_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'clicked_point_topic': LaunchConfiguration('clicked_point_topic'),
            'plan_on_startup': LaunchConfiguration('plan_on_startup'),
            'clicked_point_mode': LaunchConfiguration('clicked_point_mode'),
            'enable_interactive_markers': LaunchConfiguration('enable_interactive_markers'),
            'interactive_marker_namespace': LaunchConfiguration('interactive_marker_namespace'),
            'snap_search_radius_cells': LaunchConfiguration('snap_search_radius_cells'),
            'snap_to_traversable_layer': LaunchConfiguration('snap_to_traversable_layer'),
            'plan_on_marker_release': LaunchConfiguration('plan_on_marker_release'),
        }]
    )

    return LaunchDescription([
        rsg_root_arg,
        scene_name_arg,
        tomo_file_arg,
        tomogram_path_arg,
        frame_id_arg,
        coord_mode_arg,
        optimize_path_arg,
        publish_tomogram_arg,
        tomogram_topic_arg,
        restrict_endpoints_arg,
        endpoint_height_tolerance_arg,
        path_topic_arg,
        start_topic_arg,
        goal_topic_arg,
        clicked_point_topic_arg,
        plan_on_startup_arg,
        clicked_point_mode_arg,
        enable_interactive_markers_arg,
        interactive_marker_namespace_arg,
        snap_search_radius_cells_arg,
        snap_to_traversable_layer_arg,
        plan_on_marker_release_arg,
        planner_node
    ])
