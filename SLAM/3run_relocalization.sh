#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: launch Odin relocalization and publish map on /overall_map
# ################################
ros2 launch odin_ros_driver odin1_ros2.launch.py \
  config_file:="$SCRIPT_DIR/src/odin_ros_driver/config/control_command_relocalization.yaml" \
  enable_rviz:=true \
  publish_overall_map:=true \
  overall_map_pcd:="$SCRIPT_DIR/src/odin_ros_driver/map/map_20260807_151455.pcd"
