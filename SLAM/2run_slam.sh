#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: launch Odin SLAM mapping mode
# ################################
ros2 launch odin_ros_driver odin1_ros2.launch.py \
  config_file:="$SCRIPT_DIR/src/odin_ros_driver/config/control_command_slam.yaml" \
  enable_rviz:=true \
  publish_overall_map:=false
