#!/usr/bin/env bash
# Launch Odin real-robot local navigation (Phase A, map-free local navigation).
# Prerequisite: Odin SLAM already running via SLAM/2run_slam.sh
#   (publishes /registered_scan and /state_estimation in the odom frame).
# RViz 另开终端：ros2 launch scan_planner rviz.launch.py
# 可覆盖（透传 "$@"）：如 collision_radius:=0.12
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  controller_mode:=closed_loop \
  sensor_type:=lidar \
  "$@"
