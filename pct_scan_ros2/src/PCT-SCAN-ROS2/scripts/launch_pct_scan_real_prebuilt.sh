#!/usr/bin/env bash
# Launch real-robot PREBUILT-map 3D navigation (PCT global + SCAN Mode 3).
# Phase B — explicit opt-in; does NOT affect Phase A (launch_pct_scan_real.sh).
#
# Prerequisite: Odin SLAM in RELOCALIZATION mode
#   (SLAM/3run_relocalization.sh; map F1q9000.bin under SLAM/src/odin_ros_driver/map)
#   publishing /state_estimation + /registered_scan in frame odin_map.
#
# Defaults (declared in the launch file, NOT hard-coded here so user overrides win):
#   tomogram_path := $PCT_SCAN_ROS2_ROOT/maps/F19000_map.pickle
#   world_frame   := odin_map
#   start_mode    := 1   (Start 自动 = 当前机器人位姿；用户只需设 Goal；0=手动)
#
# Overrides pass through ("$@"): e.g. tomogram_path:=/x/a.pickle world_frame:=odom \
#     start_mode:=0 max_vel:=0.3
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"

echo "[pct-scan-real-prebuilt] 预检 Odin 重定位话题（帧应为 odin_map）..."
for tp in /state_estimation /registered_scan; do
  rate="$(timeout 3 ros2 topic hz "$tp" 2>/dev/null | grep -m1 'average rate' || true)"
  echo "  $tp -> ${rate:-未收到消息（请确认 SLAM/3run_relocalization.sh 已启动）}"
done

echo "[pct-scan-real-prebuilt] RViz：Fixed Frame=odin_map；Start 自动=当前位姿(start_mode=1)；"
echo "  设 Goal → /pct_path → SCAN Mode3 → /cmd_vel。"
echo "[pct-scan-real-prebuilt] 监控：ros2 topic echo /pct_path  /  /cmd_vel  /  /planning/bspline"

exec ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py "$@"
