#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
TOMOGRAM="$PCT_SCAN_PROJECT_ROOT/assets/spiral/tomogram/spiral0.3_2_ros2.pickle"
PCD_MAP="$PCT_SCAN_PROJECT_ROOT/assets/spiral/pcd/spiral0.3_2.pcd"

for path in "$TOMOGRAM" "$PCD_MAP"; do
  [[ -f "$path" ]] || { echo "[spiral] Missing asset: $path" >&2; exit 2; }
done

exec ros2 launch pct_scan_bridge paper_scene_pct_scan.launch.py \
  scene_name:=spiral tomogram_path:="$TOMOGRAM" pcd_map_file:="$PCD_MAP" \
  scene_start_x:=-16.0 scene_start_y:=-6.0 scene_start_z:=0.0 \
  scene_goal_x:=3.2 scene_goal_y:=-22.2 scene_goal_z:=4.0 \
  init_x:=-16.0 init_y:=-6.0 init_z:=0.6 \
  map_size_x:=90.0 map_size_y:=50.0 map_size_z:=28.0 \
  planning_horizon:=0.8 max_vel:=0.5 "$@"

