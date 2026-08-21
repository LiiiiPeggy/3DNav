#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
TOMOGRAM="$PCT_SCAN_PROJECT_ROOT/assets/plaza/tomogram/plaza3_10_ros2.pickle"
PCD_MAP="$PCT_SCAN_PROJECT_ROOT/assets/plaza/pcd/plaza3_10.pcd"

for path in "$TOMOGRAM" "$PCD_MAP"; do
  [[ -f "$path" ]] || { echo "[plaza] Missing asset: $path" >&2; exit 2; }
done

exec ros2 launch pct_scan_bridge paper_scene_pct_scan.launch.py \
  scene_name:=plaza tomogram_path:="$TOMOGRAM" pcd_map_file:="$PCD_MAP" \
  scene_start_x:=0.0 scene_start_y:=0.0 scene_start_z:=0.0 \
  scene_goal_x:=23.0 scene_goal_y:=10.0 scene_goal_z:=0.0 \
  init_x:=0.0 init_y:=0.0 init_z:=0.4 \
  map_size_x:=65.0 map_size_y:=65.0 map_size_z:=10.0 \
  planning_horizon:=0.8 max_vel:=0.6 "$@"

