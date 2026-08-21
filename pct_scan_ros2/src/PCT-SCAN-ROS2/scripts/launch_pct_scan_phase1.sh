#!/usr/bin/env bash
# ROS 2 generated setup files legitimately probe optional, unset variables.
# Enable nounset only after all overlays have been sourced.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
SCAN_ROOT="$PCT_SCAN_PROJECT_ROOT"
PCT_SCAN_ASSET_ROOT="${PCT_SCAN_ASSET_ROOT:-$SCAN_ROOT/assets/phase1}"

for required_path in \
  "$PCT_SCAN_ASSET_ROOT/source/scene/multifloor/mutifloor_upstream.pickle" \
  "$PCT_SCAN_ASSET_ROOT/source/scene/multifloor/pcd/collision_map.pcd"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[phase1] Missing required path: $required_path" >&2
    exit 2
  fi
done

export PCT_SCAN_ASSET_ROOT

exec ros2 launch scan_planner pct_scan_phase1.launch.py "$@"
