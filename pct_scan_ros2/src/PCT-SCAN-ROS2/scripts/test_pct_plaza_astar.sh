#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
exec ros2 run pct_scan_bridge pct_route_smoke \
  --tomogram "$PCT_SCAN_PROJECT_ROOT/assets/plaza/tomogram/plaza3_10_ros2.pickle" \
  --start 0 0 0 --goal 23 10 0 --snap-radius 12 --min-route-length 20 \
  --output "$PCT_SCAN_PROJECT_ROOT/assets/plaza/routes/plaza_demo.csv" "$@"

