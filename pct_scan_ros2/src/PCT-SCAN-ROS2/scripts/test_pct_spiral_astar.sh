#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
exec ros2 run pct_scan_bridge pct_route_smoke \
  --tomogram "$PCT_SCAN_PROJECT_ROOT/assets/spiral/tomogram/spiral0.3_2_ros2.pickle" \
  --start -16 -6 0 --goal 3.2 -22.2 4.0 --snap-radius 12 \
  --require-cross-floor --min-height-gain 3 --min-route-length 30 \
  --output "$PCT_SCAN_PROJECT_ROOT/assets/spiral/routes/lower_to_l2.csv" "$@"

