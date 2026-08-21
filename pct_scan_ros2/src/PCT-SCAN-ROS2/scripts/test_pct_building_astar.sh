#!/usr/bin/env bash
# Headless ground-to-upper-floor PCT A* smoke test for Building.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/building/tomogram/building2_9_ros2.pickle"
ROUTE_OUTPUT="$PROJECT_ROOT/assets/building/routes/ground_to_upper.csv"

for required_path in \
  "$TOMOGRAM"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[building-astar] Missing required path: $required_path" >&2
    exit 2
  fi
done

exec ros2 run pct_scan_bridge pct_route_smoke \
  --tomogram "$TOMOGRAM" \
  --output "$ROUTE_OUTPUT" \
  --start 5.0 5.0 0.0 \
  --goal -6.0 -1.0 14.0 \
  --snap-radius 12 \
  --require-cross-floor \
  --min-height-gain 10.0 \
  "$@"
