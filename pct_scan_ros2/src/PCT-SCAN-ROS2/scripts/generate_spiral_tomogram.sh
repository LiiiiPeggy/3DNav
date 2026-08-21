#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"

exec "$SCRIPT_DIR/generate_tomogram.sh" \
  --input "$PROJECT_ROOT/assets/spiral/pcd/spiral0.3_2.pcd" \
  --output "$PROJECT_ROOT/assets/spiral/tomogram/spiral0.3_2_ros2.pickle" \
  --resolution 0.20 --ground-height 0.0 --slice-height 0.50 \
  --kernel-size 7 --interval-min 0.50 --interval-free 0.65 \
  --slope-max 0.40 --step-max 0.30 --standable-ratio 0.40 \
  --cost-barrier 50.0 --safe-margin 1.20 --inflation 0.20 \
  --backend cpu "$@"
