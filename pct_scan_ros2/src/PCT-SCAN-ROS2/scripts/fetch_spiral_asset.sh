#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
DESTINATION="$PROJECT_ROOT/assets/spiral/pcd/spiral0.3_2.pcd"
EXPECTED_SHA256="24acbdccc1bb743cf698cfbb5dd739aa6253977cd22dc9c089c4e3d9e36248ff"
URL="https://raw.githubusercontent.com/ZJU-FAST-Lab/3D2M-planner/main/planner/src/read_pcd/PCDFiles/spiral0.3_2.pcd"

mkdir -p "$(dirname "$DESTINATION")"
temporary="$(mktemp "$PROJECT_ROOT/assets/spiral/pcd/.spiral.XXXXXX")"
trap 'rm -f -- "$temporary"' EXIT
curl --fail --location --retry 4 --retry-all-errors \
  --output "$temporary" "$URL"
actual="$(sha256sum "$temporary" | awk '{print $1}')"
if [[ "$actual" != "$EXPECTED_SHA256" ]]; then
  echo "[spiral] Checksum mismatch: expected $EXPECTED_SHA256, got $actual" >&2
  exit 3
fi
mv -- "$temporary" "$DESTINATION"
trap - EXIT
echo "[spiral] Downloaded and verified: $DESTINATION"

