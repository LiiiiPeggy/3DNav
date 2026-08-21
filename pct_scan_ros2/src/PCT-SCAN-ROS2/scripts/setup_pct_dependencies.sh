#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
DEPS_ROOT="${PCT_DEPS_ROOT:-$PROJECT_ROOT/.deps/install}"
SOURCE_ROOT="$PROJECT_ROOT/.deps/src"
BUILD_ROOT="$PROJECT_ROOT/.deps/build"
JOBS="${PCT_BUILD_JOBS:-2}"

command -v git >/dev/null
command -v cmake >/dev/null
command -v ninja >/dev/null
mkdir -p "$DEPS_ROOT" "$SOURCE_ROOT" "$BUILD_ROOT"

clone_if_missing() {
  local source_dir="$1"
  shift
  if [[ ! -d "$source_dir/.git" ]]; then
    git clone --recursive --depth 1 "$@" "$source_dir"
  fi
}

clone_if_missing "$SOURCE_ROOT/gtsam-4.2" \
  --branch 4.2 https://github.com/borglab/gtsam.git
clone_if_missing "$SOURCE_ROOT/osqp-1.0.0" \
  --branch v1.0.0 https://github.com/osqp/osqp.git

cmake -S "$SOURCE_ROOT/gtsam-4.2" -B "$BUILD_ROOT/gtsam-4.2" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT/gtsam-4.2" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF
cmake --build "$BUILD_ROOT/gtsam-4.2" --parallel "$JOBS"
cmake --install "$BUILD_ROOT/gtsam-4.2"

cmake -S "$SOURCE_ROOT/osqp-1.0.0" -B "$BUILD_ROOT/osqp-1.0.0" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT/osqp-1.0.0"
cmake --build "$BUILD_ROOT/osqp-1.0.0" --parallel "$JOBS"
cmake --install "$BUILD_ROOT/osqp-1.0.0"

echo "[pct-scan] Dependencies installed under $DEPS_ROOT"

