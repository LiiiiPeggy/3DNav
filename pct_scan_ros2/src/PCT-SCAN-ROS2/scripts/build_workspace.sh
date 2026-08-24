#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PROJECT_ROOT/.deps/install}"

if [[ ! -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
  echo "[pct-scan] Missing GTSAM. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi
if [[ ! -d "$PCT_DEPS_ROOT/osqp-1.0.0/lib/cmake/osqp" ]]; then
  echo "[pct-scan] Missing OSQP. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi

# 构建必须使用系统 Python：conda base 激活时其 python3 缺少 ament 需要的
# catkin_pkg，会让 package.xml 解析直接失败。这里临时摘除 conda 的 bin。
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[pct-scan] conda 已激活（${CONDA_PREFIX}），临时移除其 bin 以使用系统 Python"
  _pct_sys_path="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vx "${CONDA_PREFIX}/bin" | paste -sd: || true)"
  PATH="${_pct_sys_path:-$PATH}"
  unset _pct_sys_path CONDA_PREFIX CONDA_DEFAULT_ENV
fi

set +u
source /opt/ros/humble/setup.bash
set -u
export GTSAM_DIR="$PCT_DEPS_ROOT/gtsam-4.2/lib/cmake/GTSAM"
export osqp_DIR="$PCT_DEPS_ROOT/osqp-1.0.0/lib/cmake/osqp"
export OSQP_DIR="$osqp_DIR"
export CMAKE_PREFIX_PATH="$GTSAM_DIR:$osqp_DIR${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$PROJECT_ROOT"
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_DIR="$GTSAM_DIR" \
    -Dosqp_DIR="$osqp_DIR" \
  "$@"
