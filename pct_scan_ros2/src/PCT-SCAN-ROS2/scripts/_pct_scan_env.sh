#!/usr/bin/env bash
# Shared runtime environment. Source this file; do not execute it directly.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This helper must be sourced by a PCT-SCAN-ROS2 launch script." >&2
  exit 2
fi

# 运行必须使用系统 Python：conda base 激活时其 python3 缺少 ament/rospkg 依赖
# （如 catkin_pkg），会让 ROS 2 的 Python 节点无法启动。这里临时摘除 conda 的 bin。
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[pct-scan] conda 已激活（${CONDA_PREFIX}），临时移除其 bin 以使用系统 Python"
  _pct_sys_path="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vx "${CONDA_PREFIX}/bin" | paste -sd: || true)"
  PATH="${_pct_sys_path:-$PATH}"
  unset _pct_sys_path CONDA_PREFIX CONDA_DEFAULT_ENV
fi

PCT_SCAN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PCT_SCAN_PROJECT_ROOT="$(cd "$PCT_SCAN_SCRIPT_DIR/.." && pwd -P)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[pct-scan] ROS 2 Humble was not found at /opt/ros/humble." >&2
  return 2
fi
if [[ ! -f "$PCT_SCAN_PROJECT_ROOT/install/setup.bash" ]]; then
  echo "[pct-scan] Workspace is not built. Run ./scripts/build_workspace.sh." >&2
  return 2
fi

# Generated ROS setup files legitimately inspect optional unset variables.
set +u
source /opt/ros/humble/setup.bash

PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PCT_SCAN_PROJECT_ROOT/.deps/install}"
if [[ ! -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/libgtsam.so.4" ]]; then
  echo "[pct-scan] PCT dependencies are missing. Run ./scripts/setup_pct_dependencies.sh." >&2
  return 2
fi

export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source "$PCT_SCAN_PROJECT_ROOT/install/setup.bash"
set -u

export PCT_SCAN_ROS2_ROOT="$PCT_SCAN_PROJECT_ROOT"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PCT_SCAN_PROJECT_ROOT/.ros/log}"
mkdir -p "$ROS_LOG_DIR"
