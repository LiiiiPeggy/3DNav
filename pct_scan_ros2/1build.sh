#!/usr/bin/env bash
# 1build.sh — PCT-SCAN-ROS2 一键构建（依赖 + 全量 colcon）
#
#  部署：把本脚本复制到机器人上的 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 目录即可（自动定位），
#        然后  chmod +x 1build.sh
#  用法：
#        ./1build.sh                       # 依赖(PCT .deps: gtsam/osqp) + 全量 colcon build
#        ./1build.sh --skip-deps           # 跳过依赖，只 colcon build
#  离线/无网：先 export QDLDL_SOURCE_DIR=/path/to/qdldl（osqp 不再联网 FetchContent）
#  注意：conda base 不要激活（两个子脚本会自动摘除其 PATH）。
set -eo pipefail

SKIP_DEPS=0
_extra=()
for _a in "$@"; do
  if [[ "$_a" == "--skip-deps" ]]; then SKIP_DEPS=1; else _extra+=("$_a"); fi
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

# ---- 1) 定位 PCT-SCAN-ROS2 根（兼容放在 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 下）----
if [[ -x "$SCRIPT_DIR/src/PCT-SCAN-ROS2/scripts/setup_pct_dependencies.sh" ]]; then
  ROOT="$SCRIPT_DIR/src/PCT-SCAN-ROS2"
elif [[ -x "$SCRIPT_DIR/scripts/setup_pct_dependencies.sh" ]]; then
  ROOT="$SCRIPT_DIR"
else
  echo "[1build] 错误：找不到 PCT-SCAN-ROS2。请把本脚本放在 …/pct_scan_ros2 或 …/PCT-SCAN-ROS2 下" >&2
  exit 2
fi
cd "$ROOT"

# ---- 2) 依赖（gtsam/osqp → .deps/install）----
if [[ "$SKIP_DEPS" == "0" ]]; then
  if [[ -n "${QDLDL_SOURCE_DIR:-}" ]]; then
    echo "[1build] 使用离线 QDLDL_SOURCE_DIR=$QDLDL_SOURCE_DIR"
  fi
  echo "[1build] 阶段 1/2：setup_pct_dependencies.sh ..."
  ./scripts/setup_pct_dependencies.sh
else
  echo "[1build] --skip-deps：跳过依赖构建"
fi

# ---- 3) 全量 colcon build----
echo "[1build] 阶段 2/2：build_workspace.sh（colcon build）..."
./scripts/build_workspace.sh "${_extra[@]}"

echo "[1build] 完成。source install/setup.bash 后建议自检三个场景："
echo "  ./scripts/test_pct_building_astar.sh --force && ./scripts/test_pct_plaza_astar.sh --force && ./scripts/test_pct_spiral_astar.sh --force"
