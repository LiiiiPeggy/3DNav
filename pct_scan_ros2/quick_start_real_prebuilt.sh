#!/usr/bin/env bash
# quick_start_real_prebuilt.sh — PCT-SCAN-ROS2 实机预建图 3D 导航快速启动（Phase B）
#   PCT global(F19000_map.pickle) + SCAN Mode3 → /cmd_vel
#
#  部署：把本脚本复制到机器人上的 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 目录即可（自动定位），
#        然后  chmod +x quick_start_real_prebuilt.sh
#  用法：
#        ./quick_start_real_prebuilt.sh               # 默认：F19000 / odin_map / start_mode=1（自动开 RViz）
#        ./quick_start_real_prebuilt.sh start_mode:=0            # 手动 Start
#        ./quick_start_real_prebuilt.sh tomogram_path:=/x.pickle # 换地图
#        ./quick_start_real_prebuilt.sh use_rviz:=false          # 不开 RViz
#
#  前置：Odin SLAM 以重定位模式运行（SLAM/3run_relocalization.sh + F1q9000.bin），
#        /state_estimation(nav_msgs/Odometry) 与 /registered_scan(PointCloud2) 帧=odin_map。
#  注意：conda base 不要激活（脚本会自动摘除其 PATH）。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

# ---- 1) 定位 PCT-SCAN-ROS2 根（兼容脚本放在 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 下）----
if [[ -x "$SCRIPT_DIR/src/PCT-SCAN-ROS2/scripts/launch_pct_scan_real_prebuilt.sh" ]]; then
  ROOT="$SCRIPT_DIR/src/PCT-SCAN-ROS2"
elif [[ -x "$SCRIPT_DIR/scripts/launch_pct_scan_real_prebuilt.sh" ]]; then
  ROOT="$SCRIPT_DIR"
else
  echo "[quick-start] 错误：找不到 PCT-SCAN-ROS2。请把本脚本放在 …/pct_scan_ros2 或 …/PCT-SCAN-ROS2 下" >&2
  exit 2
fi
cd "$ROOT"

# ---- 2) 环境（source /opt/ros/humble + install；.deps LD_LIBRARY_PATH；缺失会报错退出）----
# shellcheck source=scripts/_pct_scan_env.sh
source "$ROOT/scripts/_pct_scan_env.sh"

# ---- 3) 前置检查：run.launch.py 的 depth remap 是否为“已修复”版本（预建图也经 run.launch）----
RUN_LAUNCH="$ROOT/src/planner/plan_manage/launch/run.launch.py"
if [[ -f "$RUN_LAUNCH" ]] && grep -q 'if depth else \[\]' "$RUN_LAUNCH"; then
  :  # 已是修复版
elif [[ -f "$RUN_LAUNCH" ]] && grep -Eq '^[[:space:]]*\("depth", depth\),' "$RUN_LAUNCH"; then
  echo "[quick-start] 错误：$RUN_LAUNCH 仍是旧版（无条件生成 -r depth:= 空 remap，会崩）。修复：" >&2
  echo "  把 remappings=[ ... ] 里那行  (\"depth\", depth),  改为只在非空时追加：" >&2
  echo "    ] + ([(\"depth\", depth)] if depth else [])," >&2
  exit 2
fi

# ---- 4) 预检 Odin 重定位话题（各等 3 s，非阻塞，仅供参考）----
echo "[quick-start] 预检 Odin 重定位话题（帧应为 odin_map）..."
for tp in /state_estimation /registered_scan; do
  rate="$(timeout 3 ros2 topic hz "$tp" 2>/dev/null | grep -m1 'average rate' || true)"
  echo "  $tp -> ${rate:-未收到消息（请确认 SLAM/3run_relocalization.sh 已启动）}"
done

echo "[quick-start] 启动实机预建图 3D 导航（PCT global + SCAN Mode3 → /cmd_vel）"
echo "[quick-start] 默认：tomogram=$PCT_SCAN_ROS2_ROOT/maps/F19000_map.pickle  world_frame=odin_map  start_mode=1"
echo "[quick-start] RViz 自动打开（Fixed Frame=odin_map）；Start 自动=当前位姿，设 Goal → /pct_path → SCAN Mode3 → /cmd_vel。"
echo "[quick-start] 不开 RViz 传 use_rviz:=false；另开终端监控："
echo "  ros2 topic echo /pct_path   /   ros2 topic echo /cmd_vel   /   ros2 topic echo /planning/bspline"

exec ros2 launch pct_scan_bridge real_prebuilt_scan.launch.py "$@"
