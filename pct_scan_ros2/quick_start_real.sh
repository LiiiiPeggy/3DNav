#!/usr/bin/env bash
# quick_start_real.sh — PCT-SCAN-ROS2 实机快速启动（Phase A：Odin SLAM → SCAN 局部导航 → /cmd_vel）
#
#  部署：把本脚本复制到机器人上的 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 目录即可（自动定位），
#        然后  chmod +x quick_start_real.sh
#  用法：
#        ./quick_start_real.sh                       # 默认：is_real / mode1 / closed_loop / lidar
#        ./quick_start_real.sh max_vel:=0.3          # 可透传任意 run.launch.py 参数
#        ./quick_start_real.sh --rviz                # 顺带在本机(需图形界面)打开 RViz2
#  注：RViz 与导航相互独立，也可另开终端手动打开（见脚本末尾提示）。
#
#  前置：Odin SLAM 已运行，发布 /state_estimation(nav_msgs/Odometry, odom) 与
#        /registered_scan(PointCloud2, odom)。
#  注意：conda base 不要激活（脚本会自动摘除其 PATH）。
set -eo pipefail

# 解析 --rviz（仅此一个自有开关，其余参数原样透传给 run.launch.py）
OPEN_RVIZ=0
_launch_args=()
for _a in "$@"; do
  if [[ "$_a" == "--rviz" ]]; then OPEN_RVIZ=1; else _launch_args+=("$_a"); fi
done
set -- "${_launch_args[@]}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

# ---- 1) 定位 PCT-SCAN-ROS2 根（兼容脚本放在 pct_scan_ros2/ 或 PCT-SCAN-ROS2/ 下）----
if [[ -x "$SCRIPT_DIR/src/PCT-SCAN-ROS2/scripts/launch_pct_scan_real.sh" ]]; then
  ROOT="$SCRIPT_DIR/src/PCT-SCAN-ROS2"
elif [[ -x "$SCRIPT_DIR/scripts/launch_pct_scan_real.sh" ]]; then
  ROOT="$SCRIPT_DIR"
else
  echo "[quick-start] 错误：找不到 PCT-SCAN-ROS2。请把本脚本放在 …/pct_scan_ros2 或 …/PCT-SCAN-ROS2 下" >&2
  exit 2
fi
cd "$ROOT"

# ---- 2) 环境（source /opt/ros/humble + install；.deps LD_LIBRARY_PATH；缺失会报错退出）----
# shellcheck source=scripts/_pct_scan_env.sh
source "$ROOT/scripts/_pct_scan_env.sh"

# ---- 3) 前置检查：run.launch.py 的 depth remap 是否为“已修复”版本 ----
# 已修复 = 存在条件式 `if depth else []`；旧版 = 存在一条“未注释的”("depth", depth),
# 活动 remap（注释掉的旧块不算 bug，不会误报）。
RUN_LAUNCH="$ROOT/src/planner/plan_manage/launch/run.launch.py"
if [[ -f "$RUN_LAUNCH" ]] && grep -q 'if depth else \[\]' "$RUN_LAUNCH"; then
  :  # 已是修复版
elif [[ -f "$RUN_LAUNCH" ]] && grep -Eq '^[[:space:]]*\("depth", depth\),' "$RUN_LAUNCH"; then
  echo "[quick-start] 错误：$RUN_LAUNCH 仍是旧版（无条件生成 -r depth:= 空 remap，会崩）。修复：" >&2
  echo "  把 remappings=[ ... ] 里那行  (\"depth\", depth),  改为只在非空时追加：" >&2
  echo "    ] + ([(\"depth\", depth)] if depth else [])," >&2
  exit 2
fi

# ---- 4) 预检 Odin 话题（各等 3 s，非阻塞，仅供参考）----
echo "[quick-start] 预检 Odin 话题 ..."
for tp in /state_estimation /registered_scan; do
  rate="$(timeout 3 ros2 topic hz "$tp" 2>/dev/null | grep -m1 'average rate' || true)"
  echo "  $tp -> ${rate:-未收到消息（请确认 Odin 已启动）}"
done

if [[ "$OPEN_RVIZ" == "1" ]]; then
  if [[ -n "${DISPLAY:-}" ]]; then
    echo "[quick-start] --rviz：后台打开 RViz2（Ctrl+C 退出导航时一并关闭）..."
    ros2 launch scan_planner rviz.launch.py &
    _RVIZ_PID=$!
    trap '[[ -n "${_RVIZ_PID:-}" ]] && kill "$_RVIZ_PID" 2>/dev/null' EXIT
  else
    echo "[quick-start] --rviz 需要图形界面，但本会话 DISPLAY 未设置，跳过自动打开。"
    echo "[quick-start] 请在有显示的机器上另开终端："
    echo "  source /opt/ros/humble/setup.bash && source $ROOT/install/setup.bash"
    echo "  ros2 launch scan_planner rviz.launch.py"
  fi
fi

echo "[quick-start] 启动实机局部导航（is_real_world:=true navi_mode:=1 controller_mode:=closed_loop）"
echo "[quick-start] RViz 使用：Fixed Frame 设为 odom；看 /grid_map/occupancy 局部地图；"
echo "           用 2D Goal Pose 点目标（若在另一台机器，需能连通本机 ROS 图/DDS）。"
echo "[quick-start] 无 RViz 时，另开终端发一个目标点："
echo "  source /opt/ros/humble/setup.bash && source $ROOT/install/setup.bash"
echo "  ros2 topic pub -1 /move_base_simple/goal geometry_msgs/msg/PoseStamped \\"
echo "    '{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
echo "[quick-start] 监控：ros2 topic echo /cmd_vel   /   ros2 topic echo /state_estimation"

ros2 launch scan_planner run.launch.py \
  is_real_world:=true navi_mode:=1 controller_mode:=closed_loop sensor_type:=lidar \
  "$@"
