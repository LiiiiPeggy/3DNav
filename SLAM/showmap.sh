#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: publish saved Odin map for standalone visualization
# ################################
cd "$SCRIPT_DIR"

# Optional arg: the map PCD to view, used as-is (absolute path, or relative to
# the SLAM workspace root — the script cds there). Default is the prebuilt map.
#   bash showmap.sh                       -> src/odin_ros_driver/map/map_20260807_151455.pcd
#   bash showmap.sh /path/to/maptest.pcd  -> that file
map_file=${1:-src/odin_ros_driver/map/map_20260807_151455.pcd}

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="$map_file" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map \
  -r __node:=standalone_overall_map_publisher &

# ################################
# Bash: clean up map publisher when standalone viewer exits
# ################################
cleanup() {
  pkill -f 'pcd_to_pointcloud.*standalone_overall_map_publisher' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Launch RViz immediately: the publisher is a periodic volatile publisher,
# so subscribing now catches the first ~10 s tick (no sleep).
ros2 run rviz2 rviz2 -d \
  "$SCRIPT_DIR/src/odin_ros_driver/config/overall_map.rviz"
