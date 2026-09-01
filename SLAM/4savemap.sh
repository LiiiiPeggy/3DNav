#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ################################
# Bash: save the Odin SLAM map via set_param.sh
# ################################
# Writes 'set save_map 1' to /tmp/odin_command.txt, which host_sdk_sample
# (running via 2run_slam.sh) picks up and sends to the device. The saved
# .bin map lands under src/odin_ros_driver/map/{save_time}/.
# NOTE: the driver must be running in SLAM mode (2run_slam.sh) for this to work.
bash "$SCRIPT_DIR/src/odin_ros_driver/set_param.sh" save_map 1
