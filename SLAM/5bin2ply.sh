#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: convert Odin .bin map to PLY via map_to_ply
# ################################
# Run from the SLAM workspace root. Usage:
#   bash 4trans2pcd.sh [input.bin] [output.ply]
#   defaults: input = prebuilt Odin map BIN, output = input with .bin->.ply
# map_to_ply converts the device-private .bin map (LCKF v1) to a viewable PLY;
# the PLY is then downsampled to PCD by 5downsample.sh. arm64 is the robot
# build (RK3588); amd64 is the x86_64 dev build.
if [ "$(uname -m)" = "aarch64" ]; then
  tool=src/odin_ros_driver/map/map_to_ply_arm64
else
  tool=src/odin_ros_driver/map/map_to_ply_amd64
fi
chmod +x "$tool"

input=${1:-src/odin_ros_driver/map/map_20260807_151455.bin}

args=("$input")
[ -n "$2" ] && args+=("$2")

"$tool" "${args[@]}"
