#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: run map_downsample PLY->VoxelGrid->PCD
# ################################
# Run from the SLAM workspace root. Usage:
#   bash 5downsample.sh [input.ply] [output.pcd] [leaf_size]
#   defaults: input = prebuilt Odin map PLY, output = input with .ply->.pcd, leaf = 0.05
input=${1:-src/odin_ros_driver/map/map_20260807_151455.ply}

args=("$input")
[ -n "$2" ] && args+=("$2")
[ -n "$3" ] && args+=("$3")

./install/odin_ros_driver/lib/odin_ros_driver/map_downsample "${args[@]}"
