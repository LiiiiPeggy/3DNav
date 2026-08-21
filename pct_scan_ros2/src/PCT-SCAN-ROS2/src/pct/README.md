# Vendored PCT Planner ROS 2 sources

This directory contains the source form required to build the PCT global
planner and the optional CUDA tomography generator inside the same colcon
workspace as PCT-SCAN-ROS2.

Upstream lineage:

- Original project: <https://github.com/byangw/PCT_planner>
- ROS 2 port: <https://github.com/2473o/PCT_planner/tree/feat/ros2pkg>
- Local ROS 2 deployment base: commit
  `ae9a0406e138f3b609ee1b9a2dd4208ab73a0d15`

The original PCT project is distributed under GNU GPL version 2. Its NOTICE
permits redistribution under GPL version 2 or, at the recipient's option, a
later version. The complete license is available in
`../../LICENSES/GPL-2.0-or-later.txt`, and the original attribution is retained
in `NOTICE`.

Modifications made for this repository in August 2026 include:

- ROS 2 Humble packaging and launch integration;
- relocatable tomogram paths instead of home-directory hard coding;
- interactive 3-D start/goal markers and explicit route planning;
- raw tomogram visualization and coordinate conversion;
- dynamic tomogram-layer endpoint snapping;
- OSQP 1.0 and GTSAM 4.2 build compatibility.

These PCT-derived files remain GPL-2.0-or-later. They are not relicensed by
the Apache-2.0 license used by the SCAN planner packages.
