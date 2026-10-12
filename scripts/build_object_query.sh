#!/usr/bin/env bash
# Build lidar_outline and its nested custom ROS service interface package with colcon.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd /home/jovan/ros2_ws
# Explicit paths discover the interface package nested in this repository.
exec colcon build --base-paths "$script_dir/.." \
  "$script_dir/../interfaces/lidar_outline_interfaces" \
  --packages-select lidar_outline_interfaces lidar_outline \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
