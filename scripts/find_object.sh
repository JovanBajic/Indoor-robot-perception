#!/usr/bin/env bash
# Start the ROS /find_object service for a selected completed MCMC scene. Its isolated
# worker loads matching features and cached SigLIP2 for text queries.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/jovan/ros2_ws/install/setup.bash
set -u
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -eq 0 ]]; then
  echo "Usage: bash $0 /home/jovan/mcmc_runs/RUN_FOLDER [--threshold 0.75 --eps 0.15 ...]" >&2
  exit 1
fi
exec /usr/bin/python3 "$script_dir/../lidar_outline/find_object_node.py" "$@" \
  --worker "$script_dir/object_query_worker.py"
