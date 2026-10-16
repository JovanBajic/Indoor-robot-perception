#!/usr/bin/env bash
# Submit a text query after start_semantic_navigation.sh reports ready.
set -eo pipefail
if (( $# != 1 )) || [[ -z "$1" ]]; then
  echo "Usage: bash $0 'object description'" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source /home/jovan/ros2_ws/install/setup.bash
set -u
query=$(python3 -c 'import json,sys; print(json.dumps({"query":sys.argv[1]}))' "$1")
exec ros2 service call /find_object lidar_outline_interfaces/srv/FindObject "$query"
