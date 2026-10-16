#!/usr/bin/env bash
# Start the saved-map semantic navigation stack for one completed SceneSplat run.
# A query starts the five-second candidate choice window; route safety is rechecked.
set -eo pipefail

usage() {
  echo "Usage: bash $0 /absolute/MCMC_RUN /absolute/MAP.yaml [--drive] [--no-rviz]" >&2
  echo "Default Nav2 output is disconnected. --drive connects it to /cmd_vel." >&2
}
if (( $# < 2 )); then usage; exit 2; fi
run=$1; map=$2; shift 2
drive=false; rviz=true
for arg in "$@"; do
  case "$arg" in
    --drive) drive=true ;;
    --no-rviz) rviz=false ;;
    *) usage; exit 2 ;;
  esac
done
[[ "$run" = /* && -d "$run" && "$map" = /* && -f "$map" ]] || {
  echo 'Run and map must be existing absolute paths.' >&2; exit 2;
}
scene=$(basename -- "$run")
root=/home/jovan/scenesplat_runs
for file in "$run/splat.ply" "$run/world_transform.json" \
    "$root/input/mcmc_filtered/$scene/splat.ply" \
    "$root/input/mcmc_filtered/$scene/filter.json" \
    "$root/output/mcmc_filtered/$scene/${scene}_feat.pt"; do
  [[ -s "$file" ]] || { echo "Missing completed-run artifact: $file" >&2; exit 2; }
done
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source /home/jovan/ros2_ws/install/setup.bash
set -u
for node in /navigation_scans /isaac_localization /map_server /planner_server \
    /controller_server /behavior_server /bt_navigator /find_object_server \
    /semantic_navigation; do
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    echo "Already running: $node. Stop the old stack before starting this one." >&2
    exit 2
  fi
done
if ! timeout 20 ros2 topic echo /clock --once >/dev/null 2>&1; then
  echo 'No /clock message arrived within 20 seconds. Check Isaac Play and that this terminal uses the same ROS_DOMAIN_ID as Isaac.' >&2
  ros2 topic info /clock >&2 || true
  exit 2
fi
if [[ "$drive" == true ]]; then output=/cmd_vel; else output=/nav/cmd_vel_preview; fi
log_dir=$(mktemp -d /tmp/semantic-navigation.XXXXXX)
pids=()
cleanup() {
  trap - INT TERM EXIT
  for pid in "${pids[@]}"; do kill -- "-$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
  echo "Stopped launcher processes. Logs: $log_dir"
}
trap cleanup INT TERM EXIT
start() {
  local name=$1; shift
  setsid "$@" >"$log_dir/$name.log" 2>&1 &
  pids+=("$!")
  echo "$name: $log_dir/$name.log"
}
wait_node() {
  local node=$1 pid=$2
  for ((i=0; i<60; i++)); do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then return 0; fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  echo "Failed to start $node; inspect logs in $log_dir" >&2
  return 1
}
start scans ros2 run lidar_outline navigation_scans --ros-args -p reverse_ranges:=true
wait_node /navigation_scans "${pids[-1]}"
start localization ros2 run lidar_outline isaac_localization --ros-args -p source_base_frame:=summit_xl_base_link
wait_node /isaac_localization "${pids[-1]}"
start nav2 ros2 launch lidar_outline navigation.launch.py "map:=$map" \
  'footprint:=[[-0.42,-0.30],[0.53,-0.30],[0.53,0.30],[-0.42,0.30]]' \
  "cmd_vel_topic:=$output"
wait_node /controller_server "${pids[-1]}"
for ((i=0; i<60; i++)); do
  if ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active \[3\]'; then break; fi
  kill -0 "${pids[-1]}" 2>/dev/null || { echo 'Nav2 exited.' >&2; exit 1; }
  sleep 1
done
ros2 lifecycle get /controller_server | grep -q 'active \[3\]' || {
  echo "Nav2 did not activate; inspect $log_dir/nav2.log" >&2; exit 1;
}
ros2 param set /controller_server FollowPath.desired_linear_vel 1.5
ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel 2.0
ros2 param set /controller_server FollowPath.max_angular_accel 4.0
start query bash "$script_dir/find_object.sh" "$run"
wait_node /find_object_server "${pids[-1]}"
for ((i=0; i<180; i++)); do
  if ros2 service list 2>/dev/null | grep -Fxq /find_object; then break; fi
  kill -0 "${pids[-1]}" 2>/dev/null || { echo "Query failed; inspect $log_dir/query.log" >&2; exit 1; }
  sleep 1
done
ros2 service list | grep -Fxq /find_object || { echo 'Query service was not ready.' >&2; exit 1; }
start bridge ros2 run lidar_outline semantic_navigation --ros-args -p use_sim_time:=true -p clearance:=0.2
wait_node /semantic_navigation "${pids[-1]}"
if [[ "$rviz" == true ]]; then
  if ros2 node list 2>/dev/null | grep -Eq '^/rviz([0-9]*)?$'; then
    echo 'Using existing RViz.'
  else
    start rviz rviz2 --ros-args -p use_sim_time:=true
  fi
fi
echo "Ready. Nav2 output: $output"
echo "Query: bash '$script_dir/query_semantic_object.sh' 'object name'"
echo 'In RViz: if several boxes appear, click a different one within five seconds. One box navigates immediately; otherwise #1 is the default. A route is checked before motion.'
echo 'Press Ctrl-C here to stop the processes started by this launcher.'
wait
