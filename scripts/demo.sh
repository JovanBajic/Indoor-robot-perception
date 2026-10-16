#!/usr/bin/env bash
# One-name demo workflow: record/map, reconstruct/view, then query/navigation.
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "$script_dir/.." && pwd)
state_dir="$HOME/room_captures/.lidar_outline_demo"
state_file="$state_dir/current"

usage() {
  echo "Usage: bash $0 {record|pipeline|navigate|query 'object'}" >&2
  exit 2
}

load_state() {
  [[ -f "$state_file" ]] || { echo 'No completed demo recording. Run demo.sh record first.' >&2; exit 2; }
  mapfile -t session < "$state_file"
  capture=${session[0]:-}
  map_yaml=${session[1]:-}
  run=${session[2]:-}
  [[ -d "$capture" && -f "$map_yaml" ]] || {
    echo 'Saved demo capture or map is missing.' >&2; exit 2;
  }
}

save_state() {
  mkdir -p "$state_dir"
  local staged
  staged=$(mktemp "$state_dir/current.XXXXXX")
  printf '%s\n%s\n%s\n' "$capture" "$map_yaml" "${run:-}" > "$staged"
  mv -f -- "$staged" "$state_file"
}

start() {
  setsid "$@" &
  pids+=("$!")
}

stop_started() {
  for pid in "${pids[@]}"; do kill -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}

record() {
  source /opt/ros/jazzy/setup.bash
  source /home/jovan/ros2_ws/install/setup.bash
  set -u
  if ! timeout 20 ros2 topic echo /clock --once >/dev/null 2>&1; then
    echo 'Start Isaac Play, then run this command again.' >&2; exit 2
  fi
  for node in /navigation_scans /world_map /map_server /planner_server \
      /d455_recorder /d455_01_recorder /d455_02_recorder \
      /d455_03_recorder /d455_04_recorder /d455_05_recorder; do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      echo "Already running: $node. Stop the old stack first." >&2; exit 2
    fi
  done
  local stamp map_base recorder_pid
  stamp=$(date -u +%Y%m%dT%H%M%S_%6NZ)
  capture="$HOME/room_captures/capture_$stamp"
  map_base="$project_dir/maps/demo_$stamp"
  map_yaml="$map_base.yaml"
  run=''
  [[ ! -e "$capture" && ! -e "$map_yaml" ]] || { echo 'Generated demo name already exists.' >&2; exit 2; }
  pids=()
  trap 'stop_started' EXIT
  echo "Capture: $capture"
  echo "Map: $map_yaml"
  start ros2 run lidar_outline navigation_scans --ros-args -p reverse_ranges:=true
  start ros2 run lidar_outline world_map
  start rviz2 -d "$project_dir/config/mapping.rviz" --ros-args -p use_sim_time:=true
  start ros2 run lidar_outline capture_camera --session-name "$(basename -- "$capture")" \
    --ros-args -p save_depth:=true -p max_fps:=4.0 \
    -p min_translation:=0.025 -p min_rotation_deg:=2.5
  recorder_pid=${pids[-1]}
  echo 'Drive manually. Press Ctrl-C here while Isaac is still playing to stop recording and save the map.'
  local recording_done=false
  trap 'recording_done=true' INT
  wait "$recorder_pid" || true
  trap - INT
  [[ "$recording_done" == true ]] || {
    echo 'Recorder stopped before Ctrl-C; inspect its error. Map was not saved.' >&2; exit 1;
  }
  kill -- "-$recorder_pid" 2>/dev/null || true
  wait "$recorder_pid" 2>/dev/null || true
  [[ -d "$capture" ]] || { echo 'Recorder did not create its capture folder.' >&2; exit 1; }
  echo 'Saving /nav/map while Isaac and world_map are still running...'
  ros2 run nav2_map_server map_saver_cli -f "$map_base" --ros-args \
    -p use_sim_time:=true -p map_subscribe_transient_local:=true \
    -p save_map_timeout:=10.0 -r map:=/nav/map
  [[ -s "$map_yaml" && -s "$map_base.pgm" ]] || {
    echo 'Map files were not saved; keep Isaac playing and save manually.' >&2; exit 1;
  }
  save_state
  echo "Saved demo capture and map. Next: bash '$script_dir/demo.sh' pipeline"
}

pipeline() {
  load_state
  if [[ -z "$run" ]]; then
    run="$HOME/mcmc_runs/$(basename -- "$capture")_depth_$(date -u +%Y%m%dT%H%M%S_%6NZ)"
    save_state
  fi
  echo "Capture: $capture"
  echo "MCMC run: $run"
  bash "$script_dir/run_room_pipeline.sh" "$capture" --run "$run" --no-view
  echo "Features complete. Reopen later with: bash '$script_dir/view_scenesplat_mcmc.sh' '$run'"
  bash "$script_dir/view_scenesplat_mcmc.sh" "$run"
}

navigate() {
  load_state
  [[ -n "$run" ]] || { echo 'No MCMC run recorded. Run demo.sh pipeline first.' >&2; exit 2; }
  echo "Using map: $map_yaml"
  echo "Using run: $run"
  exec bash "$script_dir/start_semantic_navigation.sh" "$run" "$map_yaml" --drive
}

case "${1:-}" in
  record) (( $# == 1 )) || usage; record ;;
  pipeline) (( $# == 1 )) || usage; pipeline ;;
  navigate) (( $# == 1 )) || usage; navigate ;;
  query) (( $# == 2 )) || usage; exec bash "$script_dir/query_semantic_object.sh" "$2" ;;
  *) usage ;;
esac
