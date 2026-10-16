# RGB/depth → MCMC → SceneSplat → object queries

All commands run in a terminal unless stated otherwise. Copy only commands
inside the code blocks, without the triple backticks. Replace the capture/run
path when using another recording; the examples below use verified existing data.

## Fast demo commands — generated names

Open the matching six-camera Isaac scene and press **Play**. With no old mapping
or Nav2 stack running, start the combined recorder, corrected scans, map collector
and RViz:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh record
```

Drive manually. Press **Ctrl-C in this terminal while Isaac is still playing**:
the script stops the recorder, saves `/nav/map`, and remembers the generated
capture and map filenames. Wait for `Saved demo capture and map` before stopping
Isaac. Close Isaac and the viewer for GPU memory, then run:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh pipeline
```

This creates or reuses the matching MCMC run, extracts SceneSplat features and
opens its viewer. Close the viewer, reopen the same Isaac room and press **Play**.
Then run:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh navigate
```

When it says **Ready**, query in another terminal:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh query plant
```

The demo wrapper stores the exact generated capture, map and run paths in
`~/room_captures/.lidar_outline_demo/current`; it does not pick the newest file
from unrelated recordings. `navigate` explicitly connects Nav2 to `/cmd_vel`.
After a query, one candidate starts route checks immediately and navigates if
reachable. With several candidates, click another numbered RViz box within five
seconds; otherwise #1 is used. The selected route is previewed and checked
against a fresh costmap and robot footprint before a Nav2 goal is sent. Cancel
through `/semantic_navigation/cancel` if needed; pause Isaac for an immediate stop.
The original commands below remain available for individual stages.

## Current status — 2026-10-13

**Drive handoff:** Open `/home/jovan/Desktop/rbkairos_franka_project/trial.usd`
for any further drive work. Its Differential Controller `wheelDistance` was set
to 1.0 in the open Isaac scene but was **not saved**; verify the value after
reopening. The original `6_cameras_rb_kairos_new.usd` was not changed for this
calibration. The temporary Nav2 stack, scan adapter, localization bridge, and
command relay were stopped; `/cmd_vel` had zero ROS publishers at the final
check. The next test needs a footprint-clear goal. The apparent heading
overshoot in the latest trial was Nav2's collision-recovery Spin, not proof of
drive overshoot. Do not reuse the tested goal (-1.17, 0.32).

### Drive response check — 2026-10-13

- Bounded direct `/cmd_vel` turn tests used Isaac measured `/nav/odom` and
  `/joint_states`, then published zero velocity. The 0.8 rad/s command produced
  about 0.266 rad/s measured body yaw and about 1.5 rad/s opposite-side wheel
  speeds. A 0.4 rad/s repeat produced about 0.093 rad/s yaw. The first 0.4
  attempt nearly stalled. These rates use simulation time; playback itself ran
  at about 0.34 real-time factor.
- Connecting Playback Tick Delta Seconds to Differential Controller `dt` did
  not materially change the measured turn response. Controller wheel radius
  0.11 m and wheel distance 0.409 m match four 0.11 m cylinder colliders and
  their 0.409 m lateral spacing. All four wheel joints reached commanded
  speeds, so the remaining yaw loss is between wheel motion and base motion.
- A read-only Isaac inspection script, `scripts/inspect_drive_physics.py`,
  writes `/tmp/isaac-drive-physics.json`. It found two enabled ground collision
  planes and no resolved wheel/ground physics material. Temporarily disabling
  one ground plane worsened the 0.4 rad/s turn and caused roughly 0.069 m base
  drift. The user restored its collision setting; do not treat duplicate
  planes as a confirmed cause. Plane orientations were not audited.
- With all D455 RGB/depth render products disabled, measured camera messages
  stopped, but the real-time factor stayed about 0.34 and the 0.8 rad/s turn
  stayed at about 0.266 rad/s. Camera publishing load did not explain the
  drive mismatch. Re-enable camera render products before recording/query work.
- A subsequent bounded straight test in mapped free space commanded 0.1 m/s:
  all four wheels reached about 0.907 rad/s, matching a 0.11 m radius, and
  measured body forward speed reached 0.1003 m/s with negligible yaw. The
  0.224 m translation over the command interval confirms straight drive
  tracks; the large response error is specific to turning, consistent with
  four-wheel skid-steer scrub/contact rather than a generic `/cmd_vel` cap.
- Extended read-only audit measured both ground plane normals upward at z=0.
  Wheel cylinder lower bounds were roughly 0.0005–0.0018 m above the plane
  at rest; non-wheel collision boxes were higher. This found no chassis-floor
  scrape. The normal for one plane is scaled by its transform; the planes were
  not proven to be the cause, and disabling one worsened motion.
- Trial copy `/home/jovan/Desktop/rbkairos_franka_project/trial.usd` was made
  for drive changes. Temporary friction tests at 0.2 static/dynamic on wheels
  and then both ground planes raised peak yaw at a 0.8 rad/s command from
  0.266 to 0.286 and 0.307 rad/s respectively, while 0.1 m/s straight motion
  remained accurate. These edits were not saved to the original scene.
- After reopening the trial from disk to discard friction changes, its
  Differential Controller `wheelDistance` was changed from 0.409 to 0.8 for
  an effective skid-steer calibration. Bounded direct commands yielded
  0.256 rad/s measured yaw at 0.4 commanded and about 0.60 rad/s at 0.8
  commanded; 0.1 m/s straight still measured 0.1003 m/s with negligible yaw.
  Trial value is not the physical wheel spacing and is not yet validated on a
  Nav2 route or endpoint. Do not silently copy it to the original scene.
- Increasing the trial-only effective `wheelDistance` to 1.0 gave peak measured
  yaw about +0.340 rad/s for +0.4 commanded, +0.755 for +0.8, and -0.326 for
  -0.4. The +0.8 turn shifted the base about 0.047 m during the short test;
  the -0.4 turn shifted it about 0.044 m. This is closer to the requested
  turn rate but shows direction/speed dependence and positional drift. Keep
  1.0 as an unsaved trial until a short route and endpoint are checked.
- At `wheelDistance` 1.0, a repeated bounded 0.1 m/s straight command measured
  0.1003 m/s with negligible yaw. Nav2 was started with output disconnected and
  planned a short route to (-1.17, 0.32), but its planner accepted a goal whose
  full robot footprint intersects the inflated static map (16 sampled edge
  points at cost 253 in the local costmap; the live obstacle layer was clear).
  A bounded relay test advanced from y=-0.17 to about y=0.10, then Regulated
  Pure Pursuit repeatedly reported `detected collision ahead`. Nav2 invoked its
  1.57 rad recovery Spin, which carried the robot past the requested heading;
  the robot then turned back. The goal was canceled at the 50 s wall-time limit
  with final pose about (-1.247, 0.142, 2.008 rad), still 0.194 m from the
  requested position. This does not validate endpoint convergence. Select a
  footprint-clear goal and inspect static-map alignment/clearance before the
  next route test; do not attribute this overshoot to wheelDistance alone.
- The test command publisher, temporary localization bridge, scan adapter,
  and Nav2 stack were stopped. No Nav2 goal reached success. Next check map
  alignment and the full footprint at a clear goal, then retest endpoint
  convergence before changing Nav2 goal tuning or the wheel configuration.

- Changed-room SceneSplat query run is
  `~/mcmc_runs/capture_20261012T114946_176800Z_depth_20261012T141242_524363`;
  its successful query artifact was verified on disk.
- Latest saved map: `maps/room_redo_20261012T141102.yaml` and matching PGM.
  User confirmed saved map and live scans display together after RViz clock/QoS fixes.
- User now confirms the manual Nav2 startup, goal driving and speed-change
  workflow works. Measured footprint and planning were checked; a monitored drive
  achieved about 0.2m/s, but timed out before goal success. User confirmation does
  not establish comprehensive route coverage or semantic navigation.
- See the restart checklist below for the current startup sequence. The semantic
  bridge previews and sends approved goals, but final turning/slip remain under
  investigation; do not treat autonomous goal completion as validated.
- Localization uses Isaac measured pose through `/nav/odom`; AMCL is not configured.
- Recording is separate from the pipeline. Do not start training just to view maps.

## Restart checklist — saved map and SceneSplat navigation

### Short path after SceneSplat finishes

Use the **exact completed MCMC run** whose SceneSplat features match the saved
map. Open the matching Isaac scene and press **Play** first. Stop any old Nav2,
scan, localization, query, or semantic bridge processes. Then run one command:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/start_semantic_navigation.sh \
  /absolute/path/to/COMPLETED_MCMC_RUN \
  /absolute/path/to/MATCHING_MAP.yaml --drive
```

`--drive` explicitly connects Nav2 to `/cmd_vel`. Omit it to preview with
disconnected output. The launcher checks the run artifacts, map, Isaac clock and
duplicate nodes; starts scans, localization, Nav2, query service, bridge and
RViz; and sets requested Nav2 speeds to 1.5 m/s forward and 2.0 rad/s heading.
It does not issue an object
query or a navigation goal. It prints log paths. Keep this terminal open; press
**Ctrl-C** to stop the processes it started.

When it says **Ready**, query in another terminal:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/query_semantic_object.sh 'plant'
```

In RViz set Fixed Frame `world`, add the InteractiveMarkers topic
`/semantic_navigation/candidates/update` and Path topic
`/semantic_navigation/preview_path` if they are not already displayed. Choose
**Interact**. With multiple boxes, click a different candidate within five
seconds; otherwise green #1 is selected. With one box, route checking begins
immediately. A fresh route/footprint check precedes the Nav2 goal. Wheel slip
and final heading remain unverified; watch the robot during motion. The detailed
manual commands below remain available for diagnosis.

The ROS packages were rebuilt after the semantic route-check fix; **no rebuild is
needed just to restart**. Open
`/home/jovan/Desktop/rbkairos_franka_project/6_cameras_rb_kairos_new.usd` in
Isaac Sim and press **Play**. Stop any old navigation goal and avoid duplicate
`navigation_scans`, `isaac_localization`, Nav2, `/find_object`, or
`semantic_navigation` processes. Keep WASD teleop stopped while Nav2 controls
the base. Run each long-running command below in its own terminal, in order.

1. Correct the front and rear laser scan order:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 run lidar_outline navigation_scans --ros-args -p reverse_ranges:=true
   ```

2. Publish measured Isaac pose and odometry:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 run lidar_outline isaac_localization --ros-args -p source_base_frame:=summit_xl_base_link
   ```

3. Start one Nav2 stack with the saved map and measured robot footprint:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch lidar_outline navigation.launch.py \
     map:="$HOME/ros2_ws/src/lidar_outline/maps/room_redo_20261012T141102.yaml" \
     footprint:='[[-0.42,-0.30],[0.53,-0.30],[0.53,0.30],[-0.42,0.30]]' \
     cmd_vel_topic:=/cmd_vel
   ```

   This launch starts its own map server. Wait for the lifecycle manager to
   activate Nav2. In another sourced terminal, check `ros2 lifecycle get
   /controller_server` returns `active [3]`. The installed config now requests
   `1.5` m/s forward and `2.0` rad/s heading rotation. To use the slower
   settings from earlier testing, apply these **before sending a goal**:

   ```bash
   ros2 param set /controller_server FollowPath.desired_linear_vel 0.2
   ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel 0.4
   ros2 param set /controller_server FollowPath.max_angular_accel 4.0
   ```

4. Start the object-query service on the verified changed-room run:

   ```bash
   bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh \
     ~/mcmc_runs/capture_20261012T114946_176800Z_depth_20261012T141242_524363
   ```

   Wait for `/find_object` to become available. Loading the model can take time;
   this reuses trained features and does not retrain SceneSplat.

5. Start the clickable semantic bridge:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 run lidar_outline semantic_navigation --ros-args -p use_sim_time:=true -p clearance:=0.2
   ```

   It searches farther stopping poses up to `max_clearance:=1.2` if needed.
   This process must be restarted to load source changes; the other running
   nodes need not be restarted for a bridge-only change.

6. Start RViz in another sourced terminal:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   rviz2 --ros-args -p use_sim_time:=true
   ```

   Set Fixed Frame to `world`. Add Map `/map` with **Reliable / Transient Local**;
   Path `/semantic_navigation/preview_path`; Polygon
   `/local_costmap/published_footprint`; TF; and InteractiveMarkers with update
   topic `/semantic_navigation/candidates/update`. Optional scans are
   `/nav/front_scan` and `/nav/rear_scan` with **Best Effort / Volatile**. If an
   old RViz config opens a blank map, change its Map topic from `/nav/map` to
   `/map`. Section 10 has the click-by-click display setup.

7. Query an object in a new sourced terminal, for example:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 service call /find_object lidar_outline_interfaces/srv/FindObject "{query: 'plant'}"
   ```

   In RViz, choose **Interact**, left-click a numbered box and wait for the
   preview. This sends no motion. Right-click the same box and choose **Approve
   selected approach and navigate** only when the preview is suitable. To cancel
   an active goal or clear the preview, right-click **Cancel navigation / clear
   preview**, or run:

   ```bash
   ros2 service call /semantic_navigation/cancel std_srvs/srv/Trigger '{}'
   ```

   Wait for cancellation to finish before clicking another box. The current
   differential-drive setup has shown wheel slip and imperfect final turns;
   omnidirectional control has not been configured. Pause Isaac for an immediate
   stop, then cancel the goal before resuming Play.

## 1. Record RGB and depth

Open the saved six-camera scene in Isaac and press **Play**. Start the recorder:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline capture_camera --ros-args -p save_depth:=true -p max_fps:=4.0 -p min_translation:=0.025 -p min_rotation_deg:=2.5
```

Always pass `save_depth:=true` explicitly. Source changes need a package rebuild
to affect `ros2 run`. Each recording is saved under `~/room_captures/capture_...`.

In a second sourced ROS terminal, start keyboard driving:

```bash
ros2 run lidar_outline wasd_teleop --speed 0.3 --turn 2.0
```

Keep the WASD window focused. Drive slowly with overlapping views. After a few
frames, check that the new capture has `depth/*.npy` as well as `rgb/*.png` before
completing the drive. The cameras capture independently, not in six-camera bundles.
Press **Ctrl+C in the recorder before stopping Isaac playback**.

## 2. Run the complete pipeline

Close Isaac and viewers to free GPU memory. Select your exact recording folder:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_room_pipeline.sh ~/room_captures/capture_20261009T110859_039769Z
```

This command starts **new training**. It checks RGB/depth pairs, calibration and
poses, trains depth-initialized MCMC, exports PLY, preprocesses/filters it,
extracts SceneSplat features, and starts http://127.0.0.1:8083.
Enable **Feature Map** in the browser. The recorded dataset is read directly;
no separate dataset conversion is needed.

Defaults: 100,000 colored depth seeds, valid axial-Z depth 0.1–10m, 30,000 steps,
640x360 images, up to 1,000,000 Gaussians and fixed measured poses. Depth initializes
geometry; the optimization loss remains RGB-only. Every eighth view is held out.
Training progress prints to the terminal and saves beside the run as
`<run-name>.pipeline.log`. The output folder is printed at startup.
`validation.png` updates every 5,000 steps: original left, reconstruction right.
Ctrl+C preserves completed outputs; automatic checkpoint resume is unavailable.

Useful flags:

- `--check-only`: validate the capture without training.
- `--no-view`: finish after feature extraction.
- `--port 8084`: choose another viewer port. A busy port causes an early error.
- `--run /absolute/run/folder`: use that output folder; reuse a completed matching
  depth-initialized run, or start training if the folder does not exist.

The `splatfacto` Conda environment supplies working CUDA/Torch/gsplat for MCMC;
its name does not select standard Splatfacto training. SceneSplat inference and
viewing use the separate `scene_splat` environment. Neither environment nor
SceneSplat source is changed by these launchers.

## 3. Reuse a completed run

This command skips finished training and existing feature extraction, then views:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_room_pipeline.sh ~/room_captures/capture_20261009T110859_039769Z --run ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

For viewing only, no capture validation or extraction is needed:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat_mcmc.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

Open http://127.0.0.1:8083. Stop the viewer with Ctrl+C before reusing its port.
To view RGB reconstruction only, without features:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/view_mcmc.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

Open http://127.0.0.1:8082. These viewers show the explicitly selected run.

## 4. Individual training and extraction commands

Use these only when running stages manually instead of the pipeline:

```bash
CAPTURE="$HOME/room_captures/capture_20261009T110859_039769Z"
RUN="$HOME/mcmc_runs/$(basename -- "$CAPTURE")_depth_$(date +%Y%m%dT%H%M%S)"
echo "Training output: $RUN"
MCMC_INITIALIZATION=depth bash ~/ros2_ws/src/lidar_outline/scripts/train_six_cameras_mcmc.sh "$CAPTURE" "$RUN"
```

Wait for `Finished: .../splat.ply`, then in the same terminal:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_scenesplat_mcmc.sh "$RUN"
bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat_mcmc.sh "$RUN"
```

Skip extraction when matching features already exist. Always pass the capture
and run explicitly; standalone launchers retain historical defaults.

The SceneSplat helper creates separate filtered inputs: trajectory bounds plus
5m on each side, opacity at least 1/255. Original PLY stays intact. Feature rows
match the filtered PLY; `source_indices.npy` maps them back to the original.
MCMC/filtered coordinates remain recorded `world` metres. Do not apply the
archived Splatfacto normalization conversion to them.

## 5. External object queries

SceneSplat remains unchanged. Start the separate service against a run with
completed matching features:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

Wait for `/find_object ready`. In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /find_object lidar_outline_interfaces/srv/FindObject "{query: 'table'}"
```

The service uses cached SigLIP2 text embeddings and saved Gaussian features,
then threshold/top-k selection and DBSCAN. It returns the cluster with highest
mean raw cosine (point count breaks ties): centroid, bbox min/max, counts,
score, frame `world` and result-file path. JSON/NPZ results are saved under the
run's `object_queries/`; NPZ records filtered and original PLY rows.

Defaults: relative threshold 0.75, top 10,000 points, DBSCAN radius 0.15m,
minimum 5 nearby samples and 20 points per cluster. `found` reports a spatial
candidate, not verified object identity. The bbox encloses selected Gaussian
centers and can represent only part of an object. Scores are not probabilities.

To adjust clustering, stop the service and relaunch with, for example:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838 --threshold 0.8 --eps 0.12 --top-k 10000
```

`--score-mode cosine` uses raw cosine thresholds instead of per-query min/max
scaling. `--device cpu` frees GPU memory at the cost of slower queries.
The ROS adapter uses system Jazzy Python; the persistent worker uses the
existing `scene_splat` environment. Stop the service with Ctrl+C.

## 6. Show a query box in RViz

Isaac does not need to run for a saved-scene query. Start RViz in a sourced terminal:

```bash
rviz2
```

Set **Global Options → Fixed Frame** to `world`. Click **Add → By display type →
MarkerArray → OK**, then set its **Topic** to `/object_query/markers`.
Send the query after adding the display. A translucent orange box appears for
an object candidate. This does not annotate the open Isaac scene or move the robot.

Nav2 is not connected yet. The intended next stage is object candidate → reachable
approach pose beside the object → Nav2 → existing `/cmd_vel` drive. It also needs
robot pose, a navigation map and obstacle handling.

## 7. Rebuild after ROS package/interface changes

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/build_object_query.sh
```

This builds `lidar_outline` and its nested `lidar_outline_interfaces` package.
Standalone Python/shell script and documentation edits need no ROS rebuild.

## Archived workflows

Standard Splatfacto training/export, its normalized bbox conversion, old viewer
launchers and obsolete TF setup/recovery scripts are under `archive/`.
See [archive/README.md](archive/README.md). Previous recordings, reconstructions
and environments are preserved. Working LiDAR outline and driving code remain active.

## 8. Localization and Nav2 setup in progress

Latest 2026-10-12: live audit confirmed descending physical ray order and timed
partial scans. The external adapter reverses samples and discards ambiguous
maximum-range entries, preventing inactive rays from clearing walls. It leaves
the raw LiDAR topics and Isaac graphs untouched. After rebuilding, run these in
separate sourced ROS terminals while Isaac is playing:

```bash
ros2 run lidar_outline navigation_scans --ros-args -p reverse_ranges:=true
ros2 run lidar_outline world_map
rviz2 -d ~/ros2_ws/src/lidar_outline/config/mapping.rviz --ros-args -p use_sim_time:=true
```

The map uses both corrected scans and full timestamped TF in `world`, at 5cm
resolution, with bounds x/y -20..20m. Unknown space stays unknown. A simulation
clock reset clears the collected map; keep Play running during a mapping pass.
Maximum-range readings are discarded conservatively because ROS provides no
active-ray mask; free space is established only along finite hit rays.

After manually driving the room, save to a **new** filename:

```bash
ros2 run nav2_map_server map_saver_cli -f /absolute/path/to/new_room_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 -r map:=/nav/map
```

Draft Nav2 launch/config is installed and all five lifecycle servers activated
in a live bringup check. No goal/driving test has been completed. It requires an
inspected map and measured footprint; the default command topic is disconnected
from Isaac. Preview command (replace paths/polygon with verified values):

```bash
ros2 launch lidar_outline navigation.launch.py map:=/absolute/path/to/verified_map.yaml footprint:='[[-0.45,-0.4],[0.45,-0.4],[0.45,0.4],[-0.45,0.4]]'
```

That example polygon is **provisional**, not a verified live footprint. Run the
audit below again to obtain `robot_bounds_in_base`. Keep WASD stopped for Nav2
drive tests. Only after map/footprint/planning checks, `cmd_vel_topic:=/cmd_vel`
connects controller/behavior output to the existing drive. Default limits are
0.2m/s desired forward speed and0.4rad/s heading rotation. Semantic navigation
is not connected. Synthetic clock-reset, scan-order, map and planar tests:9 pass.

### View the saved changed-room map

Only map_server and RViz are required to view a saved map. Stop any old
map_server first. In a sourced ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$HOME/ros2_ws/src/lidar_outline/maps/room_redo_20261012T141102.yaml" -p frame_id:=world
```

In another sourced terminal:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
rviz2 --ros-args -p use_sim_time:=true
```

RViz: Fixed Frame `world`; Add → By topic → /map → Map. Expand Topic:
Reliability `Reliable`, Durability `Transient Local`. The YAML path must include
`maps/`; omitting it caused a bad-file configure failure. Restart map_server
with the correct path if configuration fails.

For live alignment checks, also keep Isaac playing and navigation_scans running.
Add LaserScan displays on `/nav/front_scan` and `/nav/rear_scan`, with Reliability
`Best Effort` and Durability `Volatile`. Size 0.05m; Color Transformer `FlatColor`
then choose distinct colors. Disable /lidar_lines markers if they obscure points.
The saved map is `/map`; `/nav/map` is the active collector, which is unnecessary
for viewing saved maps. Compare laser hits with black obstacle edges while driving
slowly and turning at several positions. Display success alone is not validation.

If RViz reports 0 points from 0 messages, check Play, adapter, scan QoS and clock.
Blank transformer choices can accompany no received scan data. Live checks found
scans arriving but RViz using system time. Switch an existing RViz:

```bash
ros2 param set /rviz use_sim_time true
```

Toggle LaserScan and Map displays off/on after changing the clock; retain the
map's Transient Local durability. User confirmed all displays working afterward.

### Repeat mapping and recording safely

`world_map` collects the map in memory; `navigation_scans` corrects readings;
`capture_camera` records independently. Restart world_map to begin a fresh pass.
Keep the adapter running. Both mapping and RGB/depth recording can run during
one slow drive (suggested teleop `--speed 0.2 --turn 0.5`). In RViz use `/nav/map`
while collecting. Stop recorder before stopping Play. Save the map with the
command above while world_map and Play are still running, wait for successful
save, then stop the collector. Stopping it before saving loses the in-memory
map; the camera recording remains usable without rerecording. Use new filenames.

The changed-room SceneSplat run is user-reported working; obtain its exact run
folder before object queries. The pipeline in section 2 starts new MCMC training;
do not launch it when only inspecting a saved map or existing reconstruction.

### Localization and next navigation checks

Nav2 is installed. Start the measured-pose bridge if needed:

```bash
ros2 run lidar_outline isaac_localization --ros-args -p source_base_frame:=summit_xl_base_link
```

For a fresh live footprint audit, run in Isaac's Script Editor:

```python
exec(open('/home/jovan/ros2_ws/src/lidar_outline/scripts/inspect_live_lasers.py').read())
```

Inspect `/tmp/isaac-live-lasers.json`, including `robot_bounds_in_base`. Next:
verify scan/map alignment, measured velocity and footprint, then preview plans
with drive disconnected and test a short ordinary goal after connecting drive.
AMCL would add laser/map-based localization; it is not currently configured.

The `isaac_localization` ROS node is built and tested with synthetic ROS data.
Its required `source_base_frame` parameter must name the verified live base TF
frame. It publishes `/nav/odom` and `world → nav_odom → nav_base` using measured
Isaac poses and finite-difference body velocities, leaving existing TF parents
unchanged. This is simulator ground-truth localization, not wheel odometry or
AMCL. Stale poses are rejected, simulation resets clear velocity history, and
the node never publishes motion commands.

Verification already completed: three planar pose/velocity tests and an isolated
ROS-domain test producing 104 synthetic odometry messages. This does not verify
the live robot or its TF source frame.

The initial preparation checklist above is superseded by the live checks and
user-confirmed workflow in section 9. Preserve disabled D455 IMUs and the
existing Isaac drive, camera and LiDAR graphs. AMCL and semantic approach-goal
integration remain pending.

### Live controller diagnosis — 2026-10-12

Planning succeeded; first drive test stalled during initial heading rotation.
At 10Hz, max_angular_accel 0.6 limited initial turn command to 0.06rad/s, which
produced no measurable rotation. Live tests with 2.0 (.2rad/s initial command)
produced minimal rotation; 4.0 (.4rad/s initial command) produced about .088rad/s
measured rotation. Source config now retains 4.0, with heading speed cap .4rad/s.
The running controller was updated live; rebuild before relying on installed
config at restart. Actual response remains slower than commanded, and Isaac
runs below real time. These monitored tests did not reach goal success; later
user-operated navigation was reported working (section 9). Tests used a temporary relay,
canceled on timeout, then sent zero velocity and disconnected from /cmd_vel.
Next: verify a short goal completes and assess drive/odometry response before
longer routes. Keep WASD stopped during autonomous tests.

## 9. Start navigation and change speed — user-confirmed 2026-10-12

User confirmed this workflow works. Commands are for the user to run; do not
start or drive automatically when asked only for instructions. Use the measured
footprint below with the current robot configuration. Reaudit if geometry or
arm configuration changes. Localization uses Isaac measured TF, not AMCL.

### Start everything

Open the changed six-camera Isaac scene and press Play. Keep WASD teleop stopped.
Run each long-running command in its own terminal. Do not start duplicate nodes;
reuse an existing scan adapter/localization bridge if already running.

Terminal 1 — corrected scans:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline navigation_scans --ros-args -p reverse_ranges:=true
```

Terminal 2 — measured-pose localization:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline isaac_localization --ros-args -p source_base_frame:=summit_xl_base_link
```

Terminal 3 — navigation and saved map, connected to the drive:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_outline navigation.launch.py \
  map:="$HOME/ros2_ws/src/lidar_outline/maps/room_redo_20261012T141102.yaml" \
  footprint:='[[-0.42,-0.30],[0.53,-0.30],[0.53,0.30],[-0.42,0.30]]' \
  cmd_vel_topic:=/cmd_vel
```

This launch includes map_server; do not start a separate one. Without the
cmd_vel_topic argument, launch defaults to disconnected /nav/cmd_vel_preview.

Terminal 4 — check activation and apply the tested turning adjustment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 lifecycle get /controller_server
```

Wait for `active [3]`, then:

```bash
ros2 param set /controller_server FollowPath.max_angular_accel 4.0
```

Terminal 5 — visualization:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

RViz Fixed Frame: world. Add these using Add → By topic, except TF via
Add → By display type:

| Display | Topic | Settings |
| --- | --- | --- |
| Map | /map | Reliable, Transient Local |
| Path | /plan | Add before requesting the goal |
| Polygon | /local_costmap/published_footprint | Bright color |
| LaserScan | /nav/front_scan | Best Effort, Volatile, Decay Time 0 |
| LaserScan | /nav/rear_scan | Best Effort, Volatile, Decay Time 0 |
| TF | — | Show nav_base axes; red X points forward |

The polygon is the measured 2D envelope, not a full 3D bounding box. To compare
scans with the map, hide /lidar_lines Marker displays: these may use raw scans.
Check `ros2 param get /navigation_scans reverse_ranges` returns True. The adapter
reverses ray order and discards ambiguous max-range/Inf samples as NaN; it does
not localize, build maps or drive. Explicit launch override above avoids stale
installed defaults. Python source defaults use `True`, not lowercase `true`.

Click 2D Goal Pose. Click clear white space about 0.5m ahead along nav_base's red
axis, hold and drag forward to set heading, then release. Releasing sends the
goal and starts driving when /cmd_vel is connected. Keep Isaac playing. The
controller generates /plan automatically; drawing a path alone is not required.
world_map and capture_camera are unnecessary for navigation on a saved map.

Pause Isaac to stop immediately. Keep it paused until the active navigation
goal is canceled or the Nav2 launch is stopped; Play can resume an active goal.

### Change speed in the config

File: `/home/jovan/ros2_ws/src/lidar_outline/config/nav2_isaac.yaml`.
Under controller_server → ros__parameters → FollowPath, the slower values used
for cautious testing are:

```yaml
desired_linear_vel: 0.2
rotate_to_heading_angular_vel: 0.4
max_angular_accel: 4.0
```

Linear speed is in m/s. Turning speed refers to the whole robot's yaw, in rad/s,
not wheel rotation. Angular acceleration is in rad/s². The controller reduces
forward speed near obstacles, curves and the goal; path-following angular speed
comes from path curvature. The heading setting applies to in-place rotation.
Measured /nav/odom velocity comes from timestamped pose changes, in simulation
time; commanded and measured turn rates can differ substantially.

To persist an edit:

1. Pause Isaac and Ctrl+C only the Nav2 launch terminal. Leave scans,
   localization and RViz running.
2. Edit and save the YAML. Use decimal values for floating-point parameters:
   `0.2`, `1.0`, `4.0`, not integer `1` or `4`. An integer desired_linear_vel
   caused controller configuration to fail with a double/integer type error.
3. Rebuild and source:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select lidar_outline
source install/setup.bash
```

4. Press Play, rerun the navigation launch once, verify active state, then
   select a new nearby goal.

For a temporary live adjustment without rebuilding:

```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 0.2
ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel 0.4
ros2 param set /controller_server FollowPath.max_angular_accel 4.0
```

### Duplicate Nav2 or no movement

Two launches previously left duplicate controller_server/planner_server/
bt_navigator names: the preview published to /nav/cmd_vel_preview while the new
controller was unconfigured. Pause Isaac, Ctrl+C all Nav2 launch terminals and
any standalone map_server. Verify with `ros2 node list` before relaunching once.
If a launch terminal is unavailable:

```bash
pgrep -af 'ros2 launch'
```

Identify the specific navigation launch PID (the temporary preview was
navigation_existing_map.launch.py), then stop it gracefully:

```bash
kill -INT PROCESS_ID
```

Replace PROCESS_ID with the identified PID; do not kill unrelated ROS nodes.
For diagnosis:

```bash
ros2 node info /controller_server
ros2 lifecycle get /controller_server
ros2 topic echo /cmd_vel
```

There should be one controller, active [3], publishing to /cmd_vel. With a goal
active, nonzero velocity should appear. Ctrl+C stops the echo only. A green
path confirms planning, not drive connectivity or successful goal completion.

## 10. Click SceneSplat candidates in RViz and preview navigation

Implemented 2026-10-12; synthetic clustering/geometry and isolated ROS integration
checks passed. Live semantic accuracy, RViz mouse interaction and object approach
in the changed room still need user verification. No live semantic goal was sent
during implementation. SceneSplat source/environment and Isaac graphs are unchanged.

The existing `/find_object` service keeps its single best-candidate response for
compatibility. It now also publishes all ranked clusters on the latched
`/object_query/candidates` topic. JSON artifacts contain candidate IDs, boxes,
centroids, counts and mean raw cosine; the NPZ stores each candidate's filtered
and source Gaussian indices as well as the original best-result arrays. IDs are
local to one query, not persistent object identities. A cluster can be part of
an object or combine multiple nearby objects; scores are not confidence.

Rebuild both packages and restart the query service after this interface change:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/build_object_query.sh
```

Start navigation/localization/scans using section 9. Stop WASD. Select the exact
new reconstruction folder with completed matching features; do not use an older
run by assumption. The saved map and reconstruction must describe the same room
in matching world metres. No registration/alignment estimation is implemented.

In separate terminals:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh /absolute/path/to/NEW_MCMC_RUN
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline semantic_navigation --ros-args -p use_sim_time:=true -p clearance:=0.7
```

For the changed-room restart, use the 0.2m minimum in the checklist above;
the bridge can expand the stopping distance when the closest poses are blocked.

The bridge needs fresh `/global_costmap/costmap`,
`/local_costmap/published_footprint`, world↔nav_base TF, and active Nav2 planner/
navigator actions. It reads Nav2's actual padded footprint, not a guessed polygon.
It publishes no velocity; existing Nav2 cmd_vel routing determines drive connection.

In RViz, Fixed Frame world and simulation time enabled:

1. Add → By display type → InteractiveMarkers. Set Update Topic to
   `/semantic_navigation/candidates/update`.
2. Add → By topic → `/semantic_navigation/preview_path` → Path. Use Reliable /
   Transient Local if added after a preview. Give it a distinct color from /plan.
3. Hide the old `/object_query/markers` display if its best-only box overlaps.
4. Query in another sourced terminal:

```bash
ros2 service call /find_object lidar_outline_interfaces/srv/FindObject "{query: 'chair'}"
```

5. Wait for numbered boxes. Click the RViz Interact tool, then left-click the
   desired box. If clicked immediately after creation, wait a second of simulation
   time and click again (installed interactive-marker server client timeout).
   Right-click → Preview approach to this candidate is also available.
6. Wait for the green approach arrow and route. Selecting a box never starts
   navigation. The bridge checks up to 16 poses, faces the selected bounding-box
   centre, and chooses the shortest successful route among tested poses.
7. To drive, right-click the SAME candidate box → Approve selected approach and
   navigate. Approval replans and rechecks current footprint clearance before
   sending NavigateToPose. Do not use RViz 2D Goal Pose for this approval: that
   sends an independent goal directly to Nav2.
8. Right-click → Cancel navigation / clear preview to cancel. Pause Isaac for
   an immediate stop; keep paused until cancellation completes.

Status and optional terminal approval/cancel:

```bash
ros2 topic echo /semantic_navigation/status
ros2 service call /semantic_navigation/approve std_srvs/srv/Trigger '{}'
ros2 service call /semantic_navigation/cancel std_srvs/srv/Trigger '{}'
```

Clearance starts at 0.7m from the box to a conservative circle enclosing the
padded robot footprint, making corner approaches conservative. This is greater
than a centre-to-centre distance. If that ring has no reachable pose, the bridge
tries farther rings in 0.2m increments up to `max_clearance` (default 1.2m).
Both values are configurable at bridge startup; neither asserts object identity
or true object extent. Unknown, inscribed and lethal cells are rejected. The
bridge samples the moving footprint along Nav2's path with a 0.4m lookahead
heading and checks the final alignment turn. It no longer requires an in-place
turn at every grid-scale bend. Only finite sampled poses are searched; no
reachable result does not prove no possible approach exists.
Local obstacle avoidance remains Nav2's responsibility during execution.

Default `max_candidates` is 20 and `approach_samples` is 16 (startup parameters).
New queries replace old boxes/previews. While navigating, new query results are
ignored: cancel first, then issue the query again. Clock resets cancel navigation
and invalidate previews; stale costmap/footprint updates request cancellation.
The bridge does not arbitrate independent RViz goals or other Nav2 clients:
avoid sending manual goals while semantic navigation is active.

Live acceptance: verify multiple boxes correspond to intended objects, click one,
inspect approach clearance and route, then approve one short approach in clear
space. Check navigation reports success and the robot faces the selected object.

### If a candidate click waits for a current costmap

The bridge now requests Nav2's current `/global_costmap/get_costmap` service
when selecting a candidate and again before approved navigation. This handles
Nav2's full OccupancyGrid topic stopping after an Isaac clock reset. It still
requires current `/local_costmap/published_footprint` and world/nav_base TF;
selection never proceeds on stale data. Restart only `semantic_navigation` after
building this change, then click the candidate again. A message about waiting
for fresh inputs means no navigation goal was sent. `ros2 node list` should show
only one `/isaac_localization`; duplicate localization nodes can publish
conflicting TF and should be stopped before semantic navigation tests.
