# Repository and Isaac Sim handoff

## Restart and current navigation handoff — 2026-10-12

- WORKFLOW.md now starts with a complete restart checklist for the saved
  changed-room map, scans, localization, Nav2, `/find_object`, semantic bridge,
  RViz and query. Verified Isaac scene file is
  `/home/jovan/Desktop/rbkairos_franka_project/6_cameras_rb_kairos_new.usd`.
  Verified changed-room run is
  `/home/jovan/mcmc_runs/capture_20261012T114946_176800Z_depth_20261012T141242_524363`.
- Semantic bridge was rebuilt. It searches clearance rings from the requested
  minimum up to 1.2m and checks a moving footprint with path lookahead rather
  than requiring an in-place turn at every Navfn grid bend. Isolated semantic
  ROS test and approach geometry tests passed. Restart the bridge to load it.
- User saw a semantic preview and commanded navigation. Final heading behavior
  and wheel slip are unresolved. A negative `/cmd_vel.angular.z` corresponded to
  visible clockwise rotation. User suspects current differential drive slips;
  omnidirectional Isaac/Nav2 control is a future change, not yet configured.
  Do not claim this navigation is fully validated or change Isaac drive silently.
- Current Nav2 YAML sets desired_linear_vel 0.7m/s and heading speed 0.8rad/s;
  restart guide applies optional slower runtime values 0.2m/s and 0.4rad/s.
  The map and robot footprint remain as listed in WORKFLOW.md. Never launch
  duplicate Nav2/localization/bridge nodes or start live navigation on behalf
  of the user unless they explicitly request that runtime action.

## Live plant-query preview diagnosis — 2026-10-12

- Live `/find_object` plant query returned one cluster and RViz interactive
  marker `candidate_1`. User clicked it; bridge status entered "Checking 5
  approach routes", but first implementation was slow because it converted the
  800x800 OccupancyGrid for each footprint sample. Fixed by caching array.
- After restart, click reported "Waiting for fresh global costmap and padded
  robot footprint". Live costmap topic publisher existed but sent no new full
  messages after several Isaac clock resets. Nav2 `/global_costmap/get_costmap`
  service returned a current 800x800 world costmap. Bridge now fetches it on
  selection and again on approval, and refreshes while active. Old latched map
  stamps are ignored. Fresh padded footprint/TF still required.
- Rebuilt packages and isolated ROS test passed with mock GetCostmap: click only
  previewed, newly blocked goal was refused at approval, then valid approval
  sent mock navigation. Live bridge must be restarted by user to load update.
  Do not auto-send a live semantic goal. Only one localization node should run;
  duplicate /isaac_localization was observed in live node list.

## Clickable semantic navigation implementation — 2026-10-12

- User approved concept and implementation: multiple candidates selectable by
  clicking RViz boxes, approach/path preview, explicit approval before movement.
- Added ObjectCandidate/ObjectCandidates messages. Existing FindObject service
  unchanged; query server publishes ranked clusters latched on
  /object_query/candidates. Worker JSON/NPZ retain all candidate boxes/scores/rows.
- Added approach_goal.py geometry and semantic_navigation node. Interactive
  markers namespace /semantic_navigation/candidates; click previews, right-click
  same box approves or cancels. Uses live padded Nav2 footprint and world costmap;
  samples approaches at .7m conservative envelope clearance; plans all eligible
  positions and picks shortest successful route, checking footprint along route
  and rotations. Approval replans; no direct velocity publisher or SceneSplat edits.
- Status /semantic_navigation/status; preview /semantic_navigation/preview_path.
  Trigger services /semantic_navigation/{approve,cancel}. Cancellation on stale
  map/footprint and clock reset. New queries ignored during navigation; cancel
  then query again. Avoid independent RViz goals/other Nav2 clients while active.
- Both packages built. Geometry, multicluster and isolated domain88 ROS test
  cover click→preview without motion, blocked approval rejection, explicit
  approval→mock navigation, and query replacement. No live semantic goal sent.
  Live changed-room semantic accuracy and RViz click/approach validation remain.
- WORKFLOW.md section10 contains commands and limitations. Obtain exact new
  completed MCMC run path; never silently reuse historical run. User previously
  requested manual control of runtime commands; implementation build/tests are
  authorized, but don't start live services or robot movement automatically.

## Nav2 live diagnosis — 2026-10-12

- Started localization bridge, confirmed live /nav/odom. Live audit footprint
  min [-.412464,-.290751,-.190427], max [.524825,.290666,1.563008]. Preview
  polygon [[-.42,-.30],[.53,-.30],[.53,.30],[-.42,.30]], padding .03m.
- Nav2 runs via /tmp/navigation_existing_map.launch.py, derived from repository
  launch with map_server omitted to reuse the user's active saved-map server.
  /tmp launch files/scripts are session helpers, not durable installed commands.
  Planner/controller/navigator activated once Play resumed. Planning-only action
  succeeded (31–32 poses, about .8m path). RViz Path /plan must be present before
  requesting a plan; recomputation made it visible to user.
- User stopped teleop and explicitly authorized driving test. Temporary relay
  forwarded /nav/cmd_vel_preview to /cmd_vel only during monitored tests; timeout
  cancels goal, publishes zero and removes relay. Latest test ended disconnected.
- Initial goal was diagonally behind robot; controller requested initial heading
  rotation .06rad/s (max_angular_accel .6 at 10Hz), no measurable yaw movement.
  Disconnected diagnostic confirmed clock/local costmap updating and 100 commands
  in 10 wall seconds; simulator time ran roughly one-third real time.
- Live acceleration comparison: 2.0 -> initial .2rad/s, measured turn only about
  .007rad/s. 4.0 -> initial .4rad/s, measured about .088rad/s, yaw changed from
  -.399 to -.812 over ~15 wall seconds. Retained live FollowPath.max_angular_accel
  4.0 and updated source config; heading speed cap remains .4rad/s. Installed
  config requires rebuild or explicit source config launch to retain on restart.
- Diagnosis: low initial rotation command stalled base; stronger command turns,
  but actual rate is substantially lower than commanded. No navigation goal has
  reached success yet. Further drive-response/velocity and goal-following checks
  remain. Do not report autonomy complete or change Isaac drive graphs blindly.
- Live footprint in RViz: Polygon /local_costmap/published_footprint; TF nav_base
  for position/heading, Fixed Frame world. No full 3D bounding box publisher added.

## Changed-room map and RViz continuation — 2026-10-12

This section supersedes older current-state statements below.

- User changed the room and reports the new map and SceneSplat are working.
  New capture/reconstruction/feature paths were not supplied or independently
  checked; do not reuse an older run by assumption or retrain automatically.
- User initially stopped world_map before a successful save; only old maps and
  one bad save remained. Repeated mapping instructions were provided. New saved
  map maps/room_redo_20261012T141102.yaml exists and references its matching PGM;
  resolution 0.05m, origin [-20,-20,0]. RViz screenshot shows room/furniture outlines.
  Do not treat screenshot/display success as completed navigation validation.
- Saved-map publisher initially failed because the path omitted maps/. Correct
  absolute YAML path is /home/jovan/ros2_ws/src/lidar_outline/maps/room_redo_20261012T141102.yaml.
  map_server must be configured then activated. Saved map topic /map; collector
  topic /nav/map. Stop the old map_server before starting a replacement.
- RViz LaserScan showed 0 points from 0 messages and blank color transformers.
  Live inspection confirmed navigation_scans publishing and scans arriving;
  RViz scan subscription and adapter publisher both BEST_EFFORT/VOLATILE.
  Set both LaserScan displays to Best Effort/Volatile; map display remains
  Reliable/Transient Local. RViz use_sim_time was False; changed it successfully
  with ros2 param set /rviz use_sim_time true. /clock was arriving from Isaac.
  User refreshed displays and confirmed map and lasers are now visible/working.
  Latest live node list included map_server, navigation_scans, rviz, teleop;
  no localization bridge or world_map observed in that check. Recheck live state.
- Start RViz using rviz2 --ros-args -p use_sim_time:=true for live overlays.
  Fixed Frame world. LaserScan topics /nav/front_scan and /nav/rear_scan.
  Once scans arrive, FlatColor exposes Color; red/blue help distinguish hits.
- world_map is the in-memory LiDAR collector; navigation_scans only adapts scans;
  capture_camera independently records RGB/depth for Gaussian training. Save
  /nav/map while collector and Play are running; stop recorder before stopping
  Play. Collector restart/reset loses its accumulated map. Saved capture survives.
  Mapping and recording can share one slow manual drive. Suggested teleop speed
  0.2m/s, turn 0.5rad/s; screenshot had speed 1.5, so advised reducing it.
- Localization remains simulator measured TF -> /nav/odom and navigation aliases,
  not wheel encoders or AMCL. Pose followed manual motion previously; moving
  velocity check remains pending. Map display alone starts no localization/nav.
- Next: verify saved-map/live-scan alignment while translating and turning;
  rerun live footprint audit; verify localization/velocity; preview route planning
  with disconnected output; then connect drive and test short ordinary goals.
  AMCL is an additional task if map-based estimated localization is requested.
  Object-query-to-reachable-approach-goal bridge remains unimplemented. No
  autonomous navigation goal has been tested. Preserve SceneSplat and working
  Isaac drive/LiDAR/camera setup, including disabled D455 IMUs.

## Nav2 continuation — 2026-10-12

Latest continuation (supersedes initial findings in this section):
- Corrected mapping pass now shows room walls/furniture. Saved
  maps/room_20261012_corrected.{pgm,yaml}. Progress snapshot had1490 occupied
  cells, with203/181/165/270 cells within7.5cm of left/right/bottom/top audited
  wall faces. Visually inspected full room map; coverage much improved.
  First failed room_20261012 map remains diagnostic only. Temporary Nav2
  preview bringup stopped after successful lifecycle check. Collector continues.
  Need live footprint (rerun latest audit), then plan/test ordinary goals.
- User ran live audit. Both lasers physically descend +135 to -135 with
  ascending ROS metadata. Added navigation_scans adapter; reverse_ranges:=true
  publishes /nav/front_scan and /nav/rear_scan without changing Isaac/raw topics.
- Main empty-map cause confirmed: rayTimeOffsets spread each sweep over .08s.
  Installed raycast example explains only rays within the current physics step
  fire. Audit had exactly113 contiguous fired indices293..405 of541 for both
  lasers; inactive beam endpoints equal origins while depths report30m.
  Treating these as free rays erased obstacles. Adapter conservatively maps
  maximum-range/Inf samples to NaN (discard_max_range defaults true). This also
  discards genuine no-hit clearing because ROS lacks an active-ray mask.
- Fresh corrected map preserves wall/furniture outlines; initial stationary
  preview had159 occupied/3925 free cells. User asked asynchronously to make a
  new slow mapping pass and stop, leaving Play on. Completion reply pending.
- Isaac clock was reset during work. Old TF buffer used system clock and
  rejected past TF; fixed bridge and mapper Buffer(node=self), explicit bridge
  clear on reset. Live /nav/odom recovered. Isolated clock-reset integration
  plus scan/map/planar tests:9 passed in ROS_DOMAIN_ID77.
- Added config/nav2_isaac.yaml and launch/navigation.launch.py, built packages.
  map and footprint arguments required; cmd_vel_topic defaults to disconnected
  /nav/cmd_vel_preview. All five Nav2 lifecycle servers activated live against
  /tmp/nav-active-rays-check.yaml with PROVISIONAL .9x.8m footprint. This checks
  startup only: no goal sent, live footprint not verified, no driving connected.
- Runtime logs /tmp/nav-{localization,scans,mapping,rviz}-live.log and
  /tmp/nav2-preview-bringup.log. Corrected collector and scan adapter run from
  source; installed entrypoints were built too. Localization uses source module.
- Current audit /tmp/isaac-live-lasers.json includes87 environment colliders,
  hit prim paths and native endpoints. It predates robot_bounds_in_base field
  added to audit script; rerunning same read-only script will also measure robot.
- Room inner wall faces: x=-2.5388664 and5.3384291,
  y=-3.8360142 and1.4392523, world metres. Check accumulated map against these.
- Next: receive finished/stopped reply, save NEW corrected map (original
  maps/room_20261012 remains invalid diagnostic), inspect room alignment/coverage,
  audit live footprint, plan ordinary goals, then connect drive and test goals.
  Semantic approach-goal bridge remains unimplemented. SceneSplat untouched.

- Nav2, map server, controller and SLAM Toolbox are installed. Live Isaac clock,
  six cameras and both scans observed. User confirms open scene is
  `6_cameras_rb_kairos_new.usd`.
- Started installed isaac_localization with source_base_frame summit_xl_base_link.
  Live /nav/odom pose followed user's manual drive (~44cm and -17deg yaw from
  initial sample); moving velocity still requires a recorded check. No autonomous
  commands issued. RViz opened with config/mapping.rviz.
- Added world_map node, using full timestamped laser TF, world-aligned 5cm grid,
  bounded scan queue, clock-reset clearing. Source was run directly (no build yet).
  Initial collector stopped. Saved maps/room_20261012.{pgm,yaml} is DIAGNOSTIC ONLY:
  almost no obstacles survive, so it must not be used for navigation.
- Saved scene rayDirections run +135 to -135 while ROS metadata runs -135 to
  +135. This is a suspected scan order mismatch; LIVE ray settings must be
  inspected because previous calibration scripts may have changed open scene.
  No Isaac scene/graphs or existing drive/LiDAR setup changed.
- scripts/inspect_live_lasers.py is a read-only Isaac Script Editor audit writing
  /tmp/isaac-live-lasers.json. Next: user runs audit in Isaac; inspect results,
  correct scan interpretation externally if needed, rebuild a valid map, verify
  full live footprint, configure Nav2 and test ordinary goals before semantics.
- Saved robot geometry envelope approximately x +/- .3595m, y +/- .29031m;
  remote camera references failed standalone resolution. This is not a verified
  live footprint. Nav2 config/launch and semantic navigation remain unfinished.

## Current continuation point — 2026-10-09

This section supersedes older unfinished-work statements below; keep those as history.

- User confirmed saved-scene `/find_object` queries and RViz bbox display work.
  Do not treat this as verified semantic accuracy, Isaac scene annotation or navigation.
- Code cleanup/archive and descriptions in all 42 then-existing Python/shell files
  are complete. New localization files also include purpose docstrings.
- Recording is separate. The unified script validates a finished capture, trains
  MCMC, runs SceneSplat and starts the feature viewer. It does not launch capture
  or `/find_object`. Do not silently start/retrain when asked only for instructions.
- Localization bridge is built; geometry and isolated synthetic ROS checks passed.
  It has NOT been verified against a live Isaac base frame or driving session.
- Nav2 sudo install failed for lack of a password. User instructed to run
  `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`; no completion
  confirmation yet. Recheck installation/live state at the next session.
- Next: user completes install and starts saved scene/Play → inspect live TF,
  clock, scans → verify localization → build aligned map and verify footprint →
  configure Nav2 and ordinary goals → external semantic approach-goal bridge.
- No navigation map, Nav2 launch/config or object-to-navigation bridge is ready.
  No autonomous motion has been started. Preserve SceneSplat, environments,
  recordings/reconstructions, differential drive/LiDAR and disabled D455 IMUs.
- Current commands and stopping point: WORKFLOW.md, especially section 8.
  README.md now reflects completed reconstruction/query work and pending Nav2.

## Localization/Nav2 preparation — 2026-10-09

- User requests continuing localization and Nav2. Live ROS currently has only
  /cmd_vel, /parameter_events, /rosout; no Isaac /clock, TF or scans observed.
  Async request sent to open saved six-camera scene and press Play; no reply yet.
- Nav2 is not installed. sudo -n apt-get install ros-jazzy-navigation2
  ros-jazzy-nav2-bringup failed because a password is required, not auto-review
  rejection. User asked asynchronously to run sudo apt install in own terminal.
  No system installation completed and no navigation motion was started.
- Added planar_pose.py, isaac_localization.py and ROS entry isaac_localization,
  nav_msgs dependency; built both packages. Required source_base_frame must be
  verified live, not guessed. Uses measured world-to-base TF, publishes planar
  /nav/odom and world->nav_odom->nav_base aliases, preserves existing TF parents.
  This is simulator ground truth, not wheel odometry/AMCL. No cmd_vel published.
- Three planar geometry tests and isolated ROS_DOMAIN_ID76 end-to-end test
  passed (104 synthetic odometry messages, frame/body velocity/position checked).
  Test processes stopped. Log /tmp/localization-test.log.
- WORKFLOW.md section8 records prerequisite install and outstanding live checks.
  Map, footprint verification, Nav2 config and semantic-goal bridge NOT ready.
  Keep deactivated D455 IMUs, working differential drive and SceneSplat untouched.

## Repository cleanup — 2026-10-09

- User authorized neutral pose helper, archive of retired workflows and new
  MCMC-focused WORKFLOW.md. Completed without changing SceneSplat/environments,
  captures, reconstructions or working drive/LiDAR setup.
- scripts/camera_pose.py now provides optical_to_world (NumPy 4x4 camera-to-world
  in unchanged world metres, xyzw quaternion) and optical_to_opengl for archived
  conversion. Active train_capture_mcmc.py and view_mcmc.sh use optical_to_world
  directly; no active Splatfacto converter import remains.
- archive/splatfacto contains standard train/export/conversion, old generic
  SceneSplat inference/viewing and normalized bbox conversion/helper.
  archive/isaac_setup contains superseded TF installer/recovery. Legacy tests
  moved to archive/tests; active neutral pose checks in test/test_camera_pose.py.
  Archived scripts' internal imports updated for new locations.
- Reopen preserved standard result with
  bash ~/ros2_ws/src/lidar_outline/archive/splatfacto/view_scenesplat.sh
  (old scripts/view_scenesplat.sh path no longer exists).
- archive/WORKFLOW_BEFORE_CLEANUP.md preserves historical commands with old
  paths. Current WORKFLOW.md is recording → pipeline/reuse → manual stages →
  external query → RViz → rebuild. README obsolete TF directions replaced.
  pytest.ini limits default discovery to active test/; archive has COLCON_IGNORE.
- CLEANUP_AUDIT.md original recommendations remain below a completed-status
  note as history. Active scripts need no rebuild for this pose refactor.
- Verification: 12 pose/MCMC/depth/pipeline/legacy-compatibility tests passed;
  real recorded-pose viewer camera-state setup passed in a temporary directory;
  shell/Python syntax and whitespace checks passed. No training/viewer launched.

## External object query service — 2026-10-09

- User authorized separate SigLIP2 scoring/DBSCAN/ROS service, no SceneSplat
  edits. Implemented lidar_outline/object_query.py (clustering),
  scripts/object_query_worker.py (cached encoder/features in scene_splat Python),
  lidar_outline/find_object_node.py (Jazzy adapter), scripts/find_object.sh.
- Custom service interfaces/lidar_outline_interfaces/srv/FindObject.srv:
  request query string, response found/status/header/centroid/bbox min/max,
  mean raw cosine/counts/result_file. Separate interface package discovered
  explicitly by scripts/build_object_query.sh. Both packages built successfully.
  No SceneSplat source/environment edits or installs.
- WORKFLOW.md section 6 has ready commands. Default threshold .75 relative
  per-query minmax (same as viewer, not confidence), top10000, DBSCAN eps .15m,
  min_samples5, min_cluster_points20. Rank clusters mean raw cosine, count tie.
  Returns selected Gaussian-center enclosure, potentially a partial object.
  Not a verified semantic detection, navigation goal or collision box.
- Input requires identity MCMC world metres, filtered matching PLY/768D finite
  features and row mapping. Saves JSON/NPZ query artifacts with filtered/source
  rows. Publishes RViz markers /object_query/markers, does not change live Isaac.
- Four synthetic clustering tests passed in scene_splat, real query engine
  and isolated ROS_DOMAIN_ID74 service/client test passed. Table candidate:
  centroid (1.26618,-2.36612,.37443), 9998 rows, mean cosine .113716. Accuracy
  not visually confirmed. Diagnostics /tmp/object-query-{check,ros-check}.
  Test processes stopped. Nav2 and robot motion remain unimplemented.

## One-command pipeline update — 2026-10-09

- User requested recording RGB/depth → dataset preparation → MCMC → SceneSplat
  → feature viewer, followed by text-query navigation to detected objects.
- Added scripts/run_room_pipeline.sh and scripts/room_pipeline.py, documented
  at top of WORKFLOW.md. Exact capture argument required; preflight checks all
  pairs, dimensions, timestamps, poses/calibration and rejects multiple segments.
  Calls existing standalone trainers/inference/viewer; no SceneSplat changes.
  New run by default; --run reuses only a completed matching depth run. Logs
  alongside run, prints streamed output, checks busy port and feature/PLY rows.
  --check-only, --no-view, --port supported. Training resume not implemented.
- Verified latest depth capture capture_20261009T110859_039769Z has 1029 pairs;
  complete MCMC run capture_20261009T110859_039769Z_depth_20261009T131838 has
  splat.ply. Earlier capture_20261009T104400_506749Z is RGB-only (1300 views).
- Source recorder save_depth default was changed by user to True; previously
  installed ROS copy still had False. Explicit -p save_depth:=true avoids stale
  installed defaults; source edits require colcon rebuild for ros2 run.
- Pipeline real-capture preflight and three unittest checks passed (invalid
  capture, stage failure/logging, busy port). Full new pipeline was not launched.
- User wants query to send robot to object. No Nav2 bringup/messages installed
  in /opt/ros/jazzy/share, no navigation/costmap config found under workspace src.
  Query navigation is NOT connected. Needs object localization, reachable
  approach goal, obstacle map/localization/navigation controller. Preserve drive.

## Depth initialization update — 2026-10-09

- User authorized depth-based MCMC initialization for the next attempt and plans
  to record again. Full training has not been started.
- Audited all 1539 depth arrays in capture_20261008T144755_490345Z: float32,
  720x1280, exact paired RGB timestamps. Sampled nearby-view reprojection strongly
  supports axial-Z depth in world metres; median error about 0.8–1.4mm, 93–98%
  within 5cm. Sample RGB/depth silhouettes visually agree. Diagnostics in
  /tmp/capture_depth_check.{json,png}; these are sampled geometry checks.
- scripts/train_capture_mcmc.py and its shell launcher now default to depth
  initialization via scripts/depth_initialization.py: training views only,
  valid 0.1–10m original-resolution pixels, matching RGB colors, measured optical
  poses. Samples 100000 seeds, deduplicates on a 5mm grid. No depth loss yet.
  MCMC_INITIALIZATION=random restores the old initialization option.
- Saves initial_points.npz and initialization/bounds/depth-supervision settings
  with each new run. Old outputs and SceneSplat unchanged.
- Direct known-plane geometry/color/timestamp checks and full-capture 100000 seed
  generation passed. Tiny GPU run in /tmp/mcmc-depth-smoke-run passed 610 steps,
  refinement 2000->2100 and PLY export. This verifies execution, not visual quality.
  Added focused pytest regression; pytest is absent from splatfacto/base envs,
  so pytest suite was not run. No environment packages changed.
- WORKFLOW.md documents the depth defaults and explicit new-capture command.

## Restart handoff — 2026-10-08

This handoff and the next section supersede older stopping-point statements
below. Older sections retain history and can describe work that is now complete.

- User reports this custom MCMC reconstruction is visibly worse than the
  earlier standard Splatfacto reconstruction. Prefer the earlier good result
  for continued use. Do not retrain automatically or claim MCMC will fix queries.
- Acknowledged limitation: the tiny MCMC tests verified execution, refinement,
  camera projection and export, not visual reconstruction quality. Custom MCMC
  and prior Splatfacto runs also differ in initialization/training settings;
  this is not a controlled comparison of the two strategies.
- New SceneSplat extraction is complete, not pending. It uses the filtered
  MCMC PLY (83580 rows), not the original million-row PLY. Query computation
  works with matching768D base SigLIP2, but semantic accuracy remains weak:
  couch selects roughly73% at threshold0.5. Counts do not prove correct objects.
- User requested stopping the viewer: feature viewer on8083 was terminated.
  At this handoff, old standard viewer8080 and raw MCMC viewer8082 still run.
  After a computer reboot none should be assumed running; inspect ports first.
- User wants short, ready-to-run instructions instead of prolonged setup.
  `WORKFLOW.md` is the command guide for recording, training, export, SceneSplat
  inference and viewing. No rebuild required for these standalone shell scripts.
- Reopen preferred old standard result:
  `bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat.sh` -> localhost8080.
- Reopen new MCMC reconstruction only:
  `bash ~/ros2_ws/src/lidar_outline/scripts/view_mcmc.sh` -> localhost8082.
- Reopen filtered MCMC with already computed features:
  `bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat_mcmc.sh` -> localhost8083.
  Do not rerun extraction just to view it. Correct launcher uses cached
  google/siglip2-base-patch16-512 in offline mode; default SO400M is incompatible.
- SceneSplat source and environment were not changed during this MCMC session.
  Existing earlier compatibility edits remain. Continue respecting the user's
  explicit prohibition on modifying SceneSplat.
- Current raw recording, finished MCMC run, filtered inputs/features and original
  good Splatfacto outputs are all preserved; see exact paths in next section.

## 2026-10-08 new capture and MCMC workflow

- Fixed MCMC feature-viewer launcher: direct mini-viewer defaults to SigLIP2
  SO400M, incompatible with the768D SceneSplat features and causing model-load
  delay. Explicitly pass google/siglip2-base-patch16-512 and HF_HUB_OFFLINE=1
  (base model already cached). No SceneSplat code change. Direct actual encoder
  scoring on new features passed: plant274 selected, couch60905 at threshold0.5.

- User prefers concise commands and reports standard Splatfacto visually
  better than this MCMC result. Recommend the better standard result; do not
  claim MCMC automatically improves reconstruction or SceneSplat semantics.
- `WORKFLOW.md` contains recording, standard Splatfacto conversion/train/export,
  SceneSplat preprocessing/inference/viewing, old-result viewing and optional
  MCMC commands. Standard train/export/inference launchers now accept optional
  paths while retaining earlier defaults. SceneSplat code remains unchanged.
- MCMC SceneSplat raw inference failed spatial serialization assertion because
  Gaussian coordinates reach tens of kilometres. Separate helper
  filter_mcmc_for_scenesplat.py crops trajectory min/max plus5m each axis and
  opacity>=1/255, keeping83580/1000000; original PLY preserved. Records
  filter.json and source_indices.npy; filtered PLY and arrays are row-aligned.
- SceneSplat inference completed on filtered input with393/393 checkpoint keys.
  Features shape(83580,768), allfinite and allnonzero, row count verified against
  filtered PLY. PCA/feat-vis PLYs also created. Separate input/output roots:
  `~/scenesplat_runs/{input,output}/mcmc_filtered/<MCMC run basename>/`.
- scripts/run_scenesplat_mcmc.sh handles preprocessing/filter/inference and log;
  scripts/view_scenesplat_mcmc.sh loads filtered PLY plus matching features at
  localhost8083. Raw MCMC viewer stays8082; old standard feature viewer8080.

- Full MCMC run finished: `~/mcmc_runs/capture_20261008T144755_490345Z_20261008T170318/`,
  checkpoint_030000.pt and 237MB splat.ply verified present.
- Separate `scripts/view_mcmc.sh` launches installed mini-viewer directly on
  that PLY, localhost8082. Seeds a recorded optical viewpoint only if missing,
  storing camera state beneath the MCMC run directory. SceneSplat untouched.
  This raw viewer has no semantic features; new features belong to the separate
  filtered PLY and are loaded by view_scenesplat_mcmc.sh. Old features do not align.

- New recording: `/home/jovan/room_captures/capture_20261008T144755_490345Z`.
  1539 RGB/depth pairs, one segment. Per-camera counts: d455=240, _01=241,
  _02=273, _03=263, _04=265, _05=257. Camera frames match their namespaces.
- User explicitly requests MCMC training and prohibits changes to SceneSplat.
  Keep SceneSplat checkout, viewer, and its environment untouched.
- Separate launcher `scripts/train_six_cameras_mcmc.sh` defaults to this new
  capture; old Splatfacto launcher and old reconstruction are preserved.
- `scripts/train_capture_mcmc.py` reads recorded RGB, intrinsics, optical TF
  directly (segment_000), uses installed gsplat1.4 MCMCStrategy with upstream
  MCMC initial opacity0.5, initial scale multiplier0.1 and opacity/scale
  regularization0.01. No COLMAP pose estimation or depth supervision.
- Defaults: 30000 steps, downscale2 (640x360), random100000 Gaussians around
  trajectory, cap1000000, fixed measured poses, every eighth image held out.
  Checkpoints each5000 steps; validation.png original left/render right;
  sampled validation PSNR printed. Completed run exports Graphdeco splat.ply.
- Outputs under fresh timestamped `~/mcmc_runs/` directories. Identity
  world_transform.json: PLY positions retain recorded global world metres.
  Never apply the previous Splatfacto normalization to this new output.
  /World parent transforms still matter for local marker placement.
- Uses existing splatfacto Conda environment without package changes. CUDA
  cache may compile; launcher activates Conda before enabling nounset.
  Close Isaac and SceneSplat viewer before full run for available GPU memory.
- Focused projection and PLY round-trip tests passed (2). Tiny GPU run610
  steps/2000 initial Gaussians completed refinement to2100 and PLY export.
  This checks execution, not reconstruction quality. Full training subsequently
  completed as recorded above, but user reports poor visual quality.
- Interrupted checkpoint saved on Ctrl-C; automatic resume is not implemented.

Last updated: 2026-10-08. This file records verified session findings and
instructions for continuing the room-capture project. Recheck live state before
making changes; saved USD files can differ from the open Isaac scene.

## Working style

- Give small, concrete, click-by-click UI steps. The user is new to Isaac.
  Explain unfamiliar terms: the viewport is the large 3D room window.
- Use labels visible in screenshots or verified in the installed version.
  Do not invent graph ports, settings, or camera paths.
- Inspect available local files and logs before guessing from screenshots.
- Keep the RealSense D455. Do not substitute Orbbec or another camera.
- Preserve D455 internal transforms. Earlier advice to rotate a plain USD
  camera 180 degrees about X does not apply to the whole D455 asset.
- Distinguish commanded velocity from measured robot motion and pose.
- Editing files does not change the open Isaac scene automatically. State
  explicitly when a script must run in Isaac or a saved scene must be reopened.
- Do not spawn subagents unless the user explicitly requests delegation.
- Keep fixes scoped to the task; preserve working drive and LiDAR setup.

## Environment and repository

- Ubuntu; ROS 2 Jazzy; Isaac Sim 6.0.0 installed at `/home/jovan/isaacsim`.
- Workspace: `/home/jovan/ros2_ws`.
- Package: `/home/jovan/ros2_ws/src/lidar_outline` (ament Python).
- Scene originally used:
  `/home/jovan/Desktop/rbkairos_franka_project/rbkairos_franka_ros.usd`.
  A new working scene copy was suggested, but its creation/path is unconfirmed.
- Robot: RB-KAIROS mobile base with Franka arm using `fr3_*` link names.
- `lidar_outline/lidar_outline_node.py`: scan-to-line RViz markers.
- `lidar_outline/wasd_teleop.py`: focused keyboard window publishing Twist.
- `launch/lidar_outline.launch.py`: outline and RViz launch.
- Read README.md, setup.py, and package.xml when changing package behavior.

Build from the workspace root when code changes require it:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/jovan/ros2_ws
colcon build --packages-select lidar_outline
source install/setup.bash
```

Run appropriate checks for code changes. Documentation-only changes need no
simulation run or package rebuild. Do not add tests that merely mirror code.

## Working robot drive and LiDAR

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline wasd_teleop --speed 0.3 --turn 1.2
```

- Keep the separate WASD window focused. W/S moves; A/D turns; combinations work.
- BaseDrive: ROS2 Subscribe Twist -> Differential Controller -> two
  Articulation Controllers for front/rear wheels.
- Wheel radius: 0.11 m; wheel distance: 0.409 m.
- On Playback Tick -> Delta Seconds must connect to Differential Controller ->
  Dt. Without it, acceleration limiting stopped command output.
- Articulation root:
  `/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint`.
- `root_joint` disabled to free the base; keep
  `summit_xl_base_footprint_joint` enabled.
- Unwanted `box_1` collider caused rocking; its collision was disabled.
  Later screenshots show `box_1` deactivated. Do not re-enable it casually.
- Front/rear LiDAR and RViz outline work. RViz Fixed Frame: `world`.
- Calibration scripts outside this repository:
  `/home/jovan/Desktop/rbkairos_franka_project/front sensor calibration.py`
  and `/home/jovan/Desktop/rbkairos_franka_project/rear_sensor_calibration.py`.
- Ray order changed to -135 degrees -> +135 degrees to match ROS scan angles.

## D455 mounting and crash workaround

Base link:

```text
/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/summit_xl_base_link
```

Verified asset hierarchy under that link:

```text
rsd455
  RSD455
    Camera_Pseudo_Depth
    Imu_Sensor
    Looks
    Visual
    Camera_OmniVision_OV9782_Color
    Camera_OmniVision_OV9782_Left
    Camera_OmniVision_OV9782_Right
    TemplateRenderProducts
```

- The actual asset name is `rsd455`; renaming to `base_camera` was suggested
  earlier but is not the latest verified state.
- Translation (0.35, 0, 0.65), scale (1, 1, 1) were provisional instructions,
  not verified final values. Do not overwrite the user's working placement.
- User confirmed color view faces correctly and follows the robot while driving.
- Existing Orbbec Astra housing/optical frames are not the configured camera.
- Playback crashed with D455 active. Deactivating the whole D455 prevented it.
- Disabling the inner `RSD455` rigid body did NOT prevent the crash.
- Outer `rsd455` has no physics tab according to the user.
- Deactivating `Camera_Pseudo_Depth` did NOT prevent the crash.
- User isolated the trigger: deactivating D455's **Imu_Sensor** makes playback
  work. Screenshot confirmed that child is deactivated. Keep it deactivated.
- Keep inner `RSD455` Rigid Body Enabled unchecked for the current mount.
  Do not disable the robot's rigid bodies or change its working joints.
- Restoring Visual and pseudo-depth was suggested; verify their live state.
- Root cause of the native IMU crash is not diagnosed. Do not present invalid
  inertia, driver issues, or camera rendering as established causes.

## ROS camera publishing: verified progress

UI shortcut verified in installed source:
**Tools -> Robotics -> ROS 2 OmniGraphs -> Camera** opens **ROS2 Camera Graph**.

Color camera prim:

```text
/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/summit_xl_base_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color
```

Suggested graph configuration (actual graph path still needs inspection):

- New graph `/Graph/D455_Camera`; do not add to BaseDrive.
- Camera Prim: full color camera path above.
- Frame ID: `d455_color_optical_frame`.
- Node Namespace: `d455`.
- RGB and Depth enabled; other optional outputs disabled.
- RGB Topic: `rgb/image_raw`; Depth Topic: `depth/image_raw`.
- Tool automatically adds CameraInfo publication.
- Using one color camera render product for RGB and depth gives simulated depth
  from the RGB viewpoint; do not claim physical stereo depth reconstruction.

Observed ROS topics:

```text
/clock
/cmd_vel
/d455/camera_info
/d455/rgb/image_raw
/front_laser/scan
/rear_laser/scan
/joint_commands
/joint_states
/tf
```

Depth spelling is unresolved: first topic list showed `/d455/depth/imageraw`,
later pasted command showed `/d455/depth/imagerawaw`. The same paste also showed
`--oncece` despite successful CameraInfo output. Re-list live topics before
configuring subscribers; do not assume either spelling is final.

ROS subscriber arrival rates: approximately 24 Hz RGB and 26 Hz depth.
These are measured arrival rates, not proof of frame synchronization or
simulation-time sensor frequency. Initial discovery warning cleared.

Verified CameraInfo:

- 1280 x 720.
- fx = fy = 634.0862399675711 pixels; cx = 640; cy = 360.
- `distortion_model: plumb_bob`; five distortion coefficients all zero.
- R identity; P consistent with K, no baseline term.
- `header.frame_id: d455_color_optical_frame`.
- Example simulation timestamp: 17.250000000 seconds.

## Current stopping point: camera TF missing

User ran:

```bash
ros2 run tf2_ros tf2_echo world d455_color_optical_frame
```

The initial `world` missing warning cleared as TF arrived, but
`d455_color_optical_frame` remained missing. Merely assigning the CameraInfo
Frame ID does not create TF. Other `/tf` publication does not establish camera TF.

Last requested UI step (not yet completed in the conversation):

1. Stop playback.
2. Open **Tools -> Robotics -> ROS 2 OmniGraphs -> TF Publisher**.
3. Inspect the **ROS2 TF Publisher Graph** window before generating a graph.

Installed shortcut implementation exposes Graph Path, Node Namespace,
Target Prim, Parent Prim (default to /World), Publisher Topic, and options for
adding to existing graph/node. It creates IsaacComputeTransformTree connected
to ROS2PublishTransformTree and timestamps from simulation time.

Do not publish a USD camera transform under an optical frame name without
checking axis conversion. USD camera convention is +X right, +Y up, -Z forward;
ROS optical convention is +X right, +Y down, +Z forward. A local 180-degree X
rotation relates these conventions, but apply it to a dedicated TF frame or
pose conversion, not to the working D455 asset/camera rendering transform.
Verify generated frame names, parent relationships, and existing world TF
before publishing; avoid duplicate/conflicting TF parents.

## Remaining capture and Gaussian-splat work

1. Publish and verify measured camera world pose with correct optical axes.
   Confirm pose changes when driving and agrees with the camera view.
2. Verify RGB, depth, CameraInfo, and pose timestamps use the same simulation
   clock. Match pose at each image timestamp, not latest pose or cmd_vel
   integration. Check reset-on-stop behavior across publishers.
3. Inspect RGB/depth encoding and real topic names; verify alignment and units.
4. Implement a capture workflow with intrinsics and timestamped camera poses,
   handling TF availability, dropped frames, and simulation time resets.
5. Record overlapping room views while driving slowly. Keep actual measured
   pose and meaningful viewpoint changes; avoid many identical stopped frames.
6. Choose a suitable trainer based on the available GPU/software environment;
   verify its current dataset format, pose convention, dependencies, and export
   support before converting. No trainer has been selected or installed yet.
7. Convert, train, inspect results, and export. No dataset recording, conversion,
   training, or export has been completed yet.

## Useful local diagnostics

- Logs: `/home/jovan/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/6.0/`.
- Crash artifacts: `/home/jovan/.local/share/ov/data/Kit/Isaac-Sim Full/6.0/`.
- Inspect newest actual crash log, not just newest startup log. Logs can contain
  previous-crash metadata; distinguish that from the current crash.
- ROS shortcut source:
  `/home/jovan/isaacsim/exts/isaacsim.ros2.ui/isaacsim/ros2/ui/`.
  `og_rtx_sensors.py`: camera graph; `og_utils.py`: TF graph;
  `extension.py`: menu registration.
- System Python lacks pxr. Standalone USD inspection succeeded using Kit Python
  with PYTHONPATH set to the installed omni.usd.libs extension directory and
  LD_LIBRARY_PATH to its bin directory. Discover current paths rather than
  relying on extension version hashes.
- Plain USD inspection may fail to resolve HTTPS references without Isaac's
  resolver, even when the open scene resolves them successfully.
- Official D455 USD was downloaded read-only for inspection to
  `/tmp/rsd455-inspect.usd`; it lacks the runtime-added IMU in the inspected
  bare composition. Inspect live/saved composed scene for the IMU settings.
- Prior sandbox sometimes failed with "mountinfo path is not absolute";
  approved elevated read commands worked. Use escalation only when needed.

## Verified six-camera update — 2026-10-08

- User configured six D455 namespaces: `d455`, `d455_01` through `d455_05`.
- Each publishes `/<namespace>/rgb/image_raw`, `/depth/image_raw`, and
  `/camera_info`. Original depth spelling was fixed to `depth/image_raw`.
- RGB is `rgb8`; depth is `32FC1`; inspected images are 1280 x 720.
  Depth units/alignment and cross-stream synchronization still need validation.
- Original calibration matches the intrinsics recorded above; `_01` RGB/depth
  frame IDs were inspected, and user confirmed the other camera checks work.
- Permanent optical TF setup now works: a child Xform under each color camera,
  named `<namespace>_color_optical_frame`, local translation (0,0,0), rotation
  (180,0,0) degrees, scale (1,1,1). Camera rendering transforms stay unchanged.
- Existing `/Graph/tf` publisher targets base, front/rear lasers and six optical
  Xforms, with empty parentPrim and staticPublisher unchecked. User's `/tf`
  output verified all six unique optical frames parented directly to `world`,
  with identical timestamps in that message. Original pose changes when driving.
- Raw camera target is unnecessary when directly publishing optical world poses.
  Camera graph Camera Prim must still target the actual color camera.
- Save the working USD with Ctrl+S and reopen that same file after restart.
  Exact working scene filename and save completion remain unconfirmed.
  No separate static-transform terminal is needed with this scene setup.
  Earlier suspended setup scripts are not the current working workflow.
- `capture_camera` now defaults to all six cameras in one process/session;
  `--cameras d455` selects one, or pass comma-separated namespaces for a subset.
  Each camera has separate queues, calibration, image-time TF lookup, motion
  filtering, counters and files under `capture_*/<namespace>/segment_*/`.
  Depth remains optional (`-p save_depth:=true`) and requires exact image stamps.
  Captures are independent per camera, not synchronized six-camera bundles.
- No real six-camera dataset, trainer conversion, training or export has yet
  been verified. Synthetic regression checks do not establish live synchronization.

## Six-camera Gaussian-splat preparation — 2026-10-08

- Recorded session: `/home/jovan/room_captures/capture_20261008T105933_495680Z`.
  Saved pairs by camera: 139, 134, 138, 78, 131, 146 (766 total).
  All manifest-referenced RGB/depth files exist. Sample files decode correctly.
  `_03` dropped 86 candidates without exact matching depth. No missing-TF
  drops appear in the saved counters. User accepted recording coverage.
- Existing tested environment discovered at
  `/home/jovan/miniforge3/envs/splatfacto`: Torch 2.7.1+cu128, Nerfstudio 1.1.5,
  gsplat 1.4.0, local CUDA 12.8 toolchain. Prior single-camera launch/export
  scripts and outputs are in `/home/jovan/splatfacto_runs`; preserve them.
- `scripts/convert_capture_splatfacto.py` combines one selected segment from all
  camera folders, copies RGB and emits per-image intrinsics/OpenGL camera-to-world
  poses. Conversion postmultiplies optical rotation by diag(1,-1,-1), retaining
  measured world translations. Zero distortion is required. No depth supervision
  or depth-based Gaussian initialization is used in this RGB baseline.
- Converted dataset: original capture path with `_splatfacto` suffix. Installed
  Nerfstudio parser verified 690 train and 76 validation views, finite poses.
- `scripts/train_six_cameras.sh` uses existing environment, fixed measured poses,
  random Gaussian initialization, CPU image cache, 30,000 steps and local viewer
  on port 7007. `SPLAT_STEPS` overrides iterations for a short check.
  Training must wait until user closes Isaac to release GPU memory.
  No six-camera training completion or quality result has yet been verified.

## SceneSplat preparation — 2026-10-08

- Six-camera training reached step 29999. Export:
  `/home/jovan/splatfacto_runs/exports/room_six_cameras/splat.ply` (102 MB,
  431091 Gaussians). Re-export with `scripts/export_six_cameras.sh`.
- SceneSplat checkout: `/home/jovan/SceneSplat`, inspection copy in
  `/tmp/lidar-scenesplat`. User wants custom-scene pretrained inference.
- Upstream `scripts/preprocess_gs.py` converted the PLY successfully into
  `/home/jovan/scenesplat_runs/input/room_six_cameras/` with coord/color/opacity/
  scale/quat arrays. All 431091 entries are finite; original PLY unchanged.
- Conda `scene_splat` cloned from `splatfacto` as an isolated base environment.
  SceneSplat dependencies/GPU kernels are NOT yet installed or verified.
  Upstream Torch2.5.1/CUDA12.4 requirements need adaptation for RTX5090;
  local working baseline is Torch2.7.1/CUDA12.8. No pip spconv-cu128 found.
- HF checkpoint repository reports gated=auto:
  https://huggingface.co/GaussianWorld/SceneSplat_lang-pretrain-concat-scan-ppv2-matt-mcmc-wo-normal-contrastive
  User was asked to accept access and download
  `lang-pretrain-concat-scan-ppv2-matt-mcmc-wo-normal-contrastive.pth`, then
  provide its local path. No checkpoint or inference completed yet.
- Upstream custom-scene entrypoint is `python -m tools.lang_inference` with
  `configs/inference/lang-pretrain-pt-v3m1-3dgs.py`. Outputs aligned features;
  PCA feature colors are not semantic object labels. Querying is a later step.

## SceneSplat checkpoint and runtime update — 2026-10-08

- User downloaded the 1.1 GB language checkpoint to `/home/jovan/SceneSplat/`
  with the full filename recorded above. Directory is case-sensitive.
- Installed inference dependencies only into isolated `scene_splat` environment:
  spconv-cu126 2.3.8, cumm-cu126 0.7.11, official FlashAttention2.8.3 wheel for
  cu12/Torch2.7/ABI TRUE/Python3.10, PyG scatter/cluster Torch2.7 cu128 wheels,
  torch-geometric, ocnn, addict, timm1.x, SharedArray, tensorboardx, yapf, ftfy,
  einops. Original Splatfacto environment remains unchanged.
- Small GPU attention and sparse-convolution forward checks passed on RTX5090.
- Local checkout compatibility edits: opt-in SCENESPLAT_INFERENCE_ONLY=1 in
  model registry imports only LangPretrainer and PTv3, hooks skip unrelated
  evaluator requiring pointops; inference wrapper removes unsupported
  return_backbone keyword (existing eval model returns point_feat naturally).
  Default training imports are retained when the variable is absent.
- Checkpoint loaded 393/393 keys. Full scene inference launched; completion
  must be checked before claiming output exists.
- Reusable launcher: `scripts/run_scenesplat.sh` (Conda activation before set -u).
  Output directory: `/home/jovan/scenesplat_runs/output/room_six_cameras/`.
  Runtime freeze and compatibility patch saved in `/home/jovan/scenesplat_runs/setup/`.
- Full inference completed successfully: `room_six_cameras_feat.pt` shape
  (431091,768), plus `room_six_cameras_pca_colored.ply` and renderable
  `room_six_cameras_feat_vis_3dgs.ply` in the output directory. No named object
  classification/text query has been performed yet.
- SceneSplat Mini Viewer installed from upstream recommended commit
  6c8e5c938844487319a92e19f952e76cd4eba847, with nerfview0.1.3 and splines.
  Launcher: `scripts/view_scenesplat.sh`, local-only http://127.0.0.1:8080.
  Server startup and HTTP200 verified. Viewer controls from installed source:
  Feature Map, Reset RGB, Query text / image / feature. Text encoder loads
  lazily; text queries have not been tested. Do not launch a duplicate server
  if port8080 is already serving the viewer.
- Viewer white-screen diagnosis: two instances on8080/8081, and initial camera
  offset (0,0,-6) caused a degenerate view/up configuration. Log showed Viser
  invalid normalization and singular linalg.inv during gsplat rendering.
  Local MiniViewer `site-packages/core/viewer.py` default camera now uses
  center+(3,-3,2), explicit Z-up; original/duplicate viewers stopped, one
  corrected instance restarted on8080. Browser view still requires confirmation.
- Follow-up blank-view fix: default viewpoint outside normalized reconstruction
  was replaced by recorded frame30, transformed using training dataparser
  translation/scale, saved in `/home/jovan/SceneSplat/.tmp/camera_state.json`.
- gsplat was triggering redundant concurrent JIT builds in shared Torch cache.
  Stopped viewer/compiler duplicates and reused the existing working
  py310_cu128/gsplat_cuda/gsplat_cuda.so by copying it into isolated scene_splat
  site-packages/gsplat/gsplat_cuda.so. Local cuda/_backend.py imports this module
  instead of csrc. PyTorch ABI/version matches Splatfacto. Original Splatfacto
  environment untouched; copied binary is specific to this local environment.
- Direct CUDA rendering from recorded pose succeeded and image was visually
  inspected: room wall/objects visible. Diagnostic `/tmp/check_scenesplat_view.py`
  outputs `/tmp/scenesplat-render-check.png`. Viewer restarted on8080.
  Launcher exports TORCH_CUDA_ARCH_LIST=12.0 for any future compilation.
- Orbit controls updated after user fell out of room while rotating. Saved pose
  restoration previously retained a distant browser look_at target. Local
  core/viewer.py now sets target 0.1 scene units along camera positive Z.
  Added Viewer > Orbit radius (0.01..1, default0.1) and Set nearby rotation point
  button, which retains current position/direction and changes target distance.
  User should hard-refresh8080; 0.05 gives a tighter orbit. Scene is normalized;
  radius units are not metres. Viser source confirms WASD keyboard navigation;
  earlier double-click-to-center advice was not verified and should not repeat.
- Text-query failure diagnosed from viewer log: missing transformers. Installed
  transformers4.50.1 and sentencepiece in scene_splat only. Viewer uses
  google/siglip2-base-patch16-512 (768D); first model download/cache initiated.
- Local actions/language_feature.py now displays query status/errors under the
  query button and automatically leaves feature/normal-map modes after success
  to reveal red RGB query highlights. Clears prior prune mask for a new query.
  Syntax checked; text encoder score test completion must be verified.
