# Repository and Isaac Sim handoff

Last updated: 2026-10-02. This file records verified session findings and
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
