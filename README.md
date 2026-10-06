# Indoor robot perception with Isaac Sim and ROS 2

Work in progress: an RB-KAIROS mobile robot with a Franka arm scans an indoor environment in NVIDIA Isaac Sim. This repository provides ROS 2 tools for 2D LiDAR outlines, keyboard driving and timestamped camera capture. The intended next stages are 3D Gaussian Splatting reconstruction, SceneSplat semantic segmentation, object detection and detection of objects moving during simulation.

**Reconstruction, semantic inference and change detection are planned, not implemented.** This is one `ament_python` package, `lidar_outline`, rather than a complete simulation distribution. There are no C++ sources or bundled scenes.

## Current status

| Component | Status | Evidence and limits |
| --- | --- | --- |
| Dual LiDAR line extraction | Implemented | Filtering, clustering, split-and-merge, PCA fitting and RViz markers; regression tests included |
| WASD driving | Implemented | Tkinter window publishes Twist and stops on focus loss/key release |
| Isaac drive and front/rear LiDAR | Previously observed working | Session handoff in [AGENTS.md](AGENTS.md); not reproduced during this review |
| D455 RGB/depth publishing | Previously observed working | Handoff records topics and CameraInfo; live depth spelling, units and alignment need rechecking |
| Camera recorder | Implemented; integration incomplete | RGB/calibration/exact-time TF and optional depth; synthetic-message tests, no captured dataset in the repository |
| Measured optical camera TF | Unverified prerequisite | Handoff reported missing optical frame; no verified axis conversion or live capture evidence included |
| Gaussian Splatting, SceneSplat, object/change detection | Planned | No converter, trainer, semantic inference or detector code present |

The prior handoff records 1280 × 720 CameraInfo, fx = fy approximately 634.086 pixels, and zero distortion. It records approximately 24 Hz RGB and 26 Hz depth subscriber arrival rates. These are historical observations, not a benchmark, proof of synchronization or configured sensor frequency. No reconstruction or semantic quality metrics are available.

## Demo

No screenshots or videos are included. A useful first demo would show the moving robot in Isaac, front/rear outlines in RViz and the D455 view following the robot. Add an actual recording and its setup description before claiming reproducible end-to-end results.

## System architecture

```text
WASD window -> cmd_vel (Twist) -> Isaac BaseDrive/wheel controllers
Isaac LiDAR -> front/rear LaserScan -> lidar_outline -> RViz markers
Isaac clock ---------------------------------------> outline/RViz time

Isaac D455 RGB + CameraInfo + optional depth --+
Measured TF + dedicated optical frame --------+-> capture_camera
Isaac clock ----------------------------------+       |
                                                      v
                                         PNG + JSONL (+ depth NPY)
                                                      |
                                             planned conversion
                                                      |
                                planned 3DGS -> SceneSplat -> change detection
```

[The extraction node](lidar_outline/lidar_outline_node.py) converts finite in-range polar samples to XY, groups consecutive points by distance, recursively splits at excessive perpendicular error, fits lines with PCA/SVD and merges neighboring near-collinear segments. Defaults: 0.20 m cluster gap, 0.06 m split/merge-offset thresholds, four samples, 0.15 m minimum displayed length, eight degrees merge angle and five historical outlines per sensor. Parameters are read and validated at startup. Front lines are green, rear lines blue; range boundaries are red/orange.

Markers retain the scan frame and timestamp. The package does not transform scans into a global map; RViz uses external TF. Marker QoS is reliable/transient-local, depth 20; scan subscriptions use sensor-data QoS.

[Teleop](lidar_outline/wasd_teleop.py) publishes at a nominal 20 Hz using the GUI wall-clock timer, even when simulation is paused. Displayed velocity is a **command**, not measured motion. Camera capture uses measured TF, not command integration.

| Interface | Default topic | Message |
| --- | --- | --- |
| LiDAR inputs | `/front_laser/scan`, `/rear_laser/scan` | LaserScan |
| Outline outputs | `/lidar_lines`, `/lidar_max_range` | Marker |
| Drive command | `/cmd_vel` (relative `cmd_vel` in teleop) | Twist |
| Camera inputs | `/d455/rgb/image_raw`, `/d455/camera_info` | Image, CameraInfo |
| Optional depth | `/d455/depth/imageraw` (code default; live spelling unresolved) | Image |
| Recorder pose/time | `/tf`, `/tf_static`, `/clock` | TFMessage, Clock |

## Repository structure

| Path | Purpose |
| --- | --- |
| [lidar_outline/](lidar_outline/) | Extraction, teleop and camera capture Python nodes |
| [launch/lidar_outline.launch.py](launch/lidar_outline.launch.py) | Extraction and RViz with `use_sim_time=True`; does not start Isaac, teleop or capture |
| [test/](test/) | Extraction/capture regression tests and generated ament lint tests |
| [resource/lidar_outline](resource/lidar_outline) | ament package-index marker |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependencies, installed entry points and executable locations |
| [AGENTS.md](AGENTS.md) | Local handoff, historical observations and machine-specific paths |
| [.gitignore](.gitignore) | Generated builds, caches and capture/training outputs |

No RViz configuration, YAML parameter file, scene, demo, dataset or checkpoint is included. The local `scripts/` directory is empty and untracked.

## Requirements

The handoff specifies Ubuntu, ROS 2 Jazzy and Isaac Sim 6.0.0. Jazzy and the local Isaac launcher were found; Isaac's GUI was not launched to independently confirm its version. Ubuntu release, GPU, driver and CUDA versions are undocumented. Locally inspected versions: Python 3.12.3, NumPy 1.26.4, Pillow 10.2.0 and Tk 8.6. These are observations, not pinned requirements or a compatibility matrix.

Runtime dependencies in [package.xml](package.xml): `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_msgs`, `rosgraph_msgs`, `launch`, `launch_ros`, `rviz2`, NumPy, Pillow and Tkinter. Python packaging declares setuptools and NumPy; pip installation alone does not supply ROS, Pillow or Tkinter. Use the ROS workspace installation below. Teleop/RViz/Isaac require a graphical desktop. Isaac requires its own supported hardware/installation; no training environment is selected.

## Installation

Install ROS 2 Jazzy, rosdep and colcon separately. Place this package at `~/ros2_ws/src/lidar_outline`, or the equivalent location in your workspace. From a ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lidar_outline
source install/setup.bash
```

Commands match the package layout and entry points. Dependency installation and workspace build were not rerun in this review. In each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Launch simulation and ROS components

**Local setup; not reproducible from this repository alone:** the user identified `~/Desktop/rbkairos_franka_isaac/rbkairos_franka_isaac.usda`. Inspection confirms a robot asset referencing `payloads/base.usda` and physics payloads; retain its companion directories. It is not the full room/ROS scene. A separate binary scene exists at `~/Desktop/rbkairos_franka_project/rbkairos_franka_ros.usd`; whether it is the latest working scene and resolves all assets is unverified. Neither is bundled.

This launcher exists locally but was not executed during review:

```bash
~/isaacsim/isaac-sim.sh
```

1. Use **File → Open** in Isaac to open your saved working room/ROS scene.
2. Keep D455's `Imu_Sensor` child deactivated. The handoff isolated it as the playback crash trigger; the native crash cause is unknown.
3. Preserve D455 placement/internal transforms and keep its inner `RSD455` rigid body disabled. Preserve the working robot joints and LiDAR setup.
4. Press Play beside the viewport (the large 3D room window).

The prior working drive graph is ROS2 Subscribe Twist → Differential Controller → front/rear Articulation Controllers. Wheel radius: 0.11 m; wheel distance: 0.409 m. Playback Tick Delta Seconds must feed controller Dt. These are handoff settings, not a graph created by this package. Do not re-enable the unwanted `box_1` collider. Editing repository files does not update the open Isaac scene.

Start extraction and RViz:

```bash
ros2 launch lidar_outline lidar_outline.launch.py
```

In RViz set **Fixed Frame** to `world` if sensor TF connects to it; otherwise use a connected scan frame. Add **Marker** displays for `/lidar_lines` and `/lidar_max_range`. The launch does not configure these displays.

In another sourced terminal:

```bash
ros2 run lidar_outline wasd_teleop --speed 0.3 --turn 1.2
```

Focus the WASD window: W/S drive, A/D turn, combinations work, Space stops while held, Escape exits. Without options, speed/turn default to 0.25 m/s and 1.0 rad/s. Subscriber count shows a ROS connection, not measured movement.

Run extraction alone or tune parameters:

```bash
ros2 run lidar_outline lidar_outline --ros-args -p use_sim_time:=true \
  -p split_threshold:=0.08 -p merge_angle_deg:=10.0
```

Input topic parameters are `front_scan_topic` and `back_scan_topic`; ROS remapping also works.

## Data collection and processing

### Verify camera prerequisites

While Isaac is playing, inspect actual streams rather than assuming depth topic spelling:

```bash
ros2 topic list
ros2 topic info /d455/rgb/image_raw
ros2 topic echo /d455/camera_info --once
ros2 run tf2_ros tf2_echo world d455_color_optical_frame
```

The handoff color prim is `/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/summit_xl_base_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color`. The installed shortcut **Tools → Robotics → ROS 2 OmniGraphs → Camera** adds publishing in Isaac, not this ROS package. Its live graph settings were not verified here.

Assigning a CameraInfo frame ID does not create TF. Verify measured camera pose changes while driving and agrees with the image. USD camera axes (+X right, +Y up, -Z forward) differ from ROS optical axes (+X right, +Y down, +Z forward). Establish conversion in a dedicated TF frame/pose conversion; do not rotate the working D455 asset. Identity TF is valid only if its parent already has the correct optical axes. Avoid conflicting TF parents. tf2_echo alone does not establish axis correctness or image-time TF availability.

Verify RGB encoding, depth units/encoding/alignment, matching frames and shared simulation timestamps. Depth rendered from the RGB viewpoint is simulated depth, not physical stereo reconstruction.

### Capture

After prerequisites are satisfied, run in a ROS terminal, not Isaac's Script Editor:

```bash
ros2 run lidar_outline capture_camera
```

Drive slowly through overlapping views, then Ctrl+C. Each run creates a unique directory under `~/room_captures`:

```text
capture_*/
  capture.json               # configuration and conventions
  status.json                # counters
  segment_000/
    frames.jsonl             # metadata per saved frame
    rgb/000000.png
    depth/000000.npy         # optional
```

[The recorder](lidar_outline/capture_camera.py) selects at most two candidates per simulation second by default. A view is saved after at least 5 cm translation or five degrees rotation from the last saved view. TF is looked up at the image timestamp, with interpolation allowed and no latest-pose fallback. Missing data times out after two wall-clock seconds; bounded queues/counters report drops. CameraInfo is the latest calibration at or before the image stamp, assuming calibration stays fixed between messages.

Records contain integer `stamp_ns`, image paths/dimensions, K/D/R/P, distortion model, calibration timestamp, camera-to-world position in metres and quaternion in xyzw order. Optical axes are assumed from upstream TF: the recorder does not validate or convert physical axes. Backwards `/clock` jumps start a new segment, clear pending/calibration/dynamic TF and retain static TF. Keep `/clock` publishing and check reset behavior across publishers.

Optional depth requires exact RGB/depth timestamp equality, matching dimensions/frame and `32FC1`; RGB must be `rgb8`. Replace the topic placeholder after inspecting live topics:

```bash
ros2 run lidar_outline capture_camera --ros-args \
  -p save_depth:=true -p depth_topic:=/REPLACE_WITH_VERIFIED_DEPTH_TOPIC \
  -p output_dir:=/tmp/room_capture_test
```

This example is **not runnable unchanged**. NPY retains raw floats/invalid depth; units and alignment remain unverified. Other parameters include `rgb_topic`, `info_topic`, `world_frame`, `camera_frame`, `max_fps`, `min_translation`, `min_rotation_deg` and `wait_seconds`.

### Processing (planned)

Capture is an intermediate format, not a trainer-ready dataset. No converter, training, semantic inference or export command exists. Select a trainer based on the available GPU/software, then verify camera conventions, intrinsics, dataset layout and export compatibility before conversion.

## Validation and known limitations

The documentation review inspected source, manifest, launch, tests and external USDA references. Existing tests exercise extraction validation/splitting/merging, stale markers, padded/big-endian depth, timestamped TF capture, stationary filtering and reset segmentation. These do not prove live integration, synchronization or optical axes. Isaac playback and live ROS sensor execution were not tested; no large dependencies were installed.

From the package directory, after sourcing ROS:

```bash
python3 -m pytest test -q
```

Review result: **13 passed, 1 skipped** under the local ROS Jazzy environment. The copyright test is skipped because project source headers are missing. Existing third-party test notices are preserved. Relative documentation links, Python syntax, package XML, ignore rules and `git diff --check` also passed. The lint runner emitted Python fork warnings.

LiDAR outputs are visualization markers, not a persistent map or line-segment API. Undersampled short features can be omitted; merging refits endpoints rather than all original samples, without a guaranteed residual bound. Historical outlines can overlap during motion. Capture performs synchronous disk I/O and has no live performance evaluation. Exact depth matching can drop frames. External scenes/calibration scripts make full simulation reproduction incomplete.

## Roadmap

1. Verify optical TF, image-time measured poses, depth units/alignment and shared simulation clock/reset behavior; demonstrate a small capture.
2. Record overlapping room views; inspect coverage and dropped-frame counters.
3. Select a Gaussian Splatting trainer, implement conversion, train, inspect and export with reproducible results.
4. Integrate SceneSplat semantic features/segmentation after verifying Gaussian inputs, models and licenses.
5. Implement/evaluate object identification, instance grouping and localization from semantic output as separate detection work.
6. Compare observations in a common measured coordinate frame to detect moved objects; evaluate false changes caused by pose/rendering errors.

## References and acknowledgements

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) supplies ROS interfaces, TF, launch and RViz. The three generated ament lint tests preserve Open Source Robotics Foundation copyright and Apache-2.0 notices; those notices do not license this entire project.
- [Isaac Sim 6.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/) provides simulation/ROS bridging. Review its [license FAQ](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/common/license-faq.html) and [additional software/materials terms](https://docs.isaacsim.omniverse.nvidia.com/latest/common/license-isaac-sim-additional.html) before distributing simulator components or NVIDIA assets; source-code licensing alone does not establish asset redistribution rights.
- External robot assets represent Robotnik RB-KAIROS/Summit XL and a Franka arm with `fr3_*` links. USDA metadata records URDF USD Converter v0.1.2, but the original model URL/revision/license is missing. Robot, room and D455 asset provenance/redistribution permissions remain unresolved. Existing Orbbec geometry is not the configured D455. No assets were copied into this repository.
- [3D Gaussian Splatting reference implementation](https://github.com/graphdeco-inria/gaussian-splatting) is background for planned reconstruction, not a selected dependency.
- [SceneSplat](https://github.com/unique1i/SceneSplat) is the intended semantic integration; no upstream code, weights or datasets are included.
- NumPy, Pillow and Tkinter support the Python tools. Other borrowed application code could not be established from source notices; absent notices do not prove authorship.

No license has been selected for the project's own code. `package.xml` and `setup.py` retain license and maintainer-email placeholders. Choose a license and public contact address before inviting reuse; retain upstream notices independently.

## Before sharing

- Add an actual demo and verified scene setup/download instructions, including asset sources and permissions.
- Verify live camera TF/capture and provide a small shareable sample or manifest.
- Document Ubuntu/GPU/driver versions and the latest working room scene.
- Choose your code license and replace placeholder contact metadata.
- Review already tracked `AGENTS.md`: it contains a local username and absolute machine paths. Ignore rules cannot hide tracked files.
- Keep the recorder, capture tests, dependency declarations and executable entry point together when sharing changes.

Ignore rules target generated output directories instead of all images/USD/NPY assets. No tracked files exceed 5 MB; the largest source is approximately 15 KB. External robot geometry is approximately 8.65 MB and untracked. A targeted current-file scan found no tracked private-key/provider-token/credential-assignment patterns; this was not a full Git-history audit. Machine-specific absolute paths occur in documentation rather than executable Python/launch files.
