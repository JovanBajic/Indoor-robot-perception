# [IN PROGRESS] Indoor robot perception with Isaac Sim and ROS 2
 
A robotics perception project using an RB-KAIROS mobile robot with a Franka arm to scan indoor environments in NVIDIA Isaac Sim. This repository provides ROS 2 tools for 2D LiDAR outlines, keyboard driving and timestamped camera capture. The intended next stages are 3D Gaussian Splatting reconstruction, SceneSplat semantic segmentation, object detection and detection of objects moving during simulation.

The project is under active development. Current work focuses on simulation-based sensing, LiDAR geometry extraction and camera data collection; reconstruction and scene understanding form the next development stages.

## Current implementation

| Component | Functionality |
| --- | --- |
| Dual LiDAR processing | Range filtering, clustering, recursive split-and-merge and PCA line fitting |
| RViz visualization | Separate front/rear outlines, sensor range boundaries and rolling marker history |
| Keyboard teleoperation | Configurable linear/angular commands, combined key input and stop on focus loss |
| Isaac Sim integration | Mobile-base drive and front/rear LiDAR demonstrated during development; D455 RGB/depth publication observed |
| Camera capture tooling | Timestamped RGB, calibration and measured TF recording, optional depth and simulation-reset segmentation; tested with synthetic ROS messages |

The D455 simulation setup has produced 1280 × 720 CameraInfo with fx = fy approximately 634.086 pixels and zero distortion. Camera capture requires correctly configured optical TF and image-time transforms, as described below.

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
| Optional depth | `/d455/depth/imageraw` (configurable recorder default) | Image |
| Recorder pose/time | `/tf`, `/tf_static`, `/clock` | TFMessage, Clock |

## Repository structure

| Path | Purpose |
| --- | --- |
| [lidar_outline/](lidar_outline/) | Extraction, teleop and camera capture Python nodes |
| [launch/lidar_outline.launch.py](launch/lidar_outline.launch.py) | Extraction and RViz with `use_sim_time=True` |
| [test/](test/) | Extraction/capture regression tests and generated ament lint tests |
| [resource/lidar_outline](resource/lidar_outline) | ament package-index marker |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependencies, installed entry points and executable locations |
| [AGENTS.md](AGENTS.md) | Simulation development notes |
| [.gitignore](.gitignore) | Generated builds, caches and capture/training outputs |

## Requirements

Development environment: Ubuntu, ROS 2 Jazzy and Isaac Sim 6.0.0. Locally inspected Python dependencies: Python 3.12.3, NumPy 1.26.4, Pillow 10.2.0 and Tk 8.6.

Runtime dependencies in [package.xml](package.xml): `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_msgs`, `rosgraph_msgs`, `launch`, `launch_ros`, `rviz2`, NumPy, Pillow and Tkinter. Python packaging declares setuptools and NumPy; pip installation alone does not supply ROS, Pillow or Tkinter. Use the ROS workspace installation below. Teleop/RViz/Isaac require a graphical desktop. Isaac requires a supported GPU and a separate simulator installation.

## Installation

Install ROS 2 Jazzy, rosdep and colcon separately. Place this package at `~/ros2_ws/src/lidar_outline`, or the equivalent location in your workspace. From a ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lidar_outline
source install/setup.bash
```

In each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Launch simulation and ROS components

The ROS package runs alongside an Isaac Sim room scene containing the robot, sensor publishers and drive graph. Open your configured scene before starting the ROS components. The robot USDA uses companion payload directories, which should remain together when moving the asset.

For a local Isaac installation at `~/isaacsim`:

```bash
~/isaacsim/isaac-sim.sh
```

1. Use **File → Open** in Isaac to open your saved working room/ROS scene.
2. Keep D455's `Imu_Sensor` child deactivated. This avoids the playback crash observed with that sensor active.
3. Preserve D455 placement/internal transforms and keep its inner `RSD455` rigid body disabled. Preserve the working robot joints and LiDAR setup.
4. Press Play beside the viewport (the large 3D room window).

The simulation drive configuration uses ROS2 Subscribe Twist → Differential Controller → front/rear Articulation Controllers. Wheel radius: 0.11 m; wheel distance: 0.409 m. Playback Tick Delta Seconds must feed controller Dt. Configure these connections in Isaac. Do not re-enable the unwanted `box_1` collider. Editing repository files does not update the open Isaac scene.

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

The D455 color camera prim is `/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/summit_xl_base_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color`. The installed shortcut **Tools → Robotics → ROS 2 OmniGraphs → Camera** adds publishing in Isaac, not this ROS package.

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

Replace the topic placeholder with your published depth topic. NPY retains raw floats and invalid depth values; verify units and alignment before reconstruction. Other parameters include `rgb_topic`, `info_topic`, `world_frame`, `camera_frame`, `max_fps`, `min_translation`, `min_rotation_deg` and `wait_seconds`.

### Processing direction

The recorder produces calibrated observations with camera-to-world poses for subsequent dataset conversion. The planned reconstruction stage will map this format to a Gaussian Splatting trainer, preserving intrinsics, timestamps and camera conventions.

## Testing

Regression tests cover extraction parameter validation, splitting/merging, marker clearing, padded and big-endian depth decoding, timestamped TF capture, stationary-view filtering and simulation-reset segmentation.

From the package directory, after sourcing ROS:

```bash
python3 -m pytest test -q
```

Latest local result: **13 passed, 1 skipped** under ROS Jazzy. These checks exercise the algorithms and recorder using synthetic messages; simulator behavior is evaluated separately during development.

## Engineering considerations

LiDAR markers retain scan timestamps and frames for TF-based visualization. Short undersampled features can be rejected, and merging refits segment endpoints. Camera capture uses bounded queues, timestamp-specific transforms and drop counters to handle asynchronous sensor delivery. Optional depth pairing requires exact timestamp agreement. Optical-frame conventions and depth alignment must be checked in the simulation configuration before collecting reconstruction data.

## Roadmap

1. Verify optical TF, image-time measured poses, depth units/alignment and shared simulation clock/reset behavior; demonstrate a small capture.
2. Record overlapping room views; inspect coverage and dropped-frame counters.
3. Select a Gaussian Splatting trainer, implement conversion, train, inspect and export with reproducible results.
4. Integrate SceneSplat semantic features/segmentation after verifying Gaussian inputs, models and licenses.
5. Implement/evaluate object identification, instance grouping and localization from semantic output as separate detection work.
6. Compare observations in a common measured coordinate frame to detect moved objects; evaluate false changes caused by pose/rendering errors.

## References and acknowledgements

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) provides messaging, TF, launch and RViz. Generated ament lint tests retain Open Source Robotics Foundation copyright and Apache-2.0 notices.
- [NVIDIA Isaac Sim 6.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/) provides physics simulation, sensor rendering and ROS bridging. Simulator components and assets are subject to their respective [licensing terms](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/common/license-faq.html).
- Robotnik RB-KAIROS/Summit XL, the Franka arm and the RealSense D455 provide the robot and sensor context. Third-party simulation assets remain subject to their original licenses and are maintained separately from this ROS package.
- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting) provides the research basis for the planned reconstruction stage.
- [SceneSplat](https://github.com/unique1i/SceneSplat) is the intended basis for future semantic scene understanding.
- NumPy, Pillow and Tkinter support numerical processing, image storage and keyboard interaction.
