# Indoor robot mapping and object navigation

An RB-KAIROS robot in Isaac Sim maps a room, reconstructs it as a 3D Gaussian
splat, and navigates toward objects found with a text prompt. ROS 2 Jazzy
connects the simulated LiDAR, six RealSense D455 cameras, RViz, and Nav2.

## Demo

<video src="demo/video.mp4" controls width="800"></video>

[Open or download the demo video](demo/video.mp4)

### 1. Map and record the room

The robot is driven through the room while its front and rear LiDAR scans build
an occupancy map. At the same time, six D455 cameras record paired RGB and
depth images with their measured poses for 3D reconstruction.

### 2. Reconstruct and understand the room

Depth initializes a Gaussian splat, which is trained with MCMC using the
recorded images and camera poses. The separate, upstream
[SceneSplat](https://github.com/unique1i/SceneSplat) project then extracts
semantic features from the reconstruction so objects can be found with text
queries.

### 3. Prompt and navigate

A prompt such as `plant` finds candidate object locations in the reconstructed
room and shows them in RViz. The navigation bridge checks a reachable approach
against the saved map and robot footprint, previews the route, and sends a Nav2
goal to drive the robot toward the selected candidate.

## How to run the demo

Open the matching six-camera room scene in Isaac Sim and press **Play**. Use a
ROS 2 Jazzy terminal with the workspace sourced. Run the following stages in
order; keep long-running commands in their own terminals.

**1. Record and map.** Drive the robot manually. Press **Ctrl-C in this terminal
while Isaac is still playing**; wait for `Saved demo capture and map` before
stopping Isaac.

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh record
```

**2. Train and process.** Close Isaac and any viewer to free GPU memory. This
trains the MCMC splat, runs SceneSplat, and opens the feature viewer.

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh pipeline
```

**3. Start navigation.** Close the viewer, reopen the same Isaac room, and press
**Play**. This starts the saved-map navigation stack and connects Nav2 to the
robot drive.

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh navigate
```

**4. Query an object.** Once navigation reports `Ready`, run this in another
terminal. Replace `plant` with the object you want to find.

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/demo.sh query plant
```

With one candidate, route checking starts automatically. With several, click a
numbered box in RViz within five seconds to choose it; otherwise candidate #1
is used. Watch the robot while it moves. To cancel, call
`ros2 service call /semantic_navigation/cancel std_srvs/srv/Trigger '{}'`.

The wrapper remembers the matching capture, map, and reconstruction paths. See
[WORKFLOW.md](WORKFLOW.md) for setup, individual commands, and troubleshooting.

## Credit

This repository integrates the external [SceneSplat project](https://github.com/unique1i/SceneSplat)
for semantic features; its source code and environment are maintained outside
this repository. SceneSplat is by Yue Li, Qi Ma, Runyi Yang, and collaborators.
See its upstream repository for the full author list, paper, and license.
