# lidar_outline

`lidar_outline` is a ROS 2 Python package that extracts straight-line segments
from a 2D LiDAR scan and publishes the result as RViz markers. It is intended to
turn walls and other flat surfaces in a `sensor_msgs/msg/LaserScan` into a simple
line-based outline.

## Data flow

```text
/front_laser/scan or /rear_laser/scan (LaserScan)
        |
        v
remove invalid and near-limit ranges
        |
        v
convert polar samples to XY points
        |
        v
cluster neighboring points by distance
        |
        v
recursively split clusters at large line errors
        |
        v
fit segments with PCA and merge near-collinear neighbors
        |
        +--> /lidar_lines     (green LINE_LIST marker)
        `--> /lidar_max_range (red LINE_STRIP marker)
```

The published markers use the timestamp and coordinate frame from the incoming
scan. No TF transformation is performed by this package.

## Algorithm

The node performs the following operations for every scan:

1. It converts each finite, in-range measurement from polar coordinates to
   `(x, y)`. Invalid measurements remain gaps in the scan.
2. It groups consecutive points while their Euclidean separation is at most
   `0.20 m`. Gaps and invalid measurements end a cluster.
3. For each cluster, it draws a candidate line through the first and last point.
   If an intermediate point is more than `0.06 m` from that line, it recursively
   splits the cluster at the point with the largest error.
4. It fits accepted segments with PCA/SVD, so the endpoints lie on the dominant
   direction and span the projected input points.
5. It merges adjacent segments when their directions differ by less than eight
   degrees, their endpoint gap is less than `0.20 m`, and their perpendicular
   offset is less than `0.06 m`.
6. It publishes segments at least `0.15 m` long as green RViz markers.

At least four points are required for a cluster or recursively split segment.
The node retains a rolling history of five nonempty outlines per sensor. If a
scan produces no displayed lines, it clears that sensor’s history. The
maximum-range boundary is still published.

## ROS interface

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Subscribes | `/front_laser/scan` | `sensor_msgs/msg/LaserScan` | Front scan (configurable) |
| Subscribes | `/rear_laser/scan` | `sensor_msgs/msg/LaserScan` | Rear scan (configurable) |
| Publishes | `/lidar_lines` | `visualization_msgs/msg/Marker` | Front green and rear blue line segments |
| Publishes | `/lidar_max_range` | `visualization_msgs/msg/Marker` | Red/orange sensor range boundaries |

The scan subscription uses ROS 2's sensor-data QoS. Marker publishers are
reliable and transient-local with depth 20, allowing a newly opened RViz display
to receive the latest marker.

## Build and run

This package is expected to live in the `src` directory of a ROS 2 workspace.
It requires ROS 2 packages `rclpy`, `sensor_msgs`, `geometry_msgs`, and
`visualization_msgs`, plus Python 3 and NumPy.

From the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lidar_outline
source install/setup.bash
ros2 run lidar_outline lidar_outline
```

To start both the processing node and RViz with Isaac simulation time enabled
by default, use the included launch file:

```bash
ros2 launch lidar_outline lidar_outline.launch.py
```

NumPy and Tkinter are declared as runtime dependencies in the package manifest.

To use a scan topic with a different name, remap the input:

```bash
ros2 run lidar_outline lidar_outline \
  --ros-args -r /front_laser/scan:=/scan
```

The scan topics can also be selected with parameters:

```bash
ros2 run lidar_outline lidar_outline --ros-args \
  -p front_scan_topic:=/front_laser/scan \
  -p back_scan_topic:=/rear_laser/scan
```

Both sensors share the output topics but use separate marker namespaces, so
RViz displays them simultaneously. Their scan frames must be connected to the
RViz fixed frame through TF.

## WASD driving

After building the package, open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline wasd_teleop
```

Keep the keyboard window focused. Hold W/S to drive forward/backward, A/D to
turn, or Space to stop. Releasing the keys or leaving the window sends a zero
velocity command. Escape closes the window. The node publishes
`geometry_msgs/msg/Twist` on `/cmd_vel`; Isaac Sim must be playing with a
subscriber connected to its wheel controller. The window shows the subscriber
count. Tkinter (`python3-tk`) and a graphical desktop are required.

Optional speed limits and topic remapping:

```bash
ros2 run lidar_outline wasd_teleop --speed 0.25 --turn 0.5 \
  --ros-args -r cmd_vel:=/cmd_vel
```

## RViz setup

1. Set RViz's fixed frame to the frame in the incoming `LaserScan` header, or to
   a frame connected to it by TF.
2. Add a **Marker** display for `/lidar_lines`.
3. Optionally add another **Marker** display for `/lidar_max_range`.

## Current tuning values

The tuning values are ROS parameters and can be overridden at startup.

| Setting | Value | Effect |
| --- | ---: | --- |
| `cluster_gap` | `0.20 m` | Maximum separation between consecutive points in one surface cluster |
| `split_threshold` | `0.06 m` | Maximum point-to-line error before recursive splitting |
| `min_points` | `4` | Minimum samples in a cluster or split segment |
| `min_line_length` | `0.15 m` | Minimum displayed segment length |
| `merge_angle_deg` | `8.0 deg` | Maximum direction difference for merging |
| `merge_line_distance` | `0.06 m` | Maximum perpendicular offset for merging |
| `line_history_frames` | `5` | Number of recent outlines retained per sensor |

Increase `cluster_gap` to bridge wider sample spacing, but note that this can
join separate objects. Increase `split_threshold` for fewer, smoother segments;
decrease it to follow corners and deviations more closely. Noisy scans may need
a larger threshold or preprocessing. Decrease `line_history_frames` to reduce overlapping historical outlines.
Parameters are read at startup and validated before processing scans.

For example, to produce fewer, smoother lines without rebuilding:

```bash
ros2 run lidar_outline lidar_outline --ros-args \
  -p split_threshold:=0.08 \
  -p merge_angle_deg:=10.0 \
  -p min_line_length:=0.20
```

## Repository layout

```text
lidar_outline/
├── lidar_outline/
│   ├── __init__.py
│   └── lidar_outline_node.py  # Node, extraction algorithm, and marker output
├── resource/lidar_outline     # ament resource-index marker
├── test/                      # generated ament lint tests
├── package.xml                # ROS package manifest
├── setup.cfg                  # ROS script install paths
└── setup.py                   # Python package and console entry point
```

## Known limitations and useful next steps

- The extracted geometry is available only as visualization markers, not as a
  dedicated line-segment message for downstream navigation or mapping nodes.
- Merging refits segment endpoints rather than all original measurements, so
  merged lines do not have a guaranteed maximum residual error.
- Splitting rejects fragments with fewer than `min_points` samples, which can
  omit short features near corners.
- Regression tests cover parameter validation, splitting, merging, and clearing
  stale markers; live Isaac/RViz behavior still needs integration verification.
- `setup.py` and `package.xml` still contain placeholder license
  and maintainer email metadata.

The main implementation is in
[`lidar_outline/lidar_outline_node.py`](lidar_outline/lidar_outline_node.py).
