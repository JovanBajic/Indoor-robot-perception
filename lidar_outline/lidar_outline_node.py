import math

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class LidarSplitMerge(Node):

    def __init__(self):
        super().__init__('lidar_split_merge')

        self.front_scan_topic = self.declare_parameter(
            'front_scan_topic', '/front_laser/scan'
        ).value
        self.back_scan_topic = self.declare_parameter(
            'back_scan_topic', '/rear_laser/scan'
        ).value

        self.front_subscription = self.create_subscription(
            LaserScan,
            self.front_scan_topic,
            lambda scan: self.scan_callback(scan, 'front'),
            qos_profile_sensor_data
        )
        self.back_subscription = self.create_subscription(
            LaserScan,
            self.back_scan_topic,
            lambda scan: self.scan_callback(scan, 'back'),
            qos_profile_sensor_data
        )

        marker_qos = QoSProfile(depth=20)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.bound_publisher = self.create_publisher(
            Marker,
            '/lidar_max_range',
            marker_qos
        )

        self.publisher = self.create_publisher(
            Marker,
            '/lidar_lines',
            marker_qos
        )

        # ---------------------------------------------
        # PARAMETERS
        # ---------------------------------------------

        # If consecutive LiDAR points are farther apart
        # than this, consider them separate surfaces.
        self.cluster_gap = self.declare_parameter(
            'cluster_gap', 0.20
        ).value

        # Maximum perpendicular distance from a fitted
        # line before we split the segment.
        self.split_threshold = self.declare_parameter(
            'split_threshold', 0.06
        ).value

        # Minimum number of points required for a line.
        self.min_points = self.declare_parameter(
            'min_points', 4
        ).value

        # Don't display very small line segments.
        self.min_line_length = self.declare_parameter(
            'min_line_length', 0.15
        ).value

        # Merge neighboring lines if their directions
        # differ by less than this.
        self.merge_angle_deg = self.declare_parameter(
            'merge_angle_deg', 8.0
        ).value

        # Maximum perpendicular offset between neighboring lines. This keeps
        # separate parallel surfaces from being merged into one diagonal line.
        self.merge_line_distance = self.declare_parameter(
            'merge_line_distance', 0.06
        ).value

        # Keep a rolling history to reduce visible scan-to-scan flashing.
        self.line_history_frames = self.declare_parameter(
            'line_history_frames', 5
        ).value
        self.validate_parameters()
        self.line_marker_ids = {
            'front': 0,
            'back': 0,
        }

        self.get_logger().info(
            'LiDAR Split-and-Merge started: '
            f'front={self.front_scan_topic}, back={self.back_scan_topic}'
        )

    def validate_parameters(self):
        """Reject invalid extraction settings before processing scans."""
        for name in ('cluster_gap', 'split_threshold', 'min_line_length',
                     'merge_line_distance'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(self.merge_angle_deg) or not 0 < self.merge_angle_deg <= 90:
            raise ValueError('merge_angle_deg must be in (0, 90]')
        for name, minimum in (('min_points', 2), ('line_history_frames', 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f'{name} must be an integer >= {minimum}')

    def scan_callback(self, scan, sensor_name='front'):

        # =================================================
        # 1. LASERSCAN -> XY POINTS
        # =================================================

        scan_points = []

        for i, r in enumerate(scan.ranges):

            angle = scan.angle_min + i * scan.angle_increment

            # Ignore invalid measurements.
            if (
                not math.isfinite(r)
                or r <= scan.range_min + 0.01
                or r >= scan.range_max - 0.01
            ):
                scan_points.append(None)
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            scan_points.append(
                np.array([x, y], dtype=float)
            )

        # =================================================
        # 2. RANGE-DISCONTINUITY CLUSTERING
        # =================================================

        clusters = []
        current_cluster = []

        for point in scan_points:

            if point is None:

                if len(current_cluster) >= self.min_points:
                    clusters.append(np.array(current_cluster))

                current_cluster = []
                continue

            if not current_cluster:
                current_cluster.append(point)
                continue

            previous = current_cluster[-1]

            distance = np.linalg.norm(point - previous)

            if distance <= self.cluster_gap:
                current_cluster.append(point)

            else:
                if len(current_cluster) >= self.min_points:
                    clusters.append(np.array(current_cluster))

                current_cluster = [point]

        if len(current_cluster) >= self.min_points:
            clusters.append(np.array(current_cluster))

        # =================================================
        # 3. SPLIT
        # =================================================

        all_lines = []

        for cluster in clusters:

            lines = self.split_segment(cluster)

            # =================================================
            # 4. MERGE
            # =================================================

            lines = self.merge_lines(lines)

            all_lines.extend(lines)

        # =================================================
        # 5. RVIZ MARKERS
        # =================================================

        # Replace one slot in this sensor's rolling marker history.
        marker = Marker()
        marker.header = scan.header
        marker.ns = f'lidar_split_merge_lines_{sensor_name}'
        marker.id = self.line_marker_ids[sensor_name]
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        if sensor_name == 'front':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.65
            marker.color.b = 1.0
        marker.color.a = 0.55

        for start, end in all_lines:

            length = np.linalg.norm(end - start)

            if length < self.min_line_length:
                continue

            p1 = Point()
            p1.x = float(start[0])
            p1.y = float(start[1])
            p1.z = 0.0

            p2 = Point()
            p2.x = float(end[0])
            p2.y = float(end[1])
            p2.z = 0.0

            marker.points.extend([p1, p2])

        # Empty scans clear this sensor's history instead of retaining stale walls.
        if marker.points:
            self.publisher.publish(marker)
            self.line_marker_ids[sensor_name] = (
                self.line_marker_ids[sensor_name] + 1
            ) % self.line_history_frames
        else:
            marker.action = Marker.DELETE
            for marker_id in range(self.line_history_frames):
                marker.id = marker_id
                self.publisher.publish(marker)
            self.line_marker_ids[sensor_name] = 0
        self.publish_range_bound(scan, sensor_name)

    def publish_range_bound(self, scan, sensor_name='front'):

        marker = Marker()

        marker.header = scan.header
        marker.ns = f'lidar_range_bound_{sensor_name}'
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Keep the boundary visible when the maximum range is tens of metres
        # away and RViz is zoomed out.
        marker.scale.x = 0.12

        marker.color.r = 1.0
        marker.color.g = 0.45 if sensor_name == 'back' else 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Draw the complete field-of-view boundary: start at the sensor,
        # follow the maximum-range arc, then return to the sensor. The radial
        # sides keep the marker easy to locate when the arc is far off-screen.
        samples = 300

        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        marker.points.append(origin)

        for i in range(samples + 1):

            t = i / samples

            angle = (
                scan.angle_min
                + t * (scan.angle_max - scan.angle_min)
            )

            p = Point()

            p.x = scan.range_max * math.cos(angle)
            p.y = scan.range_max * math.sin(angle)
            p.z = 0.0

            marker.points.append(p)

        marker.points.append(origin)

        self.bound_publisher.publish(marker)

    def split_segment(self, points):
        """
        Extract lines recursively using Split-and-Merge.

        Fit a line between the first and last point.
        Find the point with the largest perpendicular
        distance from that line.

        If the distance is too large:
            split there and recursively process both sides.

        Otherwise:
            accept the segment as a line.
        """
        if len(points) < self.min_points:
            return []

        start = points[0]
        end = points[-1]

        line = end - start
        line_length = np.linalg.norm(line)

        if line_length < 1e-6:
            return []

        # Unit direction vector.
        direction = line / line_length

        # Unit normal vector.
        normal = np.array([
            -direction[1],
            direction[0]
        ])

        # Perpendicular distance of every point
        # from the line.
        distances = np.abs(
            (points - start) @ normal
        )

        max_index = np.argmax(distances)
        max_distance = distances[max_index]

        # ---------------------------------------------
        # SPLIT
        # ---------------------------------------------

        if max_distance > self.split_threshold:
            # Keep any sufficiently sampled side; reject undersampled fragments
            # rather than accepting a line that exceeds the error threshold.

            left = self.split_segment(
                points[:max_index + 1]
            )

            right = self.split_segment(
                points[max_index:]
            )

            return left + right

        # ---------------------------------------------
        # ACCEPT LINE
        # ---------------------------------------------

        fitted_start, fitted_end = self.fit_line(points)

        return [(fitted_start, fitted_end)]

    def fit_line(self, points):
        """Fit the best line through a set of points using PCA."""
        center = np.mean(points, axis=0)

        centered = points - center

        _, _, vh = np.linalg.svd(
            centered,
            full_matrices=False
        )

        direction = vh[0]

        projections = centered @ direction

        minimum = np.min(projections)
        maximum = np.max(projections)

        start = center + minimum * direction
        end = center + maximum * direction

        # SVD may choose either sign for its direction vector. Preserve the
        # input scan order so merge_lines compares neighboring endpoints.
        scan_direction = points[-1] - points[0]
        if np.dot(end - start, scan_direction) < 0.0:
            start, end = end, start

        return start, end

    def merge_lines(self, lines):
        """Merge neighboring line segments with similar directions."""
        if len(lines) < 2:
            return lines

        merged = []

        current_start, current_end = lines[0]

        for next_start, next_end in lines[1:]:

            direction1 = current_end - current_start
            direction2 = next_end - next_start

            norm1 = np.linalg.norm(direction1)
            norm2 = np.linalg.norm(direction2)

            if norm1 < 1e-6 or norm2 < 1e-6:
                continue

            direction1 /= norm1
            direction2 /= norm2

            dot = np.clip(
                abs(np.dot(direction1, direction2)),
                -1.0,
                1.0
            )

            angle = math.degrees(
                math.acos(dot)
            )

            # Distance between the two segments.
            gap = np.linalg.norm(
                next_start - current_end
            )

            # Perpendicular distance from the next segment's near endpoint to
            # the current infinite line.
            relative_start = next_start - current_start
            line_offset = abs(
                direction1[0] * relative_start[1]
                - direction1[1] * relative_start[0]
            )

            # ---------------------------------------------
            # MERGE
            # ---------------------------------------------

            if (
                angle < self.merge_angle_deg
                and gap < self.cluster_gap
                and line_offset < self.merge_line_distance
            ):

                combined = np.array([
                    current_start,
                    current_end,
                    next_start,
                    next_end
                ])

                current_start, current_end = self.fit_line(
                    combined
                )

            else:

                merged.append(
                    (current_start, current_end)
                )

                current_start = next_start
                current_end = next_end

        merged.append(
            (current_start, current_end)
        )

        return merged


def main(args=None):

    rclpy.init(args=args)

    node = LidarSplitMerge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
