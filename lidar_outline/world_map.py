"""Collect a world-aligned occupancy map using timestamped measured laser TF.

This uses simulator ground truth rather than estimating poses with SLAM.
It publishes no velocity commands. Save /nav/map with Nav2 map_saver_cli.
"""

import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def rotation_matrix(quaternion):
    """Return the full 3D rotation, including upside-down laser mounting."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid laser quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def ray_cells(start, end):
    """Rasterize a segment in cell coordinates, including both endpoints."""
    count = int(max(abs(end[0]-start[0]), abs(end[1]-start[1]))) + 1
    return np.rint(np.linspace(start, end, count)).astype(int)


class WorldMap(Node):
    def __init__(self):
        super().__init__('world_map', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.frame = self.declare_parameter('world_frame', 'world').value
        self.resolution = float(self.declare_parameter('resolution', .05).value)
        size = float(self.declare_parameter('size_metres', 40.).value)
        self.origin = np.array([
            self.declare_parameter('origin_x', -20.).value,
            self.declare_parameter('origin_y', -20.).value], dtype=float)
        self.max_range = float(self.declare_parameter('max_range', 10.).value)
        topics = self.declare_parameter(
            'scan_topics', ['/nav/front_scan', '/nav/rear_scan']).value
        if (not np.isfinite([self.resolution, size, self.max_range, *self.origin]).all()
                or min(self.resolution, size, self.max_range) <= 0
                or size/self.resolution > 4000):
            raise ValueError('Invalid map dimensions or range (maximum 4000 cells per side)')
        self.side = math.ceil(size/self.resolution)
        self.evidence = np.zeros((self.side, self.side), dtype=np.float32)
        self.observed = np.zeros_like(self.evidence, dtype=bool)
        self.last_stamps = {}
        self.last_clock = None
        self.pending = deque(maxlen=60)
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            OccupancyGrid, '/nav/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions_ = [self.create_subscription(
            LaserScan, topic, lambda scan: self.pending.append(scan), qos_profile_sensor_data)
            for topic in topics]
        self.timer = self.create_timer(1., self.publish_map)
        self.scan_timer = self.create_timer(.05, self.process_pending)
        self.get_logger().info('Collecting /nav/map in world coordinates. No motion commands.')

    def process_pending(self):
        for _ in range(len(self.pending)):
            scan = self.pending.popleft()
            if self.scan(scan) is False:
                self.pending.append(scan)

    def scan(self, scan):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.evidence.fill(0)
            self.observed.fill(False)
            self.last_stamps.clear()
            self.get_logger().warning('Simulation clock reset; map cleared')
        self.last_clock = now
        stamp = Time.from_msg(scan.header.stamp)
        # Limit each laser to 5 Hz of simulation time and reject stale scans.
        age = (now-stamp.nanoseconds)/1e9
        if now == 0 or age < -.05 or age > .5:
            return
        previous = self.last_stamps.get(scan.header.frame_id)
        if previous is not None and stamp.nanoseconds-previous < 200_000_000:
            return
        try:
            tf = self.buffer.lookup_transform(self.frame, scan.header.frame_id, stamp)
        except TransformException as error:
            self.get_logger().warning(f'Scan skipped: {error}', throttle_duration_sec=5.)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = rotation_matrix([q.x, q.y, q.z, q.w])
        translation = np.array([t.x, t.y, t.z])
        start = np.floor((translation[:2]-self.origin)/self.resolution).astype(int)
        if np.any(start < 0) or np.any(start >= self.side):
            self.get_logger().error('Laser outside map bounds; increase size/change origin',
                                    throttle_duration_sec=5.)
            return
        self.last_stamps[scan.header.frame_id] = stamp.nanoseconds
        free_cells, occupied_cells = [], []
        limit = min(self.max_range, scan.range_max)
        for index, distance in enumerate(scan.ranges):
            # NaN and negative infinity carry no usable free-space evidence.
            if math.isnan(distance) or distance < scan.range_min:
                continue
            hit = math.isfinite(distance) and distance < limit
            distance = min(distance, limit)
            angle = scan.angle_min + index*scan.angle_increment
            endpoint = translation + rotation @ np.array([
                distance*math.cos(angle), distance*math.sin(angle), 0.])
            end = np.floor((endpoint[:2]-self.origin)/self.resolution).astype(int)
            cells = ray_cells(start, end)
            free_cells.extend(cells[:-1] if hit else cells)
            if hit:
                occupied_cells.append(end)
        # Apply each cell once per scan; occupied endpoints win within a scan.
        for points, increment in ((free_cells, -.4), (occupied_cells, .85)):
            if not len(points):
                continue
            cells = np.unique(np.asarray(points), axis=0)
            cells = cells[np.all((cells >= 0) & (cells < self.side), axis=1)]
            x, y = cells.T
            self.evidence[y, x] = np.clip(self.evidence[y, x]+increment, -5., 5.)
            self.observed[y, x] = True

    def publish_map(self):
        if not self.observed.any():
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = msg.info.height = self.side
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin.tolist()
        msg.info.origin.orientation.w = 1.
        values = np.full(self.evidence.shape, -1, dtype=np.int8)
        values[self.observed] = np.rint(
            100/(1+np.exp(-self.evidence[self.observed]))).astype(np.int8)
        msg.data = values.ravel().tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
