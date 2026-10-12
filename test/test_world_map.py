"""Check inverted scan geometry and occupancy evidence with a known wall."""

import math
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from lidar_outline.world_map import WorldMap, ray_cells, rotation_matrix


class WorldMapTest(unittest.TestCase):
    def test_inverted_laser_and_negative_cells(self):
        rotation = rotation_matrix([1., 0., 0., 0.])
        np.testing.assert_allclose(rotation @ [0., 1., 0.], [0., -1., 0.])
        cells = ray_cells((-3, -2), (0, 0))
        np.testing.assert_array_equal(cells[0], [-3, -2])
        np.testing.assert_array_equal(cells[-1], [0, 0])

    def test_wall_hit_unknown_and_clock_reset(self):
        rclpy.init()
        node = WorldMap()
        try:
            node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=2.))
            transform = TransformStamped()
            transform.transform.rotation.x = 1.
            transform.transform.rotation.w = 0.
            node.buffer.lookup_transform = lambda *args: transform
            scan = LaserScan()
            scan.header.frame_id = 'laser'
            scan.header.stamp = Time(seconds=2.).to_msg()
            scan.range_min, scan.range_max = .1, 30.
            scan.angle_min = math.pi/2
            scan.ranges = [2.]
            node.scan(scan)
            # Inverted laser's +Y ray hits world Y=-2, leaving Y=+2 unknown.
            # LaserScan stores float32 angles; pi/2 can fall either side of X=0.
            self.assertGreater(node.evidence[359:362, 399:401].max(), 0.)
            self.assertLess(node.evidence[380, 399:401].min(), 0.)
            self.assertFalse(node.observed[440, 400])
            node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=1.))
            scan.header.stamp = Time(seconds=1.).to_msg()
            scan.ranges = [float('nan')]
            node.scan(scan)
            self.assertFalse(node.observed.any())
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
