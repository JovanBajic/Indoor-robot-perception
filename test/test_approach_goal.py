"""Check approach clearance and conservative collision handling on synthetic room grids."""

import math
import unittest

import numpy as np

from lidar_outline.approach_goal import (
    approach_poses, footprint_is_free, footprint_overlaps_box)


class ApproachTest(unittest.TestCase):
    def setUp(self):
        self.footprint = [(-.42, -.3), (.53, -.3), (.53, .3), (-.42, .3)]
        self.grid = np.zeros((100, 100), dtype=np.int8)

    def test_front_clearance_and_heading(self):
        poses = approach_poses([1, 1, 0], [2, 2, 1], self.footprint, .7, 8)
        x, y, yaw = poses[0]
        self.assertGreaterEqual(x - .53 - 2, .7)
        self.assertAlmostEqual(y, 1.5)
        self.assertAlmostEqual(abs(yaw), math.pi)
        for x, y, yaw in poses:
            delta = math.atan2(1.5-y, 1.5-x) - yaw
            self.assertAlmostEqual(math.atan2(math.sin(delta), math.cos(delta)), 0.)

    def test_rotated_footprint_unknown_obstacle_and_boundary(self):
        self.assertTrue(footprint_is_free(self.grid, .1, (0, 0, 0),
                                          (5, 5, math.pi/2), self.footprint))
        # This obstacle touches the front of the rotated footprint.
        self.grid[55, 50] = 100
        self.assertFalse(footprint_is_free(self.grid, .1, (0, 0, 0),
                                           (5, 5, math.pi/2), self.footprint))
        self.grid[55, 50] = -1
        self.assertFalse(footprint_is_free(self.grid, .1, (0, 0, 0),
                                           (5, 5, math.pi/2), self.footprint))
        self.assertFalse(footprint_is_free(self.grid, .1, (0, 0, 0),
                                           (.1, .1, 0), self.footprint))

    def test_rotated_map_origin(self):
        self.assertTrue(footprint_is_free(self.grid, .1, (10, 0, math.pi/2),
                                          (5, 5, 0), self.footprint))

    def test_obstacle_inside_polygon_not_just_vertices(self):
        self.grid[50, 50] = 100
        self.assertFalse(footprint_is_free(self.grid, .1, (0, 0, 0),
                                           (5, 5, 0), self.footprint))

    def test_semantic_box_blocks_a_route_even_when_lidar_misses_it(self):
        self.assertTrue(footprint_overlaps_box((1.5, 1.5, .4), self.footprint,
                                               [1, 1, 0], [2, 2, 1]))
        self.assertFalse(footprint_overlaps_box((4, 4, .4), self.footprint,
                                                [1, 1, 0], [2, 2, 1]))

    def test_invalid_bounds(self):
        with self.assertRaises(ValueError):
            approach_poses([2, 0, 0], [1, 1, 1], self.footprint)


if __name__ == '__main__':
    unittest.main()
