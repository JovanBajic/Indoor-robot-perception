"""Verify ray reversal preserves pose metadata and rejects timed scans."""

import unittest
import math

from sensor_msgs.msg import LaserScan

from lidar_outline.navigation_scans import ordered_scan


class NavigationScansTest(unittest.TestCase):
    def test_reverse_pairs_and_preserve_metadata(self):
        scan = LaserScan()
        scan.header.frame_id = 'rear_laser'
        scan.header.stamp.sec = 12
        scan.angle_min, scan.angle_max, scan.angle_increment = -1., 1., 1.
        scan.ranges = [1., 2., 3.]
        scan.intensities = [10., 20., 30.]
        result = ordered_scan(scan, True)
        self.assertEqual(list(result.ranges), [3., 2., 1.])
        self.assertEqual(list(result.intensities), [30., 20., 10.])
        self.assertEqual(result.header, scan.header)
        self.assertEqual(result.angle_min, -1.)
        self.assertEqual(list(scan.ranges), [1., 2., 3.])

    def test_reject_timed_reversal(self):
        scan = LaserScan()
        scan.time_increment = .001
        with self.assertRaises(ValueError):
            ordered_scan(scan, True)

    def test_inactive_rays_do_not_clear_obstacles(self):
        scan = LaserScan()
        scan.range_max = 30.
        scan.ranges = [30., 2., float('inf')]
        result = ordered_scan(scan, True, True)
        self.assertTrue(math.isnan(result.ranges[0]))
        self.assertEqual(result.ranges[1], 2.)
        self.assertTrue(math.isnan(result.ranges[2]))
        self.assertEqual(scan.ranges[0], 30.)


if __name__ == '__main__':
    unittest.main()
