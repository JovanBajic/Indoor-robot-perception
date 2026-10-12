"""Check measured planar odometry axes, angle wrapping and invalid intervals."""

import math
import unittest

from lidar_outline.planar_pose import measured_velocity, planar_pose


class PlanarPoseTest(unittest.TestCase):
    def test_body_velocity_after_rotation(self):
        yaw = math.pi / 2
        self.assertAlmostEqual(planar_pose([3, 4, .1], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])[2], yaw)
        vx, vy, wz = measured_velocity((0, 0, yaw), (0, 1, yaw), .5)
        self.assertAlmostEqual(vx, 2)
        self.assertAlmostEqual(vy, 0)
        self.assertAlmostEqual(wz, 0)

    def test_yaw_wrap(self):
        velocity = measured_velocity((0, 0, math.pi-.01), (0, 0, -math.pi+.01), .1)
        self.assertAlmostEqual(velocity[2], .2)

    def test_invalid_pose_and_time(self):
        with self.assertRaises(ValueError):
            planar_pose([0, 0, 0], [0, 0, 0, 0])
        with self.assertRaises(ValueError):
            planar_pose([float('nan'), 0, 0], [0, 0, 0, 1])
        for dt in (0, -1, float('inf')):
            with self.assertRaises(ValueError):
                measured_velocity((0, 0, 0), (1, 0, 0), dt)


if __name__ == '__main__':
    unittest.main()
