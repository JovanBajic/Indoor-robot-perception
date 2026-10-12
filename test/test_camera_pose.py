"""Test camera-pose axes, quaternion normalization and invalid-input handling in the

neutral pose helper.
"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from camera_pose import optical_to_opengl, optical_to_world


class CameraPoseTest(unittest.TestCase):
    def test_optical_and_opengl_rays_agree(self):
        position = [3., 4., 5.]
        quaternion = [0., np.sqrt(.5), 0., np.sqrt(.5)]
        optical = optical_to_world(position, quaternion)
        opengl = np.array(optical_to_opengl(position, quaternion))
        np.testing.assert_allclose(optical[:3, 3], position)
        np.testing.assert_allclose(optical[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(opengl[:3, :3] @ [0, 0, -1], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(optical, opengl @ np.diag([1, -1, -1, 1]))

    def test_quaternion_normalization_preserves_rotation(self):
        from scipy.spatial.transform import Rotation
        rng = np.random.default_rng(42)
        for _ in range(20):
            q = rng.normal(size=4)
            pose = optical_to_world([1, 2, 3], q * 5)
            np.testing.assert_allclose(pose[:3, :3], Rotation.from_quat(q).as_matrix(), atol=1e-12)
            np.testing.assert_allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-12)

    def test_invalid_poses_are_rejected(self):
        for p, q in [([0, 0], [0, 0, 0, 1]), ([np.nan, 0, 0], [0, 0, 0, 1]),
                     ([0, 0, 0], [0, 0, 0, 0]), ([0, 0, 0], [0, 0, np.inf, 1]),
                     ([0, 0, 0], [0, 0, 1])]:
            with self.assertRaises(ValueError):
                optical_to_world(p, q)


if __name__ == '__main__':
    unittest.main()
