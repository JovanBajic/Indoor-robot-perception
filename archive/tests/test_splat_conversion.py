"""Test that the archived dataset converter preserves camera viewing rays and world

positions.
"""

import importlib.util
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location(
    'splat_conversion', Path(__file__).parents[1] / 'splatfacto' /
    'convert_capture_splatfacto.py')
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


def test_optical_ray_conversion():
    """OpenGL negative Z must match optical positive Z in world space."""
    position = [3., 4., 5.]
    # Optical camera rotated 90 degrees about world Y.
    q = [0., np.sqrt(0.5), 0., np.sqrt(0.5)]
    pose = np.array(converter.optical_to_opengl(position, q))
    np.testing.assert_allclose(pose[:3, 3], position)
    np.testing.assert_allclose(pose[:3, :3] @ [0., 0., -1.],
                               [1., 0., 0.], atol=1e-12)
    np.testing.assert_allclose(pose[:3, :3].T @ pose[:3, :3],
                               np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(pose[:3, :3]), 1.)
