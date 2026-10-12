"""Test depth-to-world seed geometry, matching RGB colors and rejection of mismatched

timestamps.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from depth_initialization import initialize_from_depth


def test_rotated_depth_plane_and_matching_colors(tmp_path):
    depth = np.full((20, 20), 2, dtype=np.float32)
    depth[0, 0] = np.inf
    np.save(tmp_path / 'depth.npy', depth)
    Image.new('RGB', (20, 20), (255, 0, 0)).save(tmp_path / 'rgb.png')
    pose = np.array([[0, 0, 1, 3], [0, 1, 0, 4], [-1, 0, 0, 5], [0, 0, 0, 1.]])
    record = {'depth': 'depth.npy', 'rgb': 'rgb.png', 'stamp_ns': 1,
              'depth_stamp_ns': 1, 'height': 20, 'width': 20,
              'K': [10, 0, 10, 0, 10, 10, 0, 0, 1]}
    view = {'record': record, 'folder': tmp_path, 'pose': pose}
    # A held-out view deliberately lacks depth: it must never be loaded.
    points, colors = initialize_from_depth([{}, view], [1], 100)
    assert np.allclose(points[:, 0], 5)
    assert np.isfinite(points).all()
    assert np.allclose(colors, [1, 0, 0])
    record['depth_stamp_ns'] = 2
    with pytest.raises(ValueError, match='timestamps'):
        initialize_from_depth([view], [0], 100)
