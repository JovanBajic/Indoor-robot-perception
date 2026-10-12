"""Test conversion of normalized Splatfacto bounding boxes back into world coordinates.

These checks belong to the archived standard workflow.
"""

from itertools import product

from archive.splatfacto.bbox_coordinates import world_bbox

import numpy as np


def test_translated_scaled_box():
    """Recover known world corners after centering and scaling."""
    result = world_bbox({'min': [0., 0., 0.], 'max': [1., 2., 3.]}, {
        'scale': 0.5,
        'transform': [[1., 0., 0., -4.], [0., 1., 0., -5.], [0., 0., 1., -6.]],
    })
    np.testing.assert_allclose(result['min'], [4., 5., 6.])
    np.testing.assert_allclose(result['max'], [6., 9., 12.])
    np.testing.assert_allclose(result['center'], [5., 7., 9.])


def test_rotated_box_contains_all_corners():
    """A rotated box needs all eight corners, not just opposite min/max."""
    a = np.pi / 4
    rotation = np.array([[np.cos(a), -np.sin(a), 0.],
                         [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
    transform = np.column_stack((rotation, [1., 2., 3.]))
    box = {'min': [-1., -2., -3.], 'max': [2., 3., 4.]}
    result = world_bbox(box, {'scale': 0.25, 'transform': transform.tolist()})
    points = np.array(list(product(*zip(box['min'], box['max']))))
    expected = (points / 0.25 - transform[:, 3]) @ rotation
    np.testing.assert_allclose(result['min'], expected.min(axis=0))
    np.testing.assert_allclose(result['max'], expected.max(axis=0))
