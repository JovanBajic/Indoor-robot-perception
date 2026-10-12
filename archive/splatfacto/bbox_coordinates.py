"""Convert bounding boxes from normalized Splatfacto coordinates to world metres. This

helper is retained only for archived standard reconstructions.
"""

from itertools import product

import numpy as np


def world_bbox(box, normalization):
    """Invert p_splat = scale * (R * p_world + t) on all eight corners."""
    transform = np.asarray(normalization['transform'], dtype=float)
    scale = float(normalization['scale'])
    lo = np.asarray(box['min'], dtype=float)
    hi = np.asarray(box['max'], dtype=float)
    if (transform.shape != (3, 4) or lo.shape != (3,) or hi.shape != (3,) or
            not np.isfinite(transform).all() or not np.isfinite(scale) or
            scale <= 0 or not np.isfinite(lo).all() or
            not np.isfinite(hi).all() or np.any(hi < lo)):
        raise ValueError('Invalid normalization or bounding box')
    matrix = np.eye(4)
    matrix[:3] = transform
    inverse = np.linalg.inv(matrix)
    corners = np.array(list(product(*zip(lo, hi))))
    homogeneous = np.column_stack((corners / scale, np.ones(8)))
    world_corners = (homogeneous @ inverse.T)[:, :3]
    minimum = world_corners.min(axis=0)
    maximum = world_corners.max(axis=0)
    return {
        'frame_id': 'world', 'units': 'metres',
        'min': minimum.tolist(), 'max': maximum.tolist(),
        'center': ((minimum + maximum) / 2).tolist(),
        'size': (maximum - minimum).tolist(),
        'transformed_corners': world_corners.tolist(),
        'definition': 'World-axis-aligned enclosure of the selected Gaussian-center box',
    }
