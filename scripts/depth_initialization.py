"""Create colored world-space Gaussian seeds from aligned RGB and axial depth. Sample

training views only and reject invalid or out-of-range depth.
"""

import numpy as np
from PIL import Image


def initialize_from_depth(views, train_ids, count, near=0.1, far=10.0, seed=42):
    """Sample training views only; keep original depth pixels and RGB aligned."""
    rng = np.random.default_rng(seed)
    budget = max(32, int(np.ceil(4 * count / len(train_ids))))
    points, colors = [], []
    for index in train_ids:
        view = views[index]
        record = view['record']
        if 'depth' not in record or record.get('depth_stamp_ns') != record['stamp_ns']:
            raise ValueError('Depth initialization requires exact RGB/depth timestamps')
        depth = np.load(view['folder'] / record['depth'], allow_pickle=False)
        if depth.dtype != np.float32 or depth.shape != (record['height'], record['width']):
            raise ValueError('Depth must be float32 with the original RGB dimensions')
        valid = np.flatnonzero(np.isfinite(depth) & (depth >= near) & (depth <= far))
        if not len(valid):
            continue
        selected = rng.choice(valid, min(budget, len(valid)), replace=False)
        y, x = np.unravel_index(selected, depth.shape)
        k = np.asarray(record['K']).reshape(3, 3)
        z = depth[y, x]
        optical = np.column_stack(((x - k[0, 2]) * z / k[0, 0],
                                   (y - k[1, 2]) * z / k[1, 1], z))
        pose = view['pose']
        points.append(optical @ pose[:3, :3].T + pose[:3, 3])
        with Image.open(view['folder'] / record['rgb']) as image:
            rgb = np.asarray(image.convert('RGB'))
            if rgb.shape[:2] != depth.shape:
                raise ValueError('RGB/depth dimensions differ')
            colors.append(rgb[y, x].astype(np.float32) / 255)
    if not points:
        raise ValueError('No valid depth samples in training views')
    points, colors = np.concatenate(points), np.concatenate(colors)
    # Avoid repeated surfaces producing zero nearest-neighbour initial scales.
    _, unique = np.unique(np.round(points / 0.005).astype(np.int64), axis=0, return_index=True)
    if len(unique) < count:
        raise ValueError(f'Only {len(unique)} distinct depth seeds; reduce --init-points')
    selected = rng.choice(unique, count, replace=False)
    return points[selected].astype(np.float32), colors[selected]
