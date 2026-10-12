"""Generate object approach poses and conservatively check padded footprints on grids."""

import math

import numpy as np


def approach_poses(bbox_min, bbox_max, footprint, clearance=.7, samples=16):
    """Place inward-facing robot envelopes at least clearance from an XY box boundary."""
    lo, hi = np.asarray(bbox_min, float), np.asarray(bbox_max, float)
    poly = np.asarray(footprint, float)
    if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all()
            or np.any(hi < lo) or poly.ndim != 2 or poly.shape[1] != 2
            or len(poly) < 3 or not np.isfinite(poly).all()
            or not math.isfinite(clearance) or clearance <= 0 or samples < 4):
        raise ValueError('Invalid bounds, footprint or approach settings')
    center = (lo[:2] + hi[:2]) / 2
    half = np.maximum((hi[:2] - lo[:2]) / 2, .005)
    radius = float(np.linalg.norm(poly, axis=1).max())
    result = []
    for angle in np.linspace(0, 2 * math.pi, samples, endpoint=False):
        direction = np.array([math.cos(angle), math.sin(angle)])
        distance = min(half[i] / abs(direction[i]) for i in range(2)
                       if abs(direction[i]) > 1e-9)
        # A circumscribed footprint disk ensures clearance even near box corners.
        low, high = distance, float(np.linalg.norm(half)) + clearance + radius
        for _ in range(40):
            mid = (low+high)/2
            gap = np.maximum(np.abs(direction*mid)-half, 0.)
            if np.linalg.norm(gap) < clearance+radius:
                low = mid
            else:
                high = mid
        xy = center + direction * high
        result.append((float(xy[0]), float(xy[1]),
                       math.atan2(-direction[1], -direction[0])))
    return result


def footprint_is_free(grid, resolution, origin, pose, footprint, max_cost=0):
    """Reject unknown/occupied cells touching the polygon, including cell edge overlap.

    origin and pose are (x,y,yaw). max_cost uses OccupancyGrid's 0..100 scale.
    Cell-centre distances expand polygon edges by a cell's half diagonal,
    conservatively covering cells intersected by the polygon.
    """
    grid = np.asarray(grid)
    poly = np.asarray(footprint, float)
    if (grid.ndim != 2 or resolution <= 0 or not math.isfinite(resolution)
            or poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3
            or not np.isfinite(poly).all()
            or not np.isfinite([*origin, *pose]).all()):
        raise ValueError('Invalid grid geometry')
    def rotation(yaw):
        return np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    world = poly @ rotation(pose[2]).T + pose[:2]
    local = (world - origin[:2]) @ rotation(origin[2]) / resolution
    height, width = grid.shape
    if (local[:, 0].min() < 0 or local[:, 1].min() < 0
            or local[:, 0].max() >= width or local[:, 1].max() >= height):
        return False
    lower = np.maximum(np.floor(local.min(0) - 1).astype(int), 0)
    upper = np.minimum(np.ceil(local.max(0) + 1).astype(int), [width, height])
    yy, xx = np.mgrid[lower[1]:upper[1], lower[0]:upper[0]]
    cells = np.column_stack([xx.ravel() + .5, yy.ravel() + .5])
    inside = np.zeros(len(cells), dtype=bool)
    touching = np.zeros(len(cells), dtype=bool)
    for a, b in zip(local, np.roll(local, -1, axis=0)):
        edge = b - a
        length = float(edge @ edge)
        if length < 1e-12:
            continue
        t = np.clip(((cells - a) @ edge) / length, 0, 1)
        touching |= np.sum((cells - (a + t[:, None] * edge)) ** 2, axis=1) <= .5 + 1e-9
        if abs(edge[1]) > 1e-12:
            crossing = ((a[1] > cells[:, 1]) != (b[1] > cells[:, 1]))
            intersection = a[0] + (cells[:, 1] - a[1]) * edge[0] / edge[1]
            inside ^= crossing & (cells[:, 0] < intersection)
    values = grid[yy.ravel(), xx.ravel()][inside | touching]
    return bool(len(values) and np.all((values >= 0) & (values <= max_cost)))


def path_length(points):
    """Measure XY route length without treating sample count as distance."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def footprint_overlaps_box(pose, footprint, bbox_min, bbox_max):
    """Use separating axes to reject a convex robot envelope touching the XY box."""
    poly = np.asarray(footprint, float)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    poly = poly @ np.array([[c, s], [-s, c]]) + pose[:2]
    lo, hi = np.asarray(bbox_min)[:2], np.asarray(bbox_max)[:2]
    box = np.array([[lo[0], lo[1]], [hi[0], lo[1]],
                    [hi[0], hi[1]], [lo[0], hi[1]]])
    axes = [np.array([1., 0.]), np.array([0., 1.])]
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b-a
        if np.linalg.norm(edge) > 1e-9:
            axes.append(np.array([-edge[1], edge[0]]))
    for axis in axes:
        a, b = poly @ axis, box @ axis
        if a.max() < b.min() or b.max() < a.min():
            return False
    return True
