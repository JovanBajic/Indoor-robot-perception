"""Build optical and OpenGL camera-to-world poses from recorded positions and

quaternions. Preserve world coordinates independently of the reconstruction trainer.
"""

import numpy as np


def optical_to_world(position, quaternion):
    """Return ROS optical camera-to-world in unchanged world metres (xyzw)."""
    position = np.asarray(position, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('Invalid position')
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Invalid quaternion')
    norm = np.linalg.norm(q)
    if norm < 1e-9 or not np.isfinite(norm):
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    pose = np.eye(4)
    pose[:3, :3] = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    pose[:3, 3] = position
    return pose


def optical_to_opengl(position, quaternion):
    """Return OpenGL camera-to-world, keeping the same world coordinate frame."""
    pose = optical_to_world(position, quaternion)
    pose[:3, :3] = pose[:3, :3] @ np.diag([1., -1., -1.])
    return pose.tolist()
