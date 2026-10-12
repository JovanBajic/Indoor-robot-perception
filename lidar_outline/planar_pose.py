"""Convert measured simulator poses into planar odometry and body velocities."""

import math


def planar_pose(position, quaternion):
    """Return world x/y/yaw from a measured xyzw quaternion, discarding roll/pitch."""
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError('Expected a 3D position and xyzw quaternion')
    values = list(position) + list(quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Measured pose must be finite')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9:
        raise ValueError('Measured quaternion must be nonzero')
    x, y, z, w = (value / norm for value in quaternion)
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return float(position[0]), float(position[1]), yaw


def measured_velocity(previous, current, dt):
    """Finite-difference measured world poses and express velocity in body axes."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Measured pose interval must be positive')
    vx = (current[0] - previous[0]) / dt
    vy = (current[1] - previous[1]) / dt
    yaw = current[2]
    delta = math.atan2(math.sin(yaw - previous[2]), math.cos(yaw - previous[2]))
    return (math.cos(yaw)*vx + math.sin(yaw)*vy,
            -math.sin(yaw)*vx + math.cos(yaw)*vy, delta / dt)
