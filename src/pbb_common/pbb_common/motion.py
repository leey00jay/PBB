"""Move camera-ground polynomials between measured CARLA vehicle poses."""
import numpy as np
from numpy.polynomial import Polynomial


def move_curve(curve, previous_pose, current_pose):
    """Pose is world (x, y, yaw); curve uses camera x forward, y LEFT.

    The center camera sits 0.5 m ahead of the vehicle origin. Transform sampled
    ground points, then refit in the current camera frame; never freeze an old
    vehicle-relative curve while the vehicle moves through a bend.
    """
    old_x, old_y, old_yaw = previous_pose
    new_x, new_y, new_yaw = current_pose
    delta = old_yaw - new_yaw
    c, s = np.cos(delta), np.sin(delta)
    dx, dy = old_x - new_x, old_y - new_y
    tx = np.cos(new_yaw) * dx + np.sin(new_yaw) * dy
    ty = -np.sin(new_yaw) * dx + np.cos(new_yaw) * dy
    x = np.linspace(-2.0, 30.0, 65)
    y_right = -curve(x)
    current_x = c * (x + 0.5) - s * y_right + tx - 0.5
    current_y = -(s * (x + 0.5) + c * y_right + ty)
    if (not np.isfinite(current_y).all() or not np.isfinite(current_x).all()
            or np.any(np.diff(current_x) <= 0) or current_x[-1] < 12.0):
        return None
    degree = min(3, curve.degree())
    # Rotation of even a constant curve can produce a nonzero slope.
    return Polynomial.fit(current_x, current_y, max(1, degree)).convert()
