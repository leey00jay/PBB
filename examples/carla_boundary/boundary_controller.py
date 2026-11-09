"""Simulation-only control helpers for the uploaded road boundary detector."""
import math


def clip(value, low, high):
    return max(low, min(high, value))


def vision_target(result, width, height, camera_height=1.82, camera_x=1.5):
    """Project a detected corridor midpoint onto an assumed flat road.

    This calibration is for the configured CARLA camera, not a general conversion
    of the uploaded center_error_px into metres. No map data is used here.
    """
    if not result['path_valid'] or result.get('estimate_age_s', 1) > .2:
        return None
    y = .72 * height
    xs = []
    for key in ('yellow_line', 'curb_candidate'):
        xb, yb, xt, yt = result[key]
        xs.append(xt + (xb - xt) * (y - yt) / (yb - yt))
    focal = width / 2  # 90-degree horizontal FOV.
    scale = camera_height / (y - height / 2)
    corridor_width = (xs[1] - xs[0]) * scale
    if not 2.3 <= corridor_width <= 5.5:
        return None
    lateral = (sum(xs) / 2 - width / 2) * scale
    forward = focal * scale + camera_x
    if abs(lateral) > 2.5:
        return None
    return forward, lateral


def pursuit_steer(target, wheelbase, max_steer_deg):
    x, y = target
    if x <= .5:
        return 0.0
    angle = math.atan2(2 * wheelbase * y, x*x + y*y)
    return clip(angle / math.radians(max_steer_deg), -.4, .4)


def target_speed(cruise_mps, signal, front_gap_m):
    if signal in ('Red', 'Yellow') and front_gap_m > -3:
        return min(cruise_mps, .65 * max(front_gap_m - 1.0, 0))
    return cruise_mps


def longitudinal(speed_mps, desired_mps):
    if desired_mps < .15:
        return 0.0, 1.0
    # One continuous effort signal avoids switching directly from positive
    # throttle to strong braking at zero speed error. Brake response in this
    # CARLA vehicle is much stronger than throttle response at low speeds.
    effort = .45 * (desired_mps - speed_mps) + .10 * min(desired_mps, 1.0)
    if effort >= 0:
        return clip(effort, 0, .55), 0.0
    return 0.0, clip(-.18 * effort, 0, .8)
