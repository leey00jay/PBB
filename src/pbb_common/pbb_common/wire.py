"""Convert typed ROS messages to the existing algorithm data classes."""
import numpy as np
from numpy.polynomial import Polynomial
from geometry_msgs.msg import Point
from pbb_interfaces.msg import DrivePlan, LaneEstimate, VehicleControl


def encode_lanes(observation, state):
    msg = LaneEstimate(state=state)
    for side in ('left', 'right'):
        poly = getattr(observation, side)
        valid = poly is not None and np.isfinite(poly.coef).all()
        setattr(msg, side + '_valid', bool(valid))
        setattr(msg, side + '_coefficients', poly.convert().coef.tolist() if valid else [])
        extent = getattr(observation, side + '_extent_m') if valid else 0.0
        if not np.isfinite(extent) or not 0 <= extent <= 60:
            raise ValueError('Invalid observation extent')
        setattr(msg, side + '_extent_m', float(extent))
    if observation.centerline is not None:
        coefficients = observation.centerline.convert().coef
        if not 1 <= len(coefficients) <= 4 or not np.isfinite(coefficients).all():
            raise ValueError('Invalid centerline polynomial')
        msg.centerline_coefficients = coefficients.tolist()
    if not np.isfinite(observation.centerline_extent_m) or not 0 <= observation.centerline_extent_m <= 60:
        raise ValueError('Invalid centerline extent')
    msg.centerline_extent_m = float(observation.centerline_extent_m)
    msg.right_double_yellow = bool(observation.right_double_yellow)
    return msg


def decode_lanes(msg):
    from pbb_common.models import LaneObservation
    curves = []
    for side in ('left', 'right'):
        coefficients = getattr(msg, side + '_coefficients')
        valid = getattr(msg, side + '_valid')
        if valid and (not 1 <= len(coefficients) <= 4 or not np.isfinite(coefficients).all()):
            raise ValueError('Invalid polynomial coefficients')
        curves.append(Polynomial(coefficients) if valid else None)
    extents = [getattr(msg, side + '_extent_m') for side in ('left', 'right')]
    if not np.isfinite(extents).all() or not all(0 <= x <= 60 for x in extents):
        raise ValueError('Invalid observation extent')
    median = msg.centerline_coefficients
    if (len(median) > 4 or not np.isfinite(median).all()
            or not np.isfinite(msg.centerline_extent_m) or not 0 <= msg.centerline_extent_m <= 60):
        raise ValueError('Invalid centerline observation')
    return LaneObservation(*curves, overlay=np.empty((0, 0, 3), dtype=np.uint8),
                           left_extent_m=extents[0], right_extent_m=extents[1],
                           centerline=Polynomial(median) if median else None,
                           centerline_extent_m=msg.centerline_extent_m,
                           right_double_yellow=msg.right_double_yellow)


def encode_plan(plan, state):
    points = [] if plan.trajectory is None else [
        Point(x=float(x), y=float(y), z=0.0) for x, y in plan.trajectory]
    return DrivePlan(state=state, trajectory=points, has_trajectory=plan.trajectory is not None,
                     target_kmh=plan.target_kmh, status=plan.status, drive_allowed=plan.drive_allowed,
                     stop_brake=plan.stop_brake, lane_width_m=plan.lane_width_m,
                     left_valid=plan.left_valid, right_valid=plan.right_valid)


def decode_plan(msg):
    from pbb_common.models import DrivePlan as AlgorithmPlan
    trajectory = np.array([(p.x, p.y) for p in msg.trajectory], dtype=float) if msg.has_trajectory else None
    if (not np.isfinite([msg.target_kmh, msg.stop_brake, msg.lane_width_m]).all()
            or msg.target_kmh < 0 or not 0 <= msg.stop_brake <= 1
            or (trajectory is not None and (len(trajectory) < 2 or not np.isfinite(trajectory).all()))
            or (msg.drive_allowed and trajectory is None)):
        raise ValueError('Invalid drive plan')
    return AlgorithmPlan(trajectory, msg.target_kmh, msg.status, msg.drive_allowed,
                         msg.stop_brake, msg.lane_width_m, msg.left_valid, msg.right_valid)


def encode_control(command, state):
    return VehicleControl(state=state, throttle=command.throttle, brake=command.brake,
                          steer=command.steer, target_kmh=command.target_kmh, status=command.status)
