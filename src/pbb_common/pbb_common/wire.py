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
    return LaneObservation(*curves, overlay=np.empty((0, 0, 3), dtype=np.uint8))


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
