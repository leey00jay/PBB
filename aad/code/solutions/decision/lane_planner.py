"""판단: choose a lane-centre path, speed target, and drive/stop state.

A single visible boundary assumes the last known lane width (initially 3.5 m).
The current settings deliberately match the pre-split Town10 demo.
"""
import numpy as np
from ..interfaces import DrivePlan


class LanePlanner:
    def __init__(self):
        self.lane_width = 3.5
        self.previous_center = None
        self.valid_frames = 0
        self.frames = 0

    def plan(self, observation, desired_speed_kmh):
        self.frames += 1
        left, right = observation.left, observation.right
        center = None
        status = 'LANE LOST - STOP'
        target_kmh = 0.0
        if left is not None and right is not None:
            widths = left(np.array([5., 10., 15.])) - right(np.array([5., 10., 15.]))
            if np.all((widths >= 2.5) & (widths <= 4.5)) and np.ptp(widths) < 0.8:
                self.lane_width = 0.95 * self.lane_width + 0.05 * float(np.median(widths))
                center = (left + right) * 0.5
                status = 'BOTH LANES'
                target_kmh = desired_speed_kmh
            else:
                status = 'LANE WIDTH INVALID - STOP'
        elif right is not None:
            center = right + self.lane_width / 2.0
            status = 'RIGHT ONLY / WIDTH ASSUMED'
            target_kmh = desired_speed_kmh
        elif left is not None:
            center = left - self.lane_width / 2.0
            status = 'LEFT ONLY / WIDTH ASSUMED'
            target_kmh = desired_speed_kmh

        x = np.arange(-2.0, 31.0)
        traj = None
        if center is not None:
            if abs(float(center(0.0))) > 2.0:
                status, center = 'PATH TOO FAR - STOP', None
            elif self.previous_center is not None and abs(float(center(6.) - self.previous_center(6.))) > 0.8:
                status, center = 'PATH JUMP - STOP', None
            if center is not None:
                if self.previous_center is not None:
                    center = 0.35 * center + 0.65 * self.previous_center
                self.previous_center = center
                traj = np.column_stack((x + 0.5, -center(x)))

        if traj is None:
            self.valid_frames = 0
            self.previous_center = None
        else:
            self.valid_frames += 1

        drive_allowed = False
        stop_brake = 0.7
        if self.frames <= 30:
            status = 'WARMUP - STOP'
            stop_brake = 1.0
        elif traj is not None and self.valid_frames >= 5:
            drive_allowed = True
        elif traj is not None:
            status = 'CHECKING LANE - STOP'
        if 'STOP' in status:
            target_kmh = 0.0
        return DrivePlan(trajectory=traj, target_kmh=float(target_kmh),
                         status=status, drive_allowed=drive_allowed,
                         stop_brake=stop_brake, lane_width_m=float(self.lane_width),
                         left_valid=left is not None, right_valid=right is not None)
