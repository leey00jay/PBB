"""판단: 차선 중심 경로, 목표 속도, 주행 상태 결정."""

import numpy as np
from pbb_common.models import DrivePlan
from numpy.polynomial import Polynomial
from pbb_common.motion import move_curve


class LanePlanner:
    def __init__(self, daytime=True):
        self.daytime = daytime
        self.lane_width = 3.5
        self.previous_center = None
        self.previous_speed = 0.0
        self.valid_frames = 0
        self.frames = 0

        # 주행 속도 설정
        self.cruise_speed = 80.0
        self.single_lane_speed = 40.0
        self.max_gap_seconds = 2.0
        self.max_gap_distance = 12.0
        self.previous_pose = None
        self.gap_seconds = 0.0
        self.gap_distance = 0.0
        self.width_frames = 0
        self.was_driving = False
        self.centerline_stop = None
        self.reacquire_center = None
        self.reacquire_frames = 0
        self.reacquire_age = 0.0
        self.searching = False
        self.search_center = None
        self.search_speed = 20.0
        self.straight_seconds = 0.0
        self.curve_ceiling = self.cruise_speed
        self.fast_single_speed = 60.0 if daytime else 55.0
        self.straight_speed = 60.0 if daytime else self.cruise_speed

        # 주행 안정성 설정
        self.max_lateral_accel = 2.0
        self.max_accel = 3.0 if daytime else 1.5
        self.max_decel = 3.0
        self.max_path_jump = 0.8
        
        # 경로 안정화 비율
        self.smoothing = 0.5

    def calculate_curve_speed(self, center):
        """전방 곡률과 감속 거리를 고려한 속도 계산."""

        x = np.linspace(3.0, 25.0, 45)

        dy = center.deriv(1)(x)
        ddy = center.deriv(2)(x)

        curvature = np.abs(ddy) / (1.0 + dy ** 2) ** 1.5

        if not np.all(np.isfinite(curvature)):
            return 0.0

        curvature = np.maximum(curvature, 1e-6)

        # 각 지점에서 허용되는 속도
        curve_speed = np.sqrt(
            self.max_lateral_accel / curvature
        )

        # 커브에 도달하기 전 감속 가능한 거리 반영
        distance = np.maximum(x - 2.0, 0.0)

        allowed_speed = np.sqrt(
            curve_speed ** 2
            + 2.0 * self.max_decel * distance
        )

        return float(
            np.clip(
                np.min(allowed_speed) * 3.6,
                0.0,
                self.cruise_speed
            )
        )

    def limit_speed_change(self, target_kmh, dt):
        """목표 속도의 급격한 변화 제한."""

        max_increase = self.max_accel * dt * 3.6
        max_decrease = self.max_decel * dt * 3.6

        target_kmh = np.clip(
            target_kmh,
            self.previous_speed - max_decrease,
            self.previous_speed + max_increase
        )

        self.previous_speed = float(target_kmh)

        return float(target_kmh)

    def observed_speed(self, center, extent, both, dt):
        """Fast only with long, centered, stable CURRENT image evidence."""
        x = np.linspace(3., max(10., min(25., extent)), 30)
        dy = center.deriv()(x)
        curvature = float(np.max(np.abs(center.deriv(2)(x)) / (1 + dy**2)**1.5))
        straight = (extent >= 18.0 and curvature < .0025
                    and np.max(np.abs(dy)) < .10 and abs(float(center(0))) < .45)
        # Tolerate a few noisy dashed fragments without repeatedly dropping to
        # 40. Actual curvature and shorter visible distance still cap speed now.
        self.straight_seconds = (min(1., self.straight_seconds+dt) if straight
                                 else max(0., self.straight_seconds-2*dt) if self.daytime else 0.)
        curve_cap = self.calculate_curve_speed(center)
        if curvature > .025:
            curve_cap = min(curve_cap, 28.0)
        elif curvature > .008:
            curve_cap = min(curve_cap, 35.0)
        # React to a newly visible bend now; release its cap gradually so one
        # straight-looking dashed fragment cannot accelerate us into a turn.
        self.curve_ceiling = min(curve_cap, self.curve_ceiling + 5.4 * dt)
        cap = self.single_lane_speed if both else 35.0
        if self.straight_seconds >= .7:
            cap = self.straight_speed if both else self.fast_single_speed
            # Leave room to slow to 25 km/h at the end of actual visible paint.
            visible_cap = np.sqrt((25 / 3.6)**2 + 2*self.max_decel*max(0., extent-4.)) * 3.6
            cap = min(cap, visible_cap)
        return min(cap, self.curve_ceiling)

    def _consistent_candidate(self, center, motion_known, hits=3, extent=15.):
        x = np.array([0., 5., max(6., min(15., extent))])
        consistent = (self.reacquire_center is not None
                      and np.max(np.abs(center(x) - self.reacquire_center(x))) < .35)
        self.reacquire_frames = self.reacquire_frames + 1 if consistent else 1
        self.reacquire_center = center
        self.reacquire_age = 0.0
        return self.reacquire_frames >= hits

    def plan(self, observation, desired_speed_kmh, dt=0.05, *, vehicle_pose=None):
        """Observed center -> cautious straight search -> confirmed recentering.

        Both missing means hold the heading at loss, not the old curve. Search
        stays at <=20 km/h until paint returns. Invalid state is stopped by ROS.
        """
        if not np.isfinite([desired_speed_kmh, dt]).all() or not 0 < dt <= 1:
            raise ValueError("Invalid requested speed or simulation interval")
        if vehicle_pose is not None:
            vehicle_pose = np.asarray(vehicle_pose, dtype=float)
            if vehicle_pose.shape != (3,) or not np.isfinite(vehicle_pose).all():
                raise ValueError("Invalid vehicle pose")
        self.frames += 1
        self.reacquire_age += dt
        if observation.right_double_yellow:
            self.centerline_stop = "RIGHT DOUBLE YELLOW - STOP"
        if self.centerline_stop:
            self.previous_speed = 0.
            return DrivePlan(None, 0., self.centerline_stop, False, 1.,
                             float(self.lane_width), False, False)
        left, right = observation.left, observation.right
        left_valid, right_valid = left is not None, right is not None
        missing = left is None and right is None
        desired = float(np.clip(desired_speed_kmh, 0., self.cruise_speed))
        prior = self.previous_center
        motion_known = vehicle_pose is not None and self.previous_pose is not None
        distance = 0.0
        if motion_known:
            distance = float(np.linalg.norm(vehicle_pose[:2] - self.previous_pose[:2]))
            if prior is not None:
                prior = move_curve(prior, self.previous_pose, vehicle_pose)
            for attr in ('reacquire_center', 'search_center'):
                curve = getattr(self, attr)
                if curve is not None:
                    setattr(self, attr, move_curve(curve, self.previous_pose, vehicle_pose))
        self.previous_pose = None if vehicle_pose is None else vehicle_pose.copy()

        center = traj = None
        target_kmh = 0.0
        status = "LANE LOST - STOP"
        stop_brake = .7
        predicted = searching_now = False
        measured_width = None
        support = min(observation.left_extent_m, observation.right_extent_m)
        sample_x = np.linspace(5., max(7., min(15., support or 15.)), 3)
        if left_valid and right_valid:
            widths = left(sample_x) - right(sample_x)
            if (np.isfinite(widths).all() and np.all((widths >= 2.5) & (widths <= 4.5))
                    and np.ptp(widths) < .8):
                measured_width = float(np.median(widths))
                center, status = (left + right) * .5, "BOTH LANES"
            else:
                status = "LANE WIDTH INVALID - STOP"
                if prior is not None and self.width_frames >= 5:
                    candidates = (left - self.lane_width/2, right + self.lane_width/2)
                    errors = [float(np.max(np.abs(c(sample_x) - prior(sample_x)))) for c in candidates]
                    best = int(np.argmin(errors))
                    if np.isfinite(errors).all() and errors[best] < .4 and errors[1-best] > max(.5, 1.4*errors[best]):
                        center = candidates[best]
                        left_valid, right_valid = best == 0, best == 1
                        status = "LEFT + VIRTUAL RIGHT" if left_valid else "RIGHT + VIRTUAL LEFT"
        elif left_valid or right_valid:
            center = left - self.lane_width/2 if left_valid else right + self.lane_width/2
            status = "LEFT + VIRTUAL RIGHT" if left_valid else "RIGHT + VIRTUAL LEFT"

        if missing and not self.searching:
            self.searching = True
            self.search_center = Polynomial([0.])
            self.reacquire_center = None
            self.reacquire_frames = 0
        jump_candidate = False
        if center is not None:
            check = center(np.array([0., 6., 15., 25.]))
            if not np.isfinite(check).all():
                status, center = "INVALID PATH - STOP", None
            elif abs(float(center(0.))) > 2.0:
                status, center = "PATH TOO FAR - STOP", None
            elif self.searching:
                jump_candidate = True
                extent = max(observation.left_extent_m, observation.right_extent_m)
                if self._consistent_candidate(center, motion_known, hits=2, extent=extent):
                    self.searching = False
                    self.search_center = None
                    self.valid_frames = max(self.valid_frames, 5)
                else:
                    center = None
            elif prior is not None and abs(float(center(6.) - prior(6.))) > self.max_path_jump:
                jump_candidate = True
                if not self._consistent_candidate(center, motion_known) or not motion_known:
                    status, center = "PATH JUMP - STOP", None
        keep_pending = self.searching and self.reacquire_age <= .3
        if (not jump_candidate and not keep_pending) or center is not None:
            self.reacquire_center = None
            self.reacquire_frames = 0

        if self.searching:
            # A fixed world heading prevents the remembered bend from steering
            # through a blank intersection. Without pose, use vehicle-forward.
            center = self.search_center if self.search_center is not None else Polynomial([0.])
            predicted = searching_now = True
            left_valid = right_valid = False
            status = "NO LANES - STRAIGHT" if missing else "REACQUIRING - STRAIGHT"
            target_kmh = min(desired, self.search_speed)
            self.straight_seconds = 0.0
            self.gap_seconds = self.gap_distance = 0.0
        elif center is None:
            # Rejected, contradictory observations keep the old bounded guard.
            self.gap_seconds += dt
            self.gap_distance += distance
            prior_valid = (prior is not None and np.isfinite(prior(np.array([0.,6.,15.,25.]))).all()
                           and abs(float(prior(0.))) <= 2.)
            if (motion_known and prior_valid and self.was_driving
                    and self.gap_seconds <= self.max_gap_seconds + 1e-9
                    and self.gap_distance <= self.max_gap_distance + 1e-9):
                center, predicted, status = prior, True, "VIRTUAL LANES"
                left_valid = right_valid = False
                target_kmh = min(desired, self.previous_speed, self.single_lane_speed)
            elif self.previous_center is not None and (self.gap_seconds > self.max_gap_seconds
                                                       or self.gap_distance > self.max_gap_distance):
                status = "LANE GAP EXPIRED - STOP"

        if center is not None:
            if not predicted:
                extent = max(observation.left_extent_m if left_valid else 0.,
                             observation.right_extent_m if right_valid else 0.)
                target_kmh = min(desired, self.observed_speed(center, extent, left_valid and right_valid, dt))
                if prior is not None:
                    center = (1-self.smoothing)*center + self.smoothing*prior
                self.valid_frames += 1
                self.gap_seconds = self.gap_distance = 0.0
                if measured_width is not None:
                    self.lane_width = .95*self.lane_width + .05*measured_width
                    self.width_frames += 1
            self.previous_center = center
            if not searching_now:
                target_kmh = min(target_kmh, self.calculate_curve_speed(center))
            x = np.arange(-2., 31.)
            traj = np.column_stack((x+.5, -center(x)))
            if observation.centerline is not None:
                # Include the vehicle half-width plus clearance, and inspect
                # only the observed near road. This also guards straight search.
                x = np.linspace(0., min(10., observation.centerline_extent_m), 21)
                median = observation.centerline
                clearance = (median(x)-center(x))/np.sqrt(1+median.deriv()(x)**2)
                if np.min(clearance) < 1.1:
                    self.centerline_stop = "CENTERLINE CROSSING - STOP"
                    self.previous_speed = 0.
                    return DrivePlan(None, 0., self.centerline_stop, False, 1.,
                                     float(self.lane_width), False, False)
        else:
            self.valid_frames = self.width_frames = 0
            self.previous_center = None
            self.straight_seconds = 0.0
            self.gap_seconds = self.gap_distance = 0.0

        drive_allowed = False
        if self.frames <= 30:
            status, stop_brake = "WARMUP - STOP", 1.0
        elif traj is not None and (searching_now or self.valid_frames >= 5):
            drive_allowed = True
        elif traj is not None:
            status = "CHECKING LANE - STOP"
        if not drive_allowed:
            target_kmh = self.previous_speed = 0.
        else:
            target_kmh = min(desired, self.limit_speed_change(target_kmh, dt))
            self.previous_speed = target_kmh
        self.was_driving = drive_allowed and target_kmh > 0.
        return DrivePlan(traj, float(target_kmh), status, drive_allowed, stop_brake,
                         float(self.lane_width), left_valid, right_valid)
