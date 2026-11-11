"""Camera-only lane following for the Town10 practice demo.

A single visible boundary uses an assumed 3.5 m lane width until two reliable
boundaries update it. This is a fallback estimate, not detection of both lanes.
No CARLA map, waypoint, or autopilot data is used for steering.
"""
import numpy as np
from ..control.pure_pursuit import PurePursuit


class LaneFollower:
    def __init__(self, detector):
        self.detector = detector
        self.lane_width = 3.5
        self.max_steer_rad = np.deg2rad(70.0)
        self.controller = PurePursuit(K_dd=0.8, min_lookahead=7.0, max_lookahead=12.0)
        self.previous_center = None
        self.previous_steer = 0.0
        self.valid_frames = 0
        self.frames = 0

    def fit_boundary(self, probabilities):
        """Reject tiny/noisy detections and fit only the nearby visible road."""
        cg = self.detector
        weights = probabilities[cg.cut_v:, :].ravel()
        x, y = cg.grid.T
        selected = ((weights > 0.5) & (x >= 4.0) & (x <= 25.0)
                    & (np.abs(y) < 8.0))
        if selected.sum() < 30:
            return None
        x, y, weights = x[selected], y[selected], weights[selected]
        # One point per half-meter avoids over-weighting thick/nearby markings.
        bins = np.floor(x * 2).astype(int)
        points = []
        for key in np.unique(bins):
            use = bins == key
            points.append((np.average(x[use], weights=weights[use]),
                           np.average(y[use], weights=weights[use]),
                           float(weights[use].mean())))
        points = np.asarray(points)
        if len(points) < 6 or np.ptp(points[:, 0]) < 3.0 or points[:, 0].min() > 8:
            return None
        x, y, weights = points.T
        degree = 2 if np.ptp(x) >= 8.0 else 1
        for _ in range(3):
            fit = np.polynomial.Polynomial.fit(x, y, degree, w=weights).convert()
            residual = np.abs(fit(x) - y)
            keep = residual < max(0.2, 2.5 * float(np.median(residual)))
            if keep.all():
                break
            if keep.sum() < 6 or np.ptp(x[keep]) < 3.0:
                return None
            x, y, weights = x[keep], y[keep], weights[keep]
        fit = np.polynomial.Polynomial.fit(x, y, degree, w=weights).convert()
        check = np.array([5.0, 10.0, 15.0])
        if (not np.isfinite(fit.coef).all()
                or np.max(np.abs(fit(check))) > 6.0
                or np.max(np.abs(fit.deriv()(check))) > 0.7
                or np.mean(np.abs(fit(x) - y)) > 0.25):
            return None
        return fit

    def update(self, rgb, speed, desired_speed_kmh, dt):
        self.frames += 1
        _, left_mask, right_mask = self.detector.detect(rgb)
        overlay = rgb.copy()
        overlay[left_mask > 0.5] = [0, 0, 255]
        overlay[right_mask > 0.5] = [255, 0, 0]
        left = self.fit_boundary(left_mask)
        right = self.fit_boundary(right_mask)
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

        throttle, brake, steer = 0.0, 0.7, self.previous_steer
        if self.frames <= 30:
            status = 'WARMUP - STOP'
            brake = 1.0
        elif traj is not None and self.valid_frames >= 5:
            angle = self.controller.get_control(traj, speed)
            if self.controller.has_target:
                wanted = float(np.clip(angle / self.max_steer_rad, -0.6, 0.6))
                steer = float(np.clip(wanted, self.previous_steer - 0.8*dt,
                                      self.previous_steer + 0.8*dt))
                # Track the requested speed when the estimated path is valid.
                error = target_kmh / 3.6 - speed
                throttle = float(np.clip(0.6 * error, 0.0, 0.4))
                brake = float(np.clip(-0.4 * error, 0.0, 0.6))
            else:
                status = 'NO STEERING TARGET - STOP'
        elif traj is not None:
            status = 'CHECKING LANE - STOP'
        if 'STOP' in status:
            target_kmh = 0.0
        self.previous_steer = steer
        diagnostics = {'status': status, 'target_kmh': float(target_kmh),
                       'lane_width_m': float(self.lane_width),
                       'left_valid': left is not None, 'right_valid': right is not None}
        return {'throttle': throttle, 'brake': brake, 'steer': steer,
                'trajectory': traj, 'overlay': overlay, 'diagnostics': diagnostics}
