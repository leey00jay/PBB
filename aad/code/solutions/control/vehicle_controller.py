"""제어: Pure Pursuit steering and PI control of vehicle speed."""
import numpy as np

from ..interfaces import ControlCommand
from .pure_pursuit import PurePursuit


class VehicleController:
    def __init__(self):
        self.max_steer_rad = np.deg2rad(70.0)
        self.controller = PurePursuit(K_dd=0.8, min_lookahead=7.0, max_lookahead=12.0)
        self.previous_steer = 0.0
        self.max_throttle = 0.75
        self.speed_kp = 0.6
        self.speed_ki = 0.3
        self.integral_throttle = 0.0
        self.previous_target_kmh = None

    def _reset_speed(self):
        self.integral_throttle = 0.0
        self.previous_target_kmh = None

    def _speed_control(self, target_kmh, speed, dt):
        # Discard compensation from a higher target when slowing down.
        if self.previous_target_kmh is not None and target_kmh < self.previous_target_kmh:
            self.integral_throttle = 0.0
        self.previous_target_kmh = target_kmh
        error = target_kmh / 3.6 - speed
        proportional = self.speed_kp * error
        # Anti-windup: do not accumulate more throttle while already saturated.
        if proportional + self.integral_throttle < self.max_throttle or error < 0:
            self.integral_throttle = float(np.clip(
                self.integral_throttle + self.speed_ki * error * dt,
                0.0, self.max_throttle))
        demand = proportional + self.integral_throttle
        # One signed demand prevents simultaneous accelerator and brake.
        return (float(np.clip(demand, 0.0, self.max_throttle)),
                float(np.clip(-demand, 0.0, 0.6)))

    def update(self, plan, speed, dt):
        """speed: m/s; dt: simulation seconds; plan.target_kmh: km/h."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be a positive simulation time interval.')
        throttle, brake, steer = 0.0, plan.stop_brake, self.previous_steer
        status, target_kmh = plan.status, plan.target_kmh
        if plan.drive_allowed and target_kmh > 0:
            angle = self.controller.get_control(plan.trajectory, speed)
            if self.controller.has_target:
                wanted = float(np.clip(angle / self.max_steer_rad, -0.6, 0.6))
                steer = float(np.clip(wanted, self.previous_steer - 0.8*dt,
                                      self.previous_steer + 0.8*dt))
                throttle, brake = self._speed_control(target_kmh, speed, dt)
            else:
                status = 'NO STEERING TARGET - STOP'
                self._reset_speed()
        else:
            self._reset_speed()
        if 'STOP' in status:
            target_kmh = 0.0
        self.previous_steer = steer
        return ControlCommand(throttle, brake, steer, float(target_kmh), status)
