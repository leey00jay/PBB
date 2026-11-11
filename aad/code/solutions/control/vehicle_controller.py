"""제어: follow a vehicle-frame path using the existing control settings."""
import numpy as np

from ..interfaces import ControlCommand
from .pure_pursuit import PurePursuit


class VehicleController:
    def __init__(self):
        self.max_steer_rad = np.deg2rad(70.0)
        self.controller = PurePursuit(K_dd=0.8, min_lookahead=7.0, max_lookahead=12.0)
        self.previous_steer = 0.0

    def update(self, plan, speed, dt):
        """speed: m/s; dt: simulation seconds; plan.target_kmh: km/h."""
        throttle, brake, steer = 0.0, plan.stop_brake, self.previous_steer
        status, target_kmh = plan.status, plan.target_kmh
        if plan.drive_allowed:
            angle = self.controller.get_control(plan.trajectory, speed)
            if self.controller.has_target:
                wanted = float(np.clip(angle / self.max_steer_rad, -0.6, 0.6))
                steer = float(np.clip(wanted, self.previous_steer - 0.8*dt,
                                      self.previous_steer + 0.8*dt))
                error = target_kmh / 3.6 - speed
                throttle = float(np.clip(0.6 * error, 0.0, 0.4))
                brake = float(np.clip(-0.4 * error, 0.0, 0.6))
            else:
                status = 'NO STEERING TARGET - STOP'
        if 'STOP' in status:
            target_kmh = 0.0
        self.previous_steer = steer
        return ControlCommand(throttle, brake, steer, float(target_kmh), status)
