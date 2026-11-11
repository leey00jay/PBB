"""Connect the three parts while keeping the existing CARLA update interface."""
from ..perception.lane_perception import LanePerception
from ..decision.lane_planner import LanePlanner
from ..control.vehicle_controller import VehicleController


class LaneFollower:
    def __init__(self, detector):
        self.perception = LanePerception(detector)
        self.decision = LanePlanner()
        self.control = VehicleController()

    @property
    def max_steer_rad(self):
        return self.control.max_steer_rad

    @max_steer_rad.setter
    def max_steer_rad(self, value):
        # Existing CARLA entry points set this from the spawned vehicle.
        self.control.max_steer_rad = value

    def update(self, rgb, speed, desired_speed_kmh, dt):
        observation = self.perception.observe(rgb)
        plan = self.decision.plan(observation, desired_speed_kmh, dt=dt)
        command = self.control.update(plan, speed, dt)
        diagnostics = {'status': command.status,
                       'target_kmh': command.target_kmh,
                       'lane_width_m': plan.lane_width_m,
                       'left_valid': plan.left_valid,
                       'right_valid': plan.right_valid}
        return {'throttle': command.throttle, 'brake': command.brake,
                'steer': command.steer, 'trajectory': plan.trajectory,
                'overlay': observation.overlay, 'diagnostics': diagnostics}
