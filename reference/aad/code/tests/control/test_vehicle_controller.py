import unittest
from dataclasses import replace
import numpy as np

from ...solutions.interfaces import DrivePlan
from ...solutions.control.vehicle_controller import VehicleController


def plan(y=0):
    trajectory = np.column_stack((np.arange(-1.5, 31.5), np.full(33, y)))
    return DrivePlan(trajectory, 20, 'BOTH LANES', True, .7, 3.5, True, True)


class VehicleControllerTests(unittest.TestCase):
    def test_underspeed_accelerates_with_new_cap(self):
        command = VehicleController().update(plan(), 10/3.6, 1/30)
        self.assertEqual(command.throttle, .75)
        self.assertEqual(command.brake, 0)
        self.assertEqual(command.target_kmh, 20)

    def test_overspeed_brakes_without_accelerating(self):
        command = VehicleController().update(plan(), 25/3.6, 1/30)
        self.assertEqual(command.throttle, 0)
        self.assertGreater(command.brake, 0)

    def test_stop_plan_brakes_and_keeps_previous_steering(self):
        controller = VehicleController()
        controller.previous_steer = .15
        stopped = replace(plan(), trajectory=None, target_kmh=0,
                          drive_allowed=False, status='LANE LOST - STOP')
        command = controller.update(stopped, 3, 1/30)
        self.assertEqual((command.throttle, command.brake, command.steer), (0, .7, .15))

    def test_unreachable_path_overrides_drive_permission_with_stop(self):
        command = VehicleController().update(plan(y=100), 3, 1/30)
        self.assertEqual(command.status, 'NO STEERING TARGET - STOP')
        self.assertEqual(command.target_kmh, 0)
        self.assertEqual(command.throttle, 0)
        self.assertEqual(command.brake, .7)

    def test_steering_respects_existing_rate_limit(self):
        command = VehicleController().update(plan(y=1), 3, 1/30)
        self.assertGreater(command.steer, 0)
        self.assertLessEqual(command.steer, .8/30)

    def test_integral_compensates_persistent_small_speed_deficit(self):
        controller = VehicleController()
        first = controller.update(plan(), 19/3.6, 1/30)
        for _ in range(60):
            command = controller.update(plan(), 19/3.6, 1/30)
        self.assertGreater(command.throttle, first.throttle)
        at_target = controller.update(plan(), 20/3.6, 1/30)
        self.assertGreater(at_target.throttle, 0)
        self.assertEqual(at_target.brake, 0)

    def test_full_throttle_does_not_wind_up_integral(self):
        controller = VehicleController()
        for _ in range(300):
            command = controller.update(plan(), 0, 1/30)
        self.assertEqual(command.throttle, .75)
        self.assertEqual(controller.integral_throttle, 0)

    def test_stop_and_missing_target_clear_speed_compensation(self):
        for stopped in (replace(plan(), drive_allowed=False, status='LANE LOST - STOP'),
                        plan(y=100), replace(plan(), target_kmh=0)):
            controller = VehicleController()
            for _ in range(60):
                controller.update(plan(), 19/3.6, 1/30)
            self.assertGreater(controller.integral_throttle, 0)
            result = controller.update(stopped, 19/3.6, 1/30)
            self.assertEqual(controller.integral_throttle, 0)
            self.assertEqual(result.throttle, 0)
            self.assertGreater(result.brake, 0)

    def test_lower_target_discards_previous_acceleration_and_brakes(self):
        controller = VehicleController()
        for _ in range(60):
            controller.update(plan(), 19/3.6, 1/30)
        command = controller.update(replace(plan(), target_kmh=8), 19/3.6, 1/30)
        self.assertEqual(controller.integral_throttle, 0)
        self.assertEqual(command.throttle, 0)
        self.assertGreater(command.brake, 0)

    def test_control_bounds_and_pedals_never_conflict(self):
        controller = VehicleController()
        for speed in list(np.linspace(0, 25, 120)) + list(np.linspace(25, 0, 120)):
            command = controller.update(plan(), speed/3.6, 1/30)
            self.assertGreaterEqual(command.throttle, 0)
            self.assertLessEqual(command.throttle, .75)
            self.assertGreaterEqual(command.brake, 0)
            self.assertLessEqual(command.brake, .6)
            self.assertEqual(command.throttle * command.brake, 0)
