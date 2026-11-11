import unittest
from dataclasses import replace
import numpy as np

from ...solutions.interfaces import DrivePlan
from ...solutions.control.vehicle_controller import VehicleController


def plan(y=0):
    trajectory = np.column_stack((np.arange(-1.5, 31.5), np.full(33, y)))
    return DrivePlan(trajectory, 20, 'BOTH LANES', True, .7, 3.5, True, True)


class VehicleControllerTests(unittest.TestCase):
    def test_underspeed_accelerates_with_existing_cap(self):
        command = VehicleController().update(plan(), 10/3.6, 1/30)
        self.assertEqual(command.throttle, .4)
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
