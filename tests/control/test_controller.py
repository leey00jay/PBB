"""Real official PID and CARLA API data types, without a simulator server."""

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import carla
from agents.navigation import controller as upstream
from agents.navigation import local_planner

from pbb.control.controller import (
    PIDLateralController,
    PIDLongitudinalController,
    VehiclePIDController,
    create_controller,
)


class Vehicle:
    def __init__(self, speed_kmh=0.0):
        self.speed_kmh = speed_kmh

    def get_world(self):
        return None

    def get_control(self):
        return carla.VehicleControl()

    def get_velocity(self):
        return carla.Vector3D(x=self.speed_kmh / 3.6)

    def get_transform(self):
        return carla.Transform()


def waypoint(x=10.0, y=0.0):
    return SimpleNamespace(transform=carla.Transform(carla.Location(x=x, y=y)))


class OfficialControllerTests(unittest.TestCase):
    def test_entry_point_is_the_same_class_used_by_official_local_planner(self):
        self.assertIs(VehiclePIDController, upstream.VehiclePIDController)
        self.assertIs(VehiclePIDController, local_planner.VehiclePIDController)
        self.assertIs(PIDLateralController, upstream.PIDLateralController)
        self.assertIs(PIDLongitudinalController, upstream.PIDLongitudinalController)

    def test_straight_target_accelerates_with_tested_throttle_limit(self):
        command = create_controller(Vehicle()).run_step(15.0, waypoint())
        self.assertIsInstance(command, carla.VehicleControl)
        self.assertAlmostEqual(command.throttle, 0.55)
        self.assertEqual(command.brake, 0.0)
        self.assertEqual(command.steer, 0.0)

    def test_overspeed_applies_tested_brake_limit_without_throttle(self):
        command = create_controller(Vehicle(20.0)).run_step(0.0, waypoint())
        self.assertEqual(command.throttle, 0.0)
        self.assertAlmostEqual(command.brake, 0.6)

    def test_target_speed_is_in_kmh(self):
        command = create_controller(Vehicle(5.0)).run_step(5.0, waypoint())
        self.assertAlmostEqual(command.throttle, 0.0, places=5)
        self.assertAlmostEqual(command.brake, 0.0, places=5)

    def test_left_and_right_targets_have_correct_sign(self):
        left = create_controller(Vehicle()).run_step(5.0, waypoint(y=-3.0))
        right = create_controller(Vehicle()).run_step(5.0, waypoint(y=3.0))
        self.assertLess(left.steer, 0.0)
        self.assertGreater(right.steer, 0.0)
        self.assertAlmostEqual(left.steer, -right.steer)

    def test_steering_is_rate_limited_and_bounded(self):
        controller = create_controller(Vehicle())
        previous = 0.0
        for _ in range(20):
            command = controller.run_step(5.0, waypoint(x=1.0, y=10.0))
            self.assertLessEqual(abs(command.steer - previous), 0.100001)
            self.assertLessEqual(abs(command.steer), 0.800001)
            previous = command.steer
        self.assertAlmostEqual(previous, 0.8)

    def test_acceleration_and_braking_are_exclusive(self):
        for speed in (0.0, 5.0, 15.0, 30.0):
            for target in (0.0, 5.0, 15.0):
                with self.subTest(speed=speed, target=target):
                    command = create_controller(Vehicle(speed)).run_step(target, waypoint())
                    self.assertFalse(command.throttle > 0 and command.brake > 0)
                    self.assertGreaterEqual(command.throttle, 0.0)
                    self.assertGreaterEqual(command.brake, 0.0)

    def test_upstream_files_keep_the_original_verified_hashes(self):
        vendor = Path(__file__).resolve().parents[2] / "third_party" / "carla_agents"
        manifest = json.loads((vendor / "UPSTREAM.json").read_text())
        for name, expected in manifest["sha256"].items():
            with self.subTest(path=name):
                self.assertEqual(hashlib.sha256((vendor / name).read_bytes()).hexdigest(), expected)


if __name__ == "__main__":
    unittest.main()
