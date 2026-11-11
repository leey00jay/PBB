import unittest
import numpy as np
from numpy.polynomial import Polynomial

from ...solutions.interfaces import LaneObservation
from ...solutions.decision.lane_planner import LanePlanner


def observation(left=1.75, right=-1.75):
    return LaneObservation(None if left is None else Polynomial([left]),
                           None if right is None else Polynomial([right]),
                           np.zeros((1, 1, 3), dtype=np.uint8))


class LanePlannerTests(unittest.TestCase):
    def ready(self, obs=None, target=20):
        obs = observation() if obs is None else obs
        planner = LanePlanner()
        for _ in range(36):
            plan = planner.plan(obs, target)
        return planner, plan

    def test_warmup_stops_for_30_frames_then_allows_valid_path(self):
        planner = LanePlanner()
        for _ in range(30):
            plan = planner.plan(observation(), 20)
            self.assertFalse(plan.drive_allowed)
            self.assertEqual(plan.target_kmh, 0)
            self.assertEqual(plan.stop_brake, 1)
        self.assertTrue(planner.plan(observation(), 20).drive_allowed)

    def test_single_boundary_uses_explicit_assumed_width_and_user_target(self):
        _, plan = self.ready(observation(left=None), target=8)
        self.assertEqual(plan.status, 'RIGHT ONLY / WIDTH ASSUMED')
        self.assertEqual(plan.target_kmh, 8)
        self.assertEqual(plan.lane_width_m, 3.5)
        np.testing.assert_allclose(plan.trajectory[:, 1], 0)

    def test_camera_left_offset_becomes_vehicle_right_negative(self):
        _, plan = self.ready(observation(left=2.0, right=-1.5))
        np.testing.assert_allclose(plan.trajectory[:, 1], -.25)
        self.assertEqual(plan.trajectory[0, 0], -1.5)

    def test_loss_clears_path_and_recovery_requires_five_valid_frames(self):
        planner, _ = self.ready()
        lost = planner.plan(observation(None, None), 20)
        self.assertIsNone(lost.trajectory)
        self.assertFalse(lost.drive_allowed)
        self.assertEqual(lost.target_kmh, 0)
        for _ in range(4):
            self.assertEqual(planner.plan(observation(), 20).status, 'CHECKING LANE - STOP')
        self.assertTrue(planner.plan(observation(), 20).drive_allowed)

    def test_implausible_width_and_abrupt_path_change_stop(self):
        planner, _ = self.ready()
        invalid = planner.plan(observation(3, -3), 20)
        self.assertFalse(invalid.drive_allowed)
        self.assertEqual(invalid.status, 'LANE WIDTH INVALID - STOP')
        planner, _ = self.ready()
        jumped = planner.plan(observation(2.75, -.75), 20)
        self.assertFalse(jumped.drive_allowed)
        self.assertEqual(jumped.status, 'PATH JUMP - STOP')
