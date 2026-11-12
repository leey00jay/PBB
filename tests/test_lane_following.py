"""Regression checks for missing lane boundaries and controller failure handling."""
import unittest
import numpy as np
from pbb_control.pure_pursuit import PurePursuit
from tests.support.pipeline import LaneFollower


from tests.support.synthetic import SyntheticDetector


class LaneFollowingTests(unittest.TestCase):
    def ready(self, detector, speed=2.0, target=20.0):
        follower = LaneFollower(detector)
        result = None
        # Allow the new acceleration ramp to reach the requested speed.
        for _ in range(150):
            result = follower.update(detector.rgb, speed, target, 1/30)
        return follower, result

    def test_missing_left_does_not_shift_center_toward_right(self):
        _, result = self.ready(SyntheticDetector(left=False))
        self.assertLess(np.max(np.abs(result['trajectory'][:,1])), .05)
        self.assertEqual(result['diagnostics']['target_kmh'], 20.0)
        self.assertGreater(result['throttle'], 0)

    def test_missing_right_does_not_shift_center_toward_left(self):
        _, result = self.ready(SyntheticDetector(right=False))
        self.assertLess(np.max(np.abs(result['trajectory'][:,1])), .05)
        self.assertEqual(result['diagnostics']['target_kmh'], 20.0)

    def test_speed_ramp_uses_actual_simulation_timestep(self):
        detector = SyntheticDetector()
        follower = LaneFollower(detector)
        for _ in range(30):
            follower.update(detector.rgb, 0, 20, 1/30)
        first = follower.update(detector.rgb, 0, 20, 1/60)
        second = follower.update(detector.rgb, 0, 20, 1/30)
        # Daytime 3.0 m/s²: 1/60 s adds 0.18 km/h; 1/30 s adds 0.36.
        self.assertAlmostEqual(first['diagnostics']['target_kmh'], .18)
        self.assertAlmostEqual(second['diagnostics']['target_kmh'], .54)

    def test_both_missing_after_warmup_requests_straight_search(self):
        _, result = self.ready(SyntheticDetector(left=False, right=False))
        np.testing.assert_allclose(result['trajectory'][:, 1], 0)
        self.assertGreater(result['throttle'], 0)
        self.assertEqual(result['brake'], 0)
        self.assertEqual(result['diagnostics']['status'], 'NO LANES - STRAIGHT')

    def test_implausible_lane_width_brakes(self):
        _, result = self.ready(SyntheticDetector(width=6.0))
        self.assertIsNone(result['trajectory'])
        self.assertEqual(result['throttle'], 0)
        self.assertIn('WIDTH INVALID', result['diagnostics']['status'])

    def test_detected_lane_obeys_lower_user_speed(self):
        _, result = self.ready(SyntheticDetector(left=False), speed=1, target=8)
        self.assertEqual(result['diagnostics']['target_kmh'], 8)

    def test_loss_without_pose_requests_vehicle_forward(self):
        detector = SyntheticDetector()
        follower, _ = self.ready(detector)
        detector.left[:] = detector.right[:] = 0
        lost = follower.update(detector.rgb, 3, 20, 1/30)
        np.testing.assert_allclose(lost['trajectory'][:, 1], 0)
        self.assertGreater(lost['throttle'], 0)
        self.assertEqual(lost['brake'], 0)

    def test_single_noisy_pixel_is_not_a_boundary(self):
        detector = SyntheticDetector(left=False,right=False)
        detector.right[10,10]=1
        _, result=self.ready(detector)
        self.assertFalse(result['diagnostics']['left_valid'] or result['diagnostics']['right_valid'])

    def test_blank_section_stays_straight_past_old_prediction_expiry(self):
        detector = SyntheticDetector()
        follower, _ = self.ready(detector)
        follower.update(detector.rgb, 3, 20, 1/30, vehicle_pose=(0, 0, 0))
        detector.left[:] = detector.right[:] = 0
        for i in range(1, 31):
            result = follower.update(detector.rgb, 3, 20, 1/30, vehicle_pose=(i * .1, 0, 0))
            self.assertEqual(result['diagnostics']['status'], 'NO LANES - STRAIGHT')
            self.assertGreater(result['throttle'], 0)
            self.assertEqual(result['brake'], 0)
        for i in range(31, 62):
            result = follower.update(detector.rgb, 3, 20, 1/30, vehicle_pose=(i * .1, 0, 0))
        np.testing.assert_allclose(result['trajectory'][:, 1], 0, atol=1e-10)
        self.assertGreater(result['throttle'], 0)
        self.assertEqual(result['brake'], 0)

    def test_no_target_never_changes_input_waypoints(self):
        points = np.column_stack((np.arange(-2.,60.),np.full(62,5.)))
        original = points.copy(); pp=PurePursuit()
        for _ in range(3):
            self.assertEqual(pp.get_control(points,20/3.6),0)
            self.assertFalse(pp.has_target)
            np.testing.assert_array_equal(points,original)

    def test_normal_steering_never_changes_input_waypoints(self):
        points=np.column_stack((np.arange(-2.,60.),np.ones(62)))
        original=points.copy();pp=PurePursuit()
        self.assertGreater(pp.get_control(points,20/3.6),0)
        self.assertTrue(pp.has_target)
        np.testing.assert_array_equal(points,original)

if __name__ == '__main__':
    unittest.main()
