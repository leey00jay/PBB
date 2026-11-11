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
        # 1.5 m/s² over 1/60 s adds 0.09 km/h; 1/30 s adds 0.18.
        self.assertAlmostEqual(first['diagnostics']['target_kmh'], .09)
        self.assertAlmostEqual(second['diagnostics']['target_kmh'], .27)

    def test_both_missing_brakes_and_has_no_fake_path(self):
        _, result = self.ready(SyntheticDetector(left=False, right=False))
        self.assertIsNone(result['trajectory'])
        self.assertEqual(result['throttle'], 0)
        self.assertGreater(result['brake'], .5)

    def test_implausible_lane_width_brakes(self):
        _, result = self.ready(SyntheticDetector(width=6.0))
        self.assertIsNone(result['trajectory'])
        self.assertEqual(result['throttle'], 0)
        self.assertIn('WIDTH INVALID', result['diagnostics']['status'])

    def test_detected_lane_obeys_lower_user_speed(self):
        _, result = self.ready(SyntheticDetector(left=False), speed=1, target=8)
        self.assertEqual(result['diagnostics']['target_kmh'], 8)

    def test_lost_lane_cannot_continue_using_previous_good_path(self):
        detector = SyntheticDetector()
        follower, good = self.ready(detector)
        self.assertGreater(good['throttle'], 0)
        detector.left[:]=0;detector.right[:]=0
        lost = follower.update(detector.rgb, 3, 20, 1/30)
        self.assertIsNone(lost['trajectory'])
        self.assertEqual(lost['throttle'],0)
        self.assertGreater(lost['brake'],.5)

    def test_single_noisy_pixel_is_not_a_boundary(self):
        detector = SyntheticDetector(left=False,right=False)
        detector.right[10,10]=1
        _, result=self.ready(detector)
        self.assertIsNone(result['trajectory'])

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
