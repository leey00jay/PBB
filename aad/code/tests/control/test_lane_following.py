"""Regression checks for missing lane boundaries and controller failure handling."""
import unittest
import numpy as np
from ...solutions.control.pure_pursuit import PurePursuit
from ...solutions.lane_detection.lane_following import LaneFollower


class SyntheticDetector:
    def __init__(self, left=True, right=True, width=3.5):
        self.cut_v = 0
        x, y = np.meshgrid(np.linspace(4.5, 25, 80), np.linspace(-4, 4, 161), indexing='ij')
        self.grid = np.column_stack((x.ravel(), y.ravel()))
        self.left = (np.abs(y - width/2) < .06).astype(float)*.9 if left else np.zeros_like(x)
        self.right = (np.abs(y + width/2) < .06).astype(float)*.9 if right else np.zeros_like(x)
        self.rgb = np.zeros((*x.shape,3),np.uint8)
    def detect(self, rgb):
        return np.zeros_like(self.left), self.left, self.right


class LaneFollowingTests(unittest.TestCase):
    def ready(self, detector, speed=2.0, target=20.0):
        follower = LaneFollower(detector)
        result = None
        for _ in range(36):
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
