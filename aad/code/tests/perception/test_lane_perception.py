import unittest
import numpy as np

from ...solutions.perception.lane_perception import LanePerception
from ..helpers import SyntheticDetector


class LanePerceptionTests(unittest.TestCase):
    def test_missing_boundary_is_reported_without_inventing_a_lane(self):
        detector = SyntheticDetector(left=False)
        observation = LanePerception(detector).observe(detector.rgb)
        self.assertIsNone(observation.left)
        self.assertIsNotNone(observation.right)
        np.testing.assert_allclose(observation.right(np.array([5, 10, 15])), -1.75, atol=.05)

    def test_overlay_does_not_modify_input_camera_image(self):
        detector = SyntheticDetector()
        original = detector.rgb.copy()
        observation = LanePerception(detector).observe(detector.rgb)
        np.testing.assert_array_equal(detector.rgb, original)
        self.assertTrue(np.any(observation.overlay != original))

    def test_tiny_false_detection_is_not_a_boundary(self):
        detector = SyntheticDetector(left=False, right=False)
        detector.left[10, 10] = 1
        observation = LanePerception(detector).observe(detector.rgb)
        self.assertIsNone(observation.left)
        self.assertIsNone(observation.right)
