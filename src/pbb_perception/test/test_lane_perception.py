import unittest
import numpy as np

from pbb_perception.lane_perception import LanePerception
"""Synthetic lane masks shared by the part-level and runtime tests."""
import numpy as np


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
