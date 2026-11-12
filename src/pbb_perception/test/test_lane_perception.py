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
    def test_bright_neural_input_uses_original_paint_to_reject_lifted_reflections(self):
        from pbb_perception.night_vision import NightVision
        detector=SyntheticDetector(left=False,right=False)
        original=detector.rgb.copy()
        bright=NightVision(True).apply(original)
        _,y=detector.grid.T.reshape(2,*detector.left.shape)
        bright[np.abs(y-1.75)<.065]=180
        received=[]
        detect=detector.detect
        detector.detect=lambda rgb: (received.append(rgb.copy()) or detect(rgb))
        observed=LanePerception(detector).observe(bright,paint_rgb=original)
        np.testing.assert_array_equal(received[0],bright)
        self.assertIsNone(observed.left)
        self.assertIsNone(observed.right)
        # Verified original paint is still usable with a bright neural image.
        original[np.abs(y+1.75)<.065]=70
        observed=LanePerception(detector).observe(NightVision(True).apply(original),paint_rgb=original)
        self.assertIsNotNone(observed.right)

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

    def test_distant_dashes_need_a_consistent_existing_track(self):
        detector = SyntheticDetector()
        perception = LanePerception(detector)
        perception.observe(detector.rgb)
        x = detector.grid[:, 0].reshape(detector.left.shape)
        detector.left[x < 9] = 0
        tracked = perception.observe(detector.rgb)
        self.assertIsNotNone(tracked.left)
        self.assertIsNone(LanePerception(detector).observe(detector.rgb).left)
        # Existing track must not authorize an unrelated distant marking.
        detector.left[:] = np.roll(detector.left, 20, axis=1)
        self.assertIsNone(perception.observe(detector.rgb).left)

    def test_short_visible_curve_does_not_become_a_straight_line(self):
        detector = SyntheticDetector(right=False)
        x, y = detector.grid.T.reshape(2, *detector.left.shape)
        detector.left = ((np.abs(y - (1.75 - .012 * x**2)) < .06)
                         & (x <= 11)).astype(float) * .9
        observed = LanePerception(detector).observe(detector.rgb)
        self.assertIsNotNone(observed.left)
        self.assertAlmostEqual(observed.left.coef[2], -.012, delta=.002)

    def test_current_paint_recovers_curve_without_any_neural_lane_pixels(self):
        detector = SyntheticDetector(left=False, right=False)
        x, y = detector.grid.T.reshape(2, *detector.left.shape)
        stripe = np.abs(y - (1.75 - .012*x*x)) < .065
        detector.rgb[stripe] = 180
        perception = LanePerception(detector)
        observed = perception.observe(detector.rgb)
        self.assertIsNotNone(observed.left)
        self.assertAlmostEqual(observed.left.coef[2], -.012, delta=.002)
        self.assertGreater(observed.left_extent_m, 18)
        self.assertEqual(perception.last_diagnostics['left']['source'], 'paint_tracking')

    def test_paint_tracking_uses_new_pixels_beyond_old_fifteen_frame_expiry(self):
        detector = SyntheticDetector(left=False, right=False)
        _, y = detector.grid.T.reshape(2, *detector.left.shape)
        detector.rgb[np.abs(y-1.75) < .065] = 70  # dim paint with local contrast
        perception = LanePerception(detector)
        for _ in range(25):
            observed = perception.observe(detector.rgb)
            self.assertIsNotNone(observed.left)
        detector.rgb[:] = 0
        self.assertIsNone(perception.observe(detector.rgb).left)

    def test_transverse_broad_paint_is_not_a_lane(self):
        detector = SyntheticDetector(left=False, right=False)
        x, _ = detector.grid.T.reshape(2, *detector.left.shape)
        detector.rgb[(x > 7) & (x < 9)] = 200
        observed = LanePerception(detector).observe(detector.rgb)
        self.assertIsNone(observed.left)
        self.assertIsNone(observed.right)

    def test_short_dim_model_fragment_updates_existing_curve(self):
        detector = SyntheticDetector(right=False)
        x,y = detector.grid.T.reshape(2, *detector.left.shape)
        stripe = np.abs(y-(1.75-.012*x*x)) < .11
        detector.left = stripe.astype(float)*.9
        perception = LanePerception(detector)
        self.assertIsNotNone(perception.observe(detector.rgb).left)
        detector.left = (stripe & (x < 6.5)).astype(float)*.4
        observed = perception.observe(detector.rgb)
        self.assertIsNotNone(observed.left)
        self.assertAlmostEqual(observed.left.coef[2], -.012, delta=.002)
        self.assertLess(observed.left_extent_m, 7.)

    def test_visible_tight_curve_is_not_rejected_for_far_tangent(self):
        detector = SyntheticDetector(right=False)
        x,y = detector.grid.T.reshape(2, *detector.left.shape)
        detector.left = ((np.abs(y-(1.75-.025*x*x)) < .08) & (x <= 15)).astype(float)*.9
        observed = LanePerception(detector).observe(detector.rgb)
        self.assertIsNotNone(observed.left)
        self.assertAlmostEqual(observed.left.coef[2], -.025, delta=.002)

    def test_side_camera_ground_coordinates_use_mount_translation_and_yaw(self):
        detector = SyntheticDetector(left=False, right=False)
        perception = LanePerception(detector)
        x,y = detector.grid.T
        expected_x = (x+y)/np.sqrt(2)
        expected_y = (-x+y)/np.sqrt(2)-.6
        np.testing.assert_allclose(perception._side_grids['right'][:,0], expected_x)
        np.testing.assert_allclose(perception._side_grids['right'][:,1], expected_y)

    def test_side_camera_points_extend_current_curve_outside_center_view(self):
        from pbb_perception.paint_tracking import fit_paint
        from numpy.polynomial import Polynomial
        detector = SyntheticDetector(left=False, right=False)
        x,y = detector.grid.T.reshape(2, *detector.left.shape)
        curve = Polynomial([-1.75, 0, -.015])
        # Center sees only a short near portion; side view supplies the bend.
        mask = (np.abs(y-curve(x)) < .08) & (x < 7)
        xx,offset = np.meshgrid(np.linspace(7,18,100), np.linspace(-.04,.04,5))
        extra = np.column_stack((xx.ravel(), (curve(xx)+offset).ravel()))
        recovered, info = fit_paint(mask, detector.grid, 0, 'right', curve, extra)
        self.assertIsNotNone(recovered)
        self.assertGreater(info['support_max_m'],17)
        np.testing.assert_allclose(recovered(np.array([5.,10.,15.])), curve(np.array([5.,10.,15.])), atol=.08)

    def test_opposite_model_boundary_reseeds_corrupted_paint_track(self):
        from numpy.polynomial import Polynomial
        detector = SyntheticDetector(right=False)
        x,y = detector.grid.T.reshape(2, *detector.left.shape)
        detector.left = (np.abs(y-(1.75-.012*x*x)) < .08).astype(float)*.9
        detector.rgb[np.abs(y-(-1.75-.012*x*x)) < .08] = 180
        perception = LanePerception(detector)
        perception._tracks['right'] = Polynomial([-1.75, 0., .012])
        observed = perception.observe(detector.rgb)
        self.assertIsNotNone(observed.right)
        self.assertAlmostEqual(observed.right.coef[2], -.012, delta=.003)
        np.testing.assert_allclose(observed.left(np.array([5.,8.]))-
                                   observed.right(np.array([5.,8.])), 3.5, atol=.12)

    def test_curved_stripe_width_is_measured_across_its_tangent(self):
        from numpy.polynomial import Polynomial
        from pbb_perception.paint_tracking import fit_paint
        detector = SyntheticDetector(left=False, right=False)
        curve = Polynomial([-1.75, 0., -.025])
        xx,offset = np.meshgrid(np.linspace(4,15,300),np.linspace(-.13,.13,11))
        extra = np.column_stack((xx.ravel(),(curve(xx)+offset).ravel()))
        recovered,info = fit_paint(np.zeros_like(detector.left,dtype=bool),
                                  detector.grid,0,'right',curve,extra)
        self.assertIsNotNone(recovered)
        self.assertGreater(info['support_max_m'],14)
        np.testing.assert_allclose(recovered(np.array([5.,10.,14.])),
                                   curve(np.array([5.,10.,14.])),atol=.08)
