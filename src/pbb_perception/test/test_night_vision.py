import unittest
import cv2
import numpy as np
from pbb_perception.night_vision import NightVision


class NightVisionTests(unittest.TestCase):
    def test_night_lifts_shadows_preserves_highlights_and_input(self):
        gray = np.tile(np.arange(256,dtype=np.uint8),(8,1))
        rgb = np.repeat(gray[:,:,None],3,axis=2)
        original = rgb.copy()
        result = NightVision(True).apply(rgb)
        np.testing.assert_array_equal(rgb,original)
        self.assertGreater(result[0,30,0],30)
        self.assertEqual(result[0,255,0],255)
        self.assertEqual(result[0,0,0],0)
        self.assertTrue(np.all(np.diff(result[0,:,0].astype(int))>=0))
        self.assertEqual(result.shape,rgb.shape)
        self.assertEqual(result.dtype,np.uint8)

    def test_daytime_and_gamma_one_are_exact_passthrough_copies(self):
        rgb = np.random.default_rng(8).integers(0,256,(24,32,3),dtype=np.uint8)
        for enhancer in (NightVision(False),NightVision(True,1)):
            result = enhancer.apply(rgb)
            np.testing.assert_array_equal(result,rgb)
            result[:]=0
            self.assertTrue(rgb.any())

    def test_yellow_lane_hue_is_preserved(self):
        rgb = np.full((4,5,3),[70,60,10],dtype=np.uint8)
        before = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
        after = cv2.cvtColor(NightVision(True,.65).apply(rgb),cv2.COLOR_RGB2HSV)
        self.assertLessEqual(np.max(np.abs(before[:,:,:2].astype(int)-after[:,:,:2])),2)
        self.assertTrue(np.all(after[:,:,2]>before[:,:,2]))

    def test_invalid_gamma_and_non_rgb_inputs_are_rejected(self):
        for gamma in (float('nan'),float('inf'),0.,1.1):
            with self.assertRaises(ValueError):NightVision(True,gamma)
        with self.assertRaises(ValueError):NightVision(True).apply(np.zeros((4,5),dtype=np.uint8))
