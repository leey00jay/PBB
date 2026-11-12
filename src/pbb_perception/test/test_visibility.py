import unittest
import numpy as np
from numpy.polynomial import Polynomial
from pbb_perception.visibility import straight_extent


class VisibilityTests(unittest.TestCase):
    def setUp(self):
        self.x,self.y=np.meshgrid(np.linspace(4,50,185),np.linspace(-3,3,121),indexing='ij')
        self.grid=np.column_stack((self.x.ravel(),self.y.ravel()))
        self.curve=Polynomial([-1.75])
        self.mask=np.abs(self.y+1.75)<.08

    def extent(self,mask,curve=None):
        return straight_extent(mask,self.grid,0,self.curve if curve is None else curve,25.)

    def test_continuous_actual_distant_paint_extends_visibility(self):
        self.assertGreaterEqual(self.extent(self.mask),44.)

    def test_tiny_near_fit_curvature_does_not_hide_actual_distant_straight(self):
        noisy=Polynomial([-1.75+.0002*64,-.0002*16,.0002])
        self.assertGreaterEqual(self.extent(self.mask,noisy),44.)

    def test_blank_interval_cannot_be_bridged_to_far_paint(self):
        self.assertEqual(self.extent(self.mask & ~((self.x>27)&(self.x<36))),25.)

    def test_one_pixel_far_stripe_needs_continuous_near_paint(self):
        thin = self.mask & ((self.x<=35)|(np.abs(self.y+1.75)<.01))
        self.assertGreaterEqual(self.extent(thin),44.)
        self.assertEqual(self.extent(thin & (self.x>35)),25.)

    def test_curves_do_not_authorize_distant_straight_speed(self):
        curve=Polynomial([-1.75,0,.012])
        self.assertEqual(self.extent(np.abs(self.y-curve(self.x))<.08,curve),25.)

    def test_unrelated_far_lane_and_transverse_paint_do_not_extend(self):
        self.assertEqual(self.extent(np.abs(self.y+.9)<.08),25.)
        self.assertEqual(self.extent((self.x>20)&(self.x<45)),25.)
