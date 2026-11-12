import unittest
import numpy as np
from numpy.polynomial import Polynomial
from pbb_perception.camera_geometry import CameraGeometry
from pbb_perception.centerline import double_yellow
from pbb_perception.lane_perception import LanePerception
from pbb_decision.lane_planner import LanePlanner
from pbb_control.vehicle_controller import VehicleController


class CameraDetector:
    def __init__(self):
        self.cut_v, self.grid = CameraGeometry(image_width=512, image_height=256).precompute_grid()
        self.rgb = np.zeros((256,512,3),np.uint8)

    def detect(self, rgb):
        mask = np.zeros(rgb.shape[:2])
        return mask, mask, mask


def stripe_image(detector, grid, curves, color=(210,210,210)):
    rgb = detector.rgb.copy()
    x,y = grid.T
    selected = np.zeros(len(x),bool)
    for curve in curves:
        selected |= (np.abs(y-curve(x)) < .055) & (x > 1.5) & (x < 22)
    rgb[detector.cut_v:][selected.reshape(rgb.shape[0]-detector.cut_v,512)] = color
    return rgb


class CenterlineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = CameraDetector()

    def test_yellow_pair_is_distinct_from_single_white_or_broad_paint(self):
        d = self.detector
        curves = [Polynomial([1.65]),Polynomial([1.95])]
        image = stripe_image(d,d.grid,curves,(220,200,40))
        median, extent = double_yellow(image,d.grid,d.cut_v)
        self.assertIsNotNone(median)
        self.assertGreater(extent,8)
        np.testing.assert_allclose(median(np.array([4.,6.,8.])),1.8,atol=.08)
        for image in (stripe_image(d,d.grid,curves),
                      stripe_image(d,d.grid,curves[:1],(220,200,40)),
                      np.full_like(d.rgb,(220,200,40))):
            self.assertIsNone(double_yellow(image,d.grid,d.cut_v)[0])

    def test_either_side_camera_alone_reacquires_adjacent_lane(self):
        d = self.detector
        # At this mounting height/FOV, an adjacent line enters the side image
        # when displaced ~3 m sideways; a centered straight stripe is invisible.
        for side,offset in (('left',3.),('right',-3.)):
            perception = LanePerception(d)
            image = stripe_image(d,perception._side_grids[side],[Polynomial([offset])])
            obs = perception.observe(d.rgb,side_images={side:image})
            curve = getattr(obs,side)
            self.assertIsNotNone(curve,perception.last_diagnostics)
            self.assertLess(getattr(obs,side+'_extent_m'),8)
            np.testing.assert_allclose(curve(np.array([2.,4.])),offset,atol=.12)
            planner = LanePlanner()
            for _ in range(40):
                blank = perception.observe(d.rgb)
                planner.plan(blank,60)
            self.assertTrue(planner.searching)
            planner.plan(obs,60)
            plan = planner.plan(obs,60)
            self.assertFalse(planner.searching)
            self.assertTrue(plan.drive_allowed)
            expected = -(offset-np.sign(offset)*1.75)*.5
            np.testing.assert_allclose(plan.trajectory[:8,1],expected,atol=.15)

    def test_right_camera_yellow_pair_reaches_full_brake(self):
        d = self.detector
        perception = LanePerception(d)
        image = stripe_image(d,perception._side_grids['right'],
                             [Polynomial([-2.85]),Polynomial([-3.15])],(220,200,40))
        obs = perception.observe(d.rgb,side_images={'right':image})
        self.assertTrue(obs.right_double_yellow)
        plan = LanePlanner().plan(obs,60)
        cmd = VehicleController().update(plan,10,1/30)
        self.assertEqual((cmd.throttle,cmd.brake),(0.,1.))

    def test_curves_in_camera_pixels_produce_matching_carla_steering_signs(self):
        d = self.detector
        # Ground y is left-positive; CARLA steering is right-positive.
        for curvature, expected in ((-.012,1),(.012,-1)):
            perception = LanePerception(d)
            curve = Polynomial([0,0,curvature])
            image = stripe_image(d,d.grid,[curve+1.75,curve-1.75])
            obs = perception.observe(image)
            self.assertIsNotNone(obs.left)
            self.assertIsNotNone(obs.right)
            planner,controller = LanePlanner(),VehicleController()
            for _ in range(100):
                plan = planner.plan(obs,25)
                cmd = controller.update(plan,5,1/30)
            self.assertGreater(expected*cmd.steer,.01)
            self.assertTrue(plan.drive_allowed)
