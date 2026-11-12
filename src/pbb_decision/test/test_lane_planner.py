import unittest
from dataclasses import replace
import numpy as np
from numpy.polynomial import Polynomial

from pbb_common.models import LaneObservation
from pbb_decision.lane_planner import LanePlanner


def observation(left=1.75, right=-1.75):
    return LaneObservation(None if left is None else Polynomial([left]),
                           None if right is None else Polynomial([right]),
                           np.zeros((1, 1, 3), dtype=np.uint8),
                           left_extent_m=25.0 if left is not None else 0.0,
                           right_extent_m=25.0 if right is not None else 0.0)


class LanePlannerTests(unittest.TestCase):
    def test_one_noisy_straight_fragment_does_not_erase_stability_but_curve_slows(self):
        planner=LanePlanner()
        straight=Polynomial([0.])
        for _ in range(40):
            planner.observed_speed(straight,44.,True,1/30)
        mild_noise=Polynomial([0.,0.,.0015])
        self.assertEqual(planner.observed_speed(mild_noise,44.,True,1/30),60.)
        self.assertLessEqual(planner.observed_speed(Polynomial([0,0,.015]),44.,True,1/30),28.)
        self.assertLess(planner.observed_speed(straight,7.,True,1/30),35.)

    def test_daytime_sixty_requires_actual_long_straight_visibility(self):
        for obs in (observation(), observation(left=None)):
            planner = LanePlanner(daytime=True)
            for _ in range(350):
                short = planner.plan(obs, 80)
            self.assertLess(short.target_kmh, 50)
            long = replace(obs, left_extent_m=44. if obs.left is not None else 0., right_extent_m=44.)
            for _ in range(350):
                fast = planner.plan(long, 80)
            self.assertEqual(fast.target_kmh, 60.)
            self.assertEqual(planner.plan(long, 20).target_kmh, 20.)

    def test_night_keeps_previous_acceleration_and_single_boundary_ceiling(self):
        planner = LanePlanner(daytime=False)
        obs = replace(observation(left=None), right_extent_m=44.)
        for _ in range(30):
            planner.plan(obs, 80)
        self.assertAlmostEqual(planner.plan(obs,80,dt=1/60).target_kmh,.09)
        for _ in range(400):
            plan = planner.plan(obs,80)
        self.assertEqual(plan.target_kmh,55.)

    def ready(self, obs=None, target=20):
        obs = observation() if obs is None else obs
        planner = LanePlanner()
        # Include enough driving frames for the speed ramp to settle.
        for _ in range(150):
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
        self.assertEqual(plan.status, 'RIGHT + VIRTUAL LEFT')
        self.assertEqual(plan.target_kmh, 8)
        self.assertAlmostEqual(plan.lane_width_m, 3.5)
        np.testing.assert_allclose(plan.trajectory[:, 1], 0)

    def test_polynomial_curve_speed_decreases_for_tighter_turns(self):
        planner = LanePlanner()
        straight = planner.calculate_curve_speed(Polynomial([0.0]))
        gentle = planner.calculate_curve_speed(Polynomial([0.0, 0.0, .005]))
        tight = planner.calculate_curve_speed(Polynomial([0.0, 0.0, .02]))
        self.assertEqual(straight, 80.0)
        self.assertGreater(tight, 0.0)
        self.assertLess(tight, gentle)
        self.assertLess(gentle, straight)

    def test_single_boundary_limits_high_requested_speed(self):
        for obs in (observation(left=None), observation(right=None)):
            planner = LanePlanner()
            for _ in range(350):
                plan = planner.plan(obs, 80.0)
            self.assertTrue(plan.drive_allowed)
            self.assertGreater(plan.target_kmh, 40.0)
            self.assertLessEqual(plan.target_kmh, 55.0)

    def test_camera_left_offset_becomes_vehicle_right_negative(self):
        _, plan = self.ready(observation(left=2.0, right=-1.5))
        np.testing.assert_allclose(plan.trajectory[:, 1], -.25)
        self.assertEqual(plan.trajectory[0, 0], -1.5)

    def test_loss_without_pose_uses_straight_search_and_confirmed_reacquisition(self):
        planner, _ = self.ready()
        lost = planner.plan(observation(None, None), 20)
        self.assertTrue(lost.drive_allowed)
        self.assertEqual(lost.status, 'NO LANES - STRAIGHT')
        np.testing.assert_allclose(lost.trajectory[:, 1], 0)
        self.assertEqual(planner.plan(observation(), 20).status, 'REACQUIRING - STRAIGHT')
        self.assertEqual(planner.plan(observation(), 20).status, 'BOTH LANES')

    def test_implausible_width_and_abrupt_path_change_stop(self):
        planner, _ = self.ready()
        invalid = planner.plan(observation(3, -3), 20)
        self.assertFalse(invalid.drive_allowed)
        self.assertEqual(invalid.status, 'LANE WIDTH INVALID - STOP')
        planner, _ = self.ready()
        jumped = planner.plan(observation(2.75, -.75), 20)
        self.assertFalse(jumped.drive_allowed)
        self.assertEqual(jumped.status, 'PATH JUMP - STOP')

    def ready_with_pose(self):
        planner = LanePlanner()
        for _ in range(150):
            planner.plan(observation(), 20, vehicle_pose=(0, 0, 0))
        return planner

    def test_short_gap_preserves_speed_and_reacquires_without_restart(self):
        planner = self.ready_with_pose()
        for i in range(1, 21):
            plan = planner.plan(observation(None, None), 20,
                                vehicle_pose=(i * .2, 0, 0))
            self.assertEqual(plan.status, 'NO LANES - STRAIGHT')
            self.assertTrue(plan.drive_allowed)
            self.assertEqual(plan.target_kmh, 20)
            self.assertFalse(plan.left_valid or plan.right_valid)
            np.testing.assert_allclose(plan.trajectory[:, 1], 0, atol=1e-10)
        for _ in range(3):
            recovered = planner.plan(observation(), 20, vehicle_pose=(4.2, 0, 0))
        self.assertTrue(recovered.drive_allowed)
        self.assertEqual(recovered.target_kmh, 20)
        self.assertEqual(recovered.status, 'BOTH LANES')

    def test_rejected_geometry_expires_by_elapsed_time_at_different_frame_rates(self):
        for dt in (1 / 20, 1 / 30, 1 / 60):
            planner = self.ready_with_pose()
            for _ in range(round(planner.max_gap_seconds / dt)):
                plan = planner.plan(observation(3, -3), 20, dt,
                                    vehicle_pose=(0, 0, 0))
                self.assertTrue(plan.drive_allowed)
            expired = planner.plan(observation(3, -3), 20, dt,
                                   vehicle_pose=(0, 0, 0))
            self.assertFalse(expired.drive_allowed)
            self.assertIsNone(expired.trajectory)
            self.assertEqual(expired.target_kmh, 0)
            for _ in range(4):
                self.assertFalse(planner.plan(observation(), 20, dt,
                                              vehicle_pose=(0, 0, 0)).drive_allowed)
            self.assertTrue(planner.plan(observation(), 20, dt,
                                          vehicle_pose=(0, 0, 0)).drive_allowed)

    def test_rejected_geometry_expires_by_distance_even_before_time_limit(self):
        planner = self.ready_with_pose()
        for x in range(1, 13):
            self.assertTrue(planner.plan(observation(3, -3), 20,
                                         vehicle_pose=(x, 0, 0)).drive_allowed)
        expired = planner.plan(observation(3, -3), 20, vehicle_pose=(13, 0, 0))
        self.assertEqual(expired.status, 'LANE GAP EXPIRED - STOP')
        self.assertFalse(expired.drive_allowed)

    def test_prediction_does_not_accelerate_or_ignore_lower_user_request(self):
        planner = self.ready_with_pose()
        self.assertEqual(planner.plan(observation(None, None), 80,
                                      vehicle_pose=(0, 0, 0)).target_kmh, 20)
        self.assertEqual(planner.plan(observation(None, None), 8,
                                      vehicle_pose=(0, 0, 0)).target_kmh, 8)

    def test_search_continues_past_old_timeout_and_distance_at_limited_speed(self):
        planner = LanePlanner()
        for i in range(400):
            plan = planner.plan(observation(None, None), 80, vehicle_pose=(i*.2, 0, 0))
        self.assertTrue(plan.drive_allowed)
        self.assertEqual(plan.status, 'NO LANES - STRAIGHT')
        self.assertEqual(plan.target_kmh, 20)
        self.assertFalse(plan.left_valid or plan.right_valid)
        np.testing.assert_allclose(plan.trajectory[:, 1], 0, atol=1e-10)

    def test_brief_gap_between_consistent_paint_does_not_reset_reacquisition(self):
        planner = self.ready_with_pose()
        planner.plan(observation(None, None), 20, vehicle_pose=(0,0,0))
        first = planner.plan(observation(2.,-1.5), 20, vehicle_pose=(0,0,0))
        self.assertEqual(first.status, 'REACQUIRING - STRAIGHT')
        planner.plan(observation(None,None), 20, vehicle_pose=(0,0,0))
        recovered = planner.plan(observation(2.,-1.5), 20, vehicle_pose=(0,0,0))
        self.assertEqual(recovered.status, 'BOTH LANES')
        self.assertTrue(recovered.drive_allowed)
        self.assertEqual(recovered.target_kmh, 20)

    def test_long_straight_single_boundary_uses_higher_speed_ceiling_and_true_detection_flags(self):
        planner = self.ready_with_pose()
        for _ in range(350):
            plan = planner.plan(observation(left=None), 80, vehicle_pose=(0, 0, 0))
        self.assertGreater(plan.target_kmh, 40)
        self.assertLessEqual(plan.target_kmh, 55)
        self.assertFalse(plan.left_valid)
        self.assertTrue(plan.right_valid)
        self.assertAlmostEqual(plan.lane_width_m, 3.5)

    def test_motion_transform_uses_camera_offset_and_yaw_sign(self):
        from pbb_common.motion import move_curve
        x = np.array([0., 6., 15., 25.])
        curve = Polynomial([.2, .03, .005])
        shifted = move_curve(curve, (0, 0, 0), (2, 0, 0))
        np.testing.assert_allclose(shifted(x), curve(x + 2), atol=1e-10)
        rotated = move_curve(Polynomial([0]), (0, 0, 0), (0, 0, .1))
        np.testing.assert_allclose(rotated(x), np.tan(.1) * (x + .5), atol=1e-10)

    def test_diverging_false_side_does_not_discard_a_confirmed_good_boundary(self):
        planner = self.ready_with_pose()
        obs = LaneObservation(Polynomial([1.75]), Polynomial([-1.75, -.1]), np.zeros((1, 1, 3)))
        plan = planner.plan(obs, 20, vehicle_pose=(0, 0, 0))
        self.assertTrue(plan.drive_allowed)
        self.assertTrue(plan.left_valid)
        self.assertFalse(plan.right_valid)
        self.assertEqual(plan.status, 'LEFT + VIRTUAL RIGHT')
        np.testing.assert_allclose(plan.trajectory[:, 1], 0, atol=1e-10)

    def test_curve_is_not_extrapolated_when_both_boundaries_disappear(self):
        planner = LanePlanner()
        curve = Polynomial([0, 0, -.01])
        curved = LaneObservation(curve + 1.75, curve - 1.75, np.zeros((1, 1, 3)))
        for _ in range(150):
            planner.plan(curved, 20, vehicle_pose=(0, 0, 0))
        plan = planner.plan(observation(None, None), 20, vehicle_pose=(2, .04, .04))
        self.assertTrue(plan.drive_allowed)
        self.assertEqual(plan.status, 'NO LANES - STRAIGHT')
        np.testing.assert_allclose(plan.trajectory[:, 1], 0, atol=1e-10)
        # Keep the heading at loss, even if residual steering rotates the car.
        plan = planner.plan(observation(None, None), 20, vehicle_pose=(2, .04, .06))
        slope = np.polyfit(plan.trajectory[:, 0], plan.trajectory[:, 1], 1)[0]
        self.assertLess(slope, 0)

    def test_curve_entry_slows_before_acceleration_resumes_on_straight(self):
        planner = LanePlanner()
        for _ in range(350):
            straight = planner.plan(observation(), 80, vehicle_pose=(0, 0, 0))
        curve = Polynomial([0, 0, -.015])
        obs = LaneObservation(curve+1.75, curve-1.75, np.zeros((1,1,3)), 22, 22)
        first = planner.plan(obs, 80, vehicle_pose=(0, 0, 0))
        self.assertLess(first.target_kmh, straight.target_kmh)
        for _ in range(120):
            bent = planner.plan(obs, 80, vehicle_pose=(0, 0, 0))
        self.assertLessEqual(bent.target_kmh, 35)
        one_straight = planner.plan(observation(), 80, vehicle_pose=(0, 0, 0))
        self.assertLess(one_straight.target_kmh, straight.target_kmh)

    def test_short_unknown_support_does_not_authorize_fast_straight_speed(self):
        planner = LanePlanner()
        obs = LaneObservation(None, Polynomial([-1.75]), np.zeros((1,1,3)), 0, 7)
        for _ in range(350):
            plan = planner.plan(obs, 80)
        self.assertLessEqual(plan.target_kmh, 35)

    def test_ambiguous_boundaries_do_not_refresh_prediction_and_eventually_stop(self):
        planner = self.ready_with_pose()
        obs = LaneObservation(Polynomial([1.75, .06]), Polynomial([-1.75, -.06]), np.zeros((1, 1, 3)))
        for _ in range(40):
            plan = planner.plan(obs, 20, vehicle_pose=(0, 0, 0))
            self.assertEqual(plan.status, 'VIRTUAL LANES')
            self.assertFalse(plan.left_valid or plan.right_valid)
            np.testing.assert_allclose(plan.trajectory[:, 1], 0, atol=1e-10)
        expired = planner.plan(obs, 20, vehicle_pose=(0, 0, 0))
        self.assertFalse(expired.drive_allowed)
        self.assertEqual(expired.status, 'LANE GAP EXPIRED - STOP')

    def test_rejected_path_jump_uses_prediction_and_recovers_without_speed_reset(self):
        planner = self.ready_with_pose()
        rejected = planner.plan(observation(2.75, -.75), 20, vehicle_pose=(0, 0, 0))
        self.assertEqual(rejected.status, 'VIRTUAL LANES')
        self.assertEqual(rejected.target_kmh, 20)
        np.testing.assert_allclose(rejected.trajectory[:, 1], 0, atol=1e-10)
        recovered = planner.plan(observation(), 20, vehicle_pose=(0, 0, 0))
        self.assertTrue(recovered.drive_allowed)
        self.assertEqual(recovered.target_kmh, 20)

    def test_consistent_reacquisition_can_correct_aged_prediction_without_stopping(self):
        planner = self.ready_with_pose()
        for _ in range(2):
            waiting = planner.plan(observation(2.75, -.75), 20, vehicle_pose=(0, 0, 0))
            self.assertEqual(waiting.status, 'VIRTUAL LANES')
            self.assertEqual(waiting.target_kmh, 20)
        reacquired = planner.plan(observation(2.75, -.75), 20, vehicle_pose=(0, 0, 0))
        self.assertTrue(reacquired.drive_allowed)
        self.assertEqual(reacquired.status, 'BOTH LANES')
        self.assertEqual(reacquired.target_kmh, 20)
        # Blend toward the confirmed measurement; retain controller steering slew limits.
        np.testing.assert_allclose(reacquired.trajectory[:, 1], -.5, atol=1e-10)

    def test_right_double_yellow_stops_even_during_search_and_stays_latched(self):
        from pbb_control.vehicle_controller import VehicleController
        planner = self.ready_with_pose()
        hazard = replace(observation(None,None),right_double_yellow=True)
        plan = planner.plan(hazard,60,vehicle_pose=(0,0,0))
        self.assertEqual(plan.status,'RIGHT DOUBLE YELLOW - STOP')
        command = VehicleController().update(plan,60/3.6,1/30)
        self.assertEqual((command.throttle,command.brake,command.target_kmh),(0,1,0))
        for obs in (observation(),observation(None,None)):
            self.assertFalse(planner.plan(obs,60).drive_allowed)
        # A new session/planner starts independently.
        self.assertIsNone(LanePlanner().centerline_stop)

    def test_search_and_detected_paths_cannot_cross_observed_centerline(self):
        for obs in (observation(None,None), observation()):
            planner = self.ready_with_pose()
            median = Polynomial([2.,-.16])
            plan = planner.plan(replace(obs,centerline=median,centerline_extent_m=15),20)
            self.assertEqual(plan.status,'CENTERLINE CROSSING - STOP')
            self.assertFalse(plan.drive_allowed)
        planner = self.ready_with_pose()
        safe = planner.plan(replace(observation(),centerline=Polynomial([1.9]),
                                    centerline_extent_m=20),20)
        self.assertTrue(safe.drive_allowed)
