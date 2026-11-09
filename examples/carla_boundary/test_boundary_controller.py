import copy
import unittest
from boundary_controller import vision_target, pursuit_steer, target_speed, longitudinal


def projected_corridor(lateral_m=0):
    result={'path_valid':True,'estimate_age_s':0.0}
    for name,side in [('yellow_line',-1.75),('curb_candidate',1.75)]:
        # A 3.5 m straight lane under the specified pinhole camera.
        result[name]=[640+(side+lateral_m)*(684-360)/1.82,684,
                      640+(side+lateral_m)*(403-360)/1.82,403]
    return result


class BoundaryControlTests(unittest.TestCase):
    def test_physical_projection_and_correction_direction(self):
        for lateral in [-.3,0,.3]:
            result=projected_corridor(lateral)
            before=copy.deepcopy(result)
            target=vision_target(result,1280,720)
            self.assertAlmostEqual(target[1],lateral)
            steer=pursuit_steer(target,3.0,70)
            self.assertAlmostEqual(steer,0) if lateral==0 else self.assertGreater(steer*lateral,0)
            self.assertEqual(result,before)

    def test_invalid_and_expired_detection_cannot_supply_steering(self):
        result=projected_corridor();result['path_valid']=False
        self.assertIsNone(vision_target(result,1280,720))
        result['path_valid']=True;result['estimate_age_s']=.201
        self.assertIsNone(vision_target(result,1280,720))

    def test_implausible_corridor_is_rejected(self):
        result=projected_corridor()
        result['curb_candidate']=result['yellow_line'].copy()
        self.assertIsNone(vision_target(result,1280,720))

    def test_red_and_yellow_approach_stop_then_green_releases(self):
        for state in ['Red','Yellow']:
            targets=[target_speed(3.3,state,gap) for gap in [20,5,2,1]]
            self.assertEqual(targets,sorted(targets,reverse=True))
            self.assertEqual(targets[-1],0)
            self.assertEqual(longitudinal(1,targets[-1]),(0,1))
        self.assertEqual(target_speed(3.3,'Green',1),3.3)
        throttle,brake=longitudinal(0,3.3)
        self.assertGreater(throttle,0);self.assertEqual(brake,0)

    def test_controls_are_bounded(self):
        for lateral in [-100,100]:
            self.assertLessEqual(abs(pursuit_steer((1,lateral),3,70)),.4)
        self.assertEqual(pursuit_steer((-.1,2),3,70),0)
        for speed in [0,2,10]:
            for target in [0,2,5]:
                throttle,brake=longitudinal(speed,target)
                self.assertTrue(0<=throttle<=1 and 0<=brake<=1)
                self.assertFalse(throttle>0 and brake>0)


if __name__=='__main__':unittest.main()
