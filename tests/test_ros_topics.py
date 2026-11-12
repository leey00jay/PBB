"""Typed-message and live DDS checks; skipped when ROS workspace is not sourced."""
import copy
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
try:
    import numpy as np
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from pbb_interfaces.msg import FrameState, LaneEstimate, VehicleControl
    from pbb_common.ros import FrameJoin, image_array, image_message, valid_command, stop_command, valid_state
    from pbb_common.wire import encode_lanes, decode_lanes
except ImportError:
    ROS_AVAILABLE = False
else:
    ROS_AVAILABLE = True


@unittest.skipUnless(ROS_AVAILABLE, 'Source ROS and install/setup.bash')
class MessageTests(unittest.TestCase):
    def setUp(self):
        self.state = FrameState(header=Header(stamp=Time(sec=1, nanosec=2), frame_id='carla_vehicle'),
                                session_id='run-a', frame=10, speed_mps=2.0, dt=1/30, max_steer_rad=1.2)

    def test_image_roundtrip_and_padding(self):
        image = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
        message = image_message(image, self.state.header)
        decoded = deserialize_message(serialize_message(message), type(message))
        np.testing.assert_array_equal(image_array(decoded), image)
        decoded.step = 14
        decoded.data = b''.join(row.tobytes() + b'XX' for row in image)
        np.testing.assert_array_equal(image_array(decoded), image)

    def test_no_cross_frame_join_and_bounded_cache(self):
        join = FrameJoin(('state', 'image'), capacity=2)
        h1, h2 = Header(stamp=Time(sec=1)), Header(stamp=Time(sec=2))
        self.assertIsNone(join.put('state', h1, 1))
        self.assertIsNone(join.put('image', h2, 2))
        self.assertEqual(join.put('state', h2, 2), {'state': 2, 'image': 2})
        self.assertIsNone(join.put('image', h1, 1))
        for sec in range(3, 20):
            join.put('state', Header(stamp=Time(sec=sec)), sec)
        self.assertLessEqual(len(join.frames), 2)

    def test_scaled_polynomial_survives_cdr(self):
        from pbb_common.models import LaneObservation
        from numpy.polynomial import Polynomial
        poly = Polynomial.fit([4, 10, 20], [1.8, 2.0, 2.7], deg=2)
        observation = LaneObservation(poly, None, np.empty((0, 0, 3), np.uint8), 22.5, 0.,
                                      centerline=poly+.15, centerline_extent_m=20.,
                                      right_double_yellow=True)
        wire = encode_lanes(observation, self.state)
        restored = decode_lanes(deserialize_message(serialize_message(wire), LaneEstimate))
        samples = np.array([0, 5, 15, 25])
        np.testing.assert_allclose(restored.left(samples), poly(samples))
        self.assertIsNone(restored.right)
        self.assertEqual(restored.left_extent_m, 22.5)
        np.testing.assert_allclose(restored.centerline(samples),(poly+.15)(samples))
        self.assertEqual(restored.centerline_extent_m,20.)
        self.assertTrue(restored.right_double_yellow)
        bad = copy.deepcopy(wire)
        bad.centerline_coefficients = [float('nan')]
        with self.assertRaises(ValueError):
            decode_lanes(bad)
        wire.left_extent_m = float('nan')
        with self.assertRaises(ValueError):
            decode_lanes(wire)

    def test_old_session_frame_and_invalid_controls_rejected(self):
        good = VehicleControl(state=self.state, throttle=0.3, brake=0.0, steer=0.1,
                              target_kmh=20.0, status='BOTH LANES')
        self.assertTrue(valid_command(good, self.state))
        for field, value in [('frame', 9), ('session_id', 'other-run')]:
            bad = copy.deepcopy(good)
            setattr(bad.state, field, value)
            self.assertFalse(valid_command(bad, self.state))
        for field, value in [('steer', float('nan')), ('throttle', 1.1), ('brake', 0.4),
                             ('target_kmh', 0.0), ('status', 'LANE LOST - STOP')]:
            bad = copy.deepcopy(good)
            setattr(bad, field, value)
            self.assertFalse(valid_command(bad, self.state))
        stopped = stop_command(self.state, 'COMMAND TIMEOUT - STOP')
        self.assertEqual((stopped.throttle, stopped.brake), (0.0, 1.0))
        self.assertTrue(valid_command(stopped, self.state))

    def test_pose_survives_cdr_and_nonfinite_pose_is_rejected(self):
        self.state.pose_x_m, self.state.pose_y_m, self.state.yaw_rad = 12.0, -7.0, .3
        decoded = deserialize_message(serialize_message(self.state), FrameState)
        self.assertEqual((decoded.pose_x_m, decoded.pose_y_m, decoded.yaw_rad), (12., -7., .3))
        self.assertTrue(valid_state(decoded))
        for field in ('pose_x_m', 'pose_y_m', 'yaw_rad'):
            bad = copy.deepcopy(decoded)
            setattr(bad, field, float('nan'))
            self.assertFalse(valid_state(bad))


@unittest.skipUnless(ROS_AVAILABLE, 'Requires sourced ROS workspace')
class TopicIntegrationTests(unittest.TestCase):
    def test_independent_decision_and_control_exchange_real_dds_messages(self):
        domain = 100 + os.getpid() % 20
        context = rclpy.Context()
        rclpy.init(context=context, domain_id=domain)
        node = rclpy.create_node('pbb_pipeline_test', context=context)
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self.addCleanup(lambda: context.shutdown() if context.ok() else None)
        self.addCleanup(node.destroy_node)
        self.addCleanup(executor.shutdown)
        received = []
        node.create_subscription(VehicleControl, '/pbb/control/command', received.append, 4)
        publisher = node.create_publisher(LaneEstimate, '/pbb/perception/lanes', 4)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        with tempfile.TemporaryDirectory(prefix='pbb-dds-test-') as folder:
            processes = []
            logs = []
            try:
                for role in ('decision', 'control'):
                    log = open(Path(folder) / f'{role}.log', 'w+')
                    logs.append(log)
                    process = subprocess.Popen([sys.executable, '-u', '-c',
                        f'from pbb_{role}.node import main; main()', '--ros-args',
                        '-p', 'desired_speed_kmh:=20.0'],
                        env=env, stdout=log, stderr=subprocess.STDOUT)
                    processes.append(process)
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                    if (publisher.get_subscription_count() and node.count_publishers('/pbb/control/command')
                            and node.count_subscribers('/pbb/decision/plan')):
                        break
                else:
                    self.fail('DDS graph did not connect')
                for frame in range(1, 36):
                    state = FrameState(header=Header(stamp=Time(sec=frame), frame_id='carla_vehicle'),
                                       session_id='dds-test', frame=frame, speed_mps=0.0, dt=1/30, max_steer_rad=1.2)
                    publisher.publish(LaneEstimate(state=state, left_valid=True, right_valid=True,
                                                  left_coefficients=[1.75], right_coefficients=[-1.75]))
                    deadline = time.monotonic() + 3
                    while not any(m.state.frame == frame for m in received) and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.05)
                    self.assertTrue(any(m.state.frame == frame for m in received), f'No command for frame {frame}')
                last = received[-1]
                self.assertEqual(last.status, 'BOTH LANES')
                self.assertGreater(last.throttle, 0)
                self.assertEqual(last.brake, 0.0)
                self.assertEqual(last.state.session_id, 'dds-test')
                # Blank paint with valid pose reaches the controller as a
                # drivable prediction; a new simulator session cannot inherit it.
                for frame, session in ((36, 'dds-test'), (37, 'dds-test'),
                                       (38, 'dds-test'), (39, 'new-session')):
                    state = FrameState(header=Header(stamp=Time(sec=frame)),
                        session_id=session, frame=frame, speed_mps=0.0, dt=1/30, max_steer_rad=1.2)
                    publisher.publish(LaneEstimate(state=state, right_double_yellow=frame == 37))
                    deadline = time.monotonic() + 3
                    while not any(m.state.frame == frame for m in received) and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.05)
                    messages = [m for m in received if m.state.frame == frame]
                    self.assertTrue(messages, f'No command for frame {frame}')
                    if frame == 36:
                        self.assertEqual(messages[-1].status, 'NO LANES - STRAIGHT')
                        self.assertGreater(messages[-1].throttle, 0)
                        self.assertEqual(messages[-1].brake, 0)
                    elif frame in (37,38):
                        self.assertEqual(messages[-1].status,'RIGHT DOUBLE YELLOW - STOP')
                        self.assertEqual((messages[-1].throttle,messages[-1].brake),(0.,1.))
                    else:
                        self.assertEqual(messages[-1].throttle, 0)
                        self.assertGreater(messages[-1].brake, .5)
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                for process in processes:
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                for log in logs:
                    log.close()


if __name__ == '__main__':
    unittest.main()
