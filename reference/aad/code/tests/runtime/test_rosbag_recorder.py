"""Integration checks against ROS 2 serialization and actual bag storage.

Skipped on the numpy-only CI environment. On this notebook source the ROS 2
environment and run with the project's virtualenv Python.
"""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

try:
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
except ImportError:
    ROS_AVAILABLE = False
else:
    ROS_AVAILABLE = True
    from ...solutions.runtime.rosbag_recorder import ScreenBagRecorder, FRONT_TOPIC, SCREEN_TOPIC


@unittest.skipUnless(ROS_AVAILABLE, 'Requires ROS 2 Humble and OpenCV')
class BagRecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'test_drive'
        # Asymmetric dimensions and colors catch orientation/channel mistakes.
        self.front = np.random.default_rng(4).integers(0, 256, (32, 64, 3), dtype=np.uint8)
        self.screen = np.full((60, 120, 3), (220, 40, 10), dtype=np.uint8)

    def read(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.path), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        rows = []
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            rows.append((topic, deserialize_message(data, CompressedImage), stamp))
        return rows

    def test_paired_timestamps_color_and_lossless_front_roundtrip(self):
        with ScreenBagRecorder(self.path) as recorder:
            recorder.write(self.screen, self.front, 10.0)
            recorder.write(self.screen, self.front, 10.1)
        self.assertTrue((self.path / 'metadata.yaml').exists())
        rows = self.read()
        self.assertEqual(len(rows), 4)
        self.assertEqual([row[0] for row in rows], [SCREEN_TOPIC, FRONT_TOPIC] * 2)
        self.assertEqual(rows[0][2], rows[1][2])
        self.assertEqual(rows[2][2] - rows[0][2], 100_000_000)
        for topic, msg, stamp in rows:
            self.assertEqual(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, stamp)
            decoded = cv2.cvtColor(cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            if topic == FRONT_TOPIC:
                np.testing.assert_array_equal(decoded, self.front)
                self.assertEqual(msg.format, 'rgb8; png compressed bgr8')
            else:
                self.assertEqual(decoded.shape, self.screen.shape)
                self.assertLess(np.abs(decoded.astype(float) - self.screen).mean(), 3)
        info = json.loads((self.path / 'recording_info.json').read_text())
        self.assertEqual(info['paired_frames'], 2)
        self.assertEqual(info['topics'][FRONT_TOPIC]['messages'], 2)

    def test_interrupt_finalizes_readable_bag(self):
        with self.assertRaises(KeyboardInterrupt):
            with ScreenBagRecorder(self.path) as recorder:
                recorder.write(self.screen, self.front, 1)
                raise KeyboardInterrupt()
        self.assertTrue((self.path / 'metadata.yaml').exists())
        self.assertEqual(len(self.read()), 2)

    def test_existing_destination_is_not_overwritten(self):
        self.path.mkdir()
        sentinel = self.path / 'keep.txt'
        sentinel.write_text('keep')
        with self.assertRaises(FileExistsError):
            with ScreenBagRecorder(self.path):
                self.fail('Should not open an existing folder')
        self.assertEqual(sentinel.read_text(), 'keep')

    def test_invalid_images_and_timestamps_do_not_add_frames(self):
        with ScreenBagRecorder(self.path) as recorder:
            recorder.write(self.screen, self.front, 1)
            for stamp in (1, 0.9, -1, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    recorder.write(self.screen, self.front, stamp)
            with self.assertRaises(ValueError):
                recorder.write(self.screen, self.front.astype(float), 2)
            self.assertEqual(recorder.frames, 1)
        self.assertEqual(len(self.read()), 2)
