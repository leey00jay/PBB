"""Write paired RGB images to a ROS 2 bag without starting a ROS node."""

import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


SCREEN_TOPIC = '/pbb/screen/image/compressed'
FRONT_TOPIC = '/pbb/front/image/compressed'


class ScreenBagRecorder:
    """Stream JPEG screen + lossless PNG camera frames to a new sqlite3 bag.

    Both images share the CARLA frame's time. The first frame is anchored to
    wall time; subsequent timestamps preserve simulation-time differences.
    ROS imports are deferred so ordinary driving does not require ROS 2.
    """

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        self.writer = None
        self.frames = 0
        self._first_sim_ns = None
        self._last_sim_ns = None
        self._first_wall_ns = None
        self.info = {
            'storage_id': 'sqlite3',
            'message_type': 'sensor_msgs/msg/CompressedImage',
            'timestamp_rule': 'first_wall_ns + (carla_sim_ns - first_sim_ns)',
            'topics': {
                SCREEN_TOPIC: {'encoding': 'jpeg', 'quality': 85, 'messages': 0},
                FRONT_TOPIC: {'encoding': 'png', 'lossless': True, 'messages': 0},
            },
            'notes': 'Images only. No vehicle commands, TF, or CARLA replay state.',
        }

    def __enter__(self):
        if self.path.exists():
            raise FileExistsError(f'Bag folder already exists: {self.path}. Choose a new name.')
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import CompressedImage
        except ImportError as error:
            raise RuntimeError(
                'ROS 2 recording modules unavailable. On this notebook run: '
                'source /opt/ros/humble/setup.bash') from error
        self._serialize = serialize_message
        self._message_type = CompressedImage
        self.path.parent.mkdir(parents=True, exist_ok=True)
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(self.path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
        self.writer = writer
        try:
            for topic in self.info['topics']:
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic, type=self.info['message_type'], serialization_format='cdr'))
        except BaseException:
            self.close()
            raise
        print(f'ROS bag recording: {self.path}\nStop with Ctrl+C and wait for "ROS bag saved".')
        return self

    def _encode(self, rgb, stamp_ns, *, screen):
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or 0 in rgb.shape:
            raise ValueError('Expected a non-empty H x W x 3 uint8 RGB image.')
        codec = 'jpeg' if screen else 'png'
        parameters = [cv2.IMWRITE_JPEG_QUALITY, 85] if screen else [cv2.IMWRITE_PNG_COMPRESSION, 1]
        ok, encoded = cv2.imencode('.' + codec, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), parameters)
        if not ok:
            raise RuntimeError(f'Failed to encode {codec} frame.')
        message = self._message_type()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.header.frame_id = 'pbb_screen' if screen else 'pbb_front_camera_optical'
        message.format = f'rgb8; {codec} compressed bgr8'
        message.data = encoded.tobytes()
        return self._serialize(message)

    def write(self, screen_rgb, front_rgb, simulation_seconds):
        if self.writer is None:
            raise RuntimeError('Bag is not open.')
        if not math.isfinite(simulation_seconds) or simulation_seconds < 0:
            raise ValueError('Simulation timestamp must be finite and non-negative.')
        sim_ns = round(simulation_seconds * 1_000_000_000)
        if self._last_sim_ns is not None and sim_ns <= self._last_sim_ns:
            raise ValueError('Simulation timestamps must increase for each frame.')
        first_sim_ns = sim_ns if self._first_sim_ns is None else self._first_sim_ns
        first_wall_ns = time.time_ns() if self._first_wall_ns is None else self._first_wall_ns
        stamp_ns = first_wall_ns + sim_ns - first_sim_ns
        # Encode both before writing either; invalid input cannot leave half a pair.
        screen = self._encode(screen_rgb, stamp_ns, screen=True)
        front = self._encode(front_rgb, stamp_ns, screen=False)
        for topic, data, rgb in ((SCREEN_TOPIC, screen, screen_rgb), (FRONT_TOPIC, front, front_rgb)):
            self.writer.write(topic, data, stamp_ns)
            self.info['topics'][topic].update(width=rgb.shape[1], height=rgb.shape[0])
            self.info['topics'][topic]['messages'] += 1
        self._first_sim_ns = first_sim_ns
        self._first_wall_ns = first_wall_ns
        self._last_sim_ns = sim_ns
        self.frames += 1

    def close(self):
        if self.writer is None:
            return
        writer, self.writer = self.writer, None
        # Older ROS 2 releases finalize metadata on writer destruction instead.
        if hasattr(writer, 'close'):
            writer.close()
        del writer
        self.info.update(
            paired_frames=self.frames,
            first_sim_ns=self._first_sim_ns,
            last_sim_ns=self._last_sim_ns,
            first_wall_ns=self._first_wall_ns)
        (self.path / 'recording_info.json').write_text(
            json.dumps(self.info, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'ROS bag saved: {self.path} ({self.frames} paired frames)')

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
