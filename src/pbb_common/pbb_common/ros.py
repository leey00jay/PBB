"""Message conversion, exact-frame matching and shared ROS configuration."""
from array import array
from collections import OrderedDict
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

QOS = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)
IMAGE_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)


def stamp_key(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def image_message(rgb, header):
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    msg = Image(header=header, height=rgb.shape[0], width=rgb.shape[1],
                encoding='rgb8', is_bigendian=0, step=rgb.shape[1] * 3)
    # ROS accepts a typed buffer directly; bytes assignment checks every pixel in Python.
    # Keep uint8 conversion and the public message setter (no disabled assertions).
    msg.data = array('B', rgb.tobytes())
    return msg


def image_array(msg):
    if msg.encoding != 'rgb8' or msg.step < msg.width * 3 or not msg.height or not msg.width:
        raise ValueError('Expected nonempty rgb8 image')
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if len(data) != msg.height * msg.step:
        raise ValueError('Invalid image data length')
    return data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()


class FrameJoin:
    """Bounded cache; release only messages carrying exactly the same timestamp."""
    def __init__(self, names, capacity=8):
        self.names = set(names)
        self.capacity = capacity
        self.frames = OrderedDict()
        self.last_emitted = -1

    def put(self, name, header, value):
        key = stamp_key(header)
        if key <= self.last_emitted:
            return None
        self.frames.setdefault(key, {})[name] = value
        while len(self.frames) > self.capacity:
            self.frames.popitem(last=False)
        row = self.frames.get(key, {})
        if not self.names.issubset(row):
            return None
        self.last_emitted = key
        self.frames = OrderedDict((k, v) for k, v in self.frames.items() if k > key)
        return row


def valid_state(state):
    return (bool(state.session_id) and state.frame > 0
            and all(math.isfinite(v) for v in (state.speed_mps, state.dt, state.max_steer_rad))
            and state.speed_mps >= 0 and 0 < state.dt <= 1 and 0 < state.max_steer_rad <= math.pi)


def valid_command(command, expected):
    """Reject old frames, other runs, malformed controls and conflicting pedals."""
    return (command.state.session_id == expected.session_id
            and command.state.frame == expected.frame
            and stamp_key(command.state.header) == stamp_key(expected.header)
            and all(math.isfinite(v) for v in (command.throttle, command.brake, command.steer, command.target_kmh))
            and 0 <= command.throttle <= 1 and 0 <= command.brake <= 1
            and -1 <= command.steer <= 1
            and command.target_kmh >= 0
            and not (command.throttle > 0 and command.brake > 0)
            and not (command.throttle > 0 and ('STOP' in command.status or command.target_kmh == 0)))


def stop_command(state, status):
    from pbb_interfaces.msg import VehicleControl
    return VehicleControl(state=state, throttle=0.0, brake=1.0, steer=0.0,
                          target_kmh=0.0, status=status)


def spin_node(factory):
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
