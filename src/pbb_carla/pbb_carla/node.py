"""Own CARLA actors and ticks; exchange images/state and commands ONLY via ROS."""
import math
import queue
import time
import uuid

import carla
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from rosgraph_msgs.msg import Clock
from pbb_interfaces.msg import FrameState, VehicleControl
from pbb_common.ros import IMAGE_QOS, QOS, image_message, stop_command, valid_command


class Bridge(Node):
    def __init__(self):
        super().__init__('pbb_bridge')
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('startup_timeout', 120.0)
        self.session = uuid.uuid4().hex
        self.actors = []
        self.world = self.original_settings = self.vehicle = None
        self.expected = self.command = None
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(80.0)
        self.image_pubs = {name: self.create_publisher(Image, f'/pbb/camera/{name}/image_raw', IMAGE_QOS)
                           for name in ('left', 'center', 'right', 'chase')}
        self.state_pub = self.create_publisher(FrameState, '/pbb/vehicle/state', QOS)
        self.applied_pub = self.create_publisher(VehicleControl, '/pbb/control/applied', QOS)
        self.clock_pub = self.create_publisher(Clock, '/clock', QOS)
        self.front_pub = self.create_publisher(CompressedImage, '/pbb/front/image/compressed', IMAGE_QOS)
        self.command_sub = self.create_subscription(VehicleControl, '/pbb/control/command', self.receive, QOS)
        self.queues = {}

    def receive(self, msg):
        if self.expected is not None and self.command is None and valid_command(msg, self.expected):
            self.command = msg

    def setup_world(self):
        self.world = self.client.load_world('Town10HD')
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / 30.0
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        self.world.set_weather(carla.WeatherParameters.ClearNoon)
        blueprints = self.world.get_blueprint_library()
        vehicle_bp = blueprints.find('vehicle.audi.tt')
        vehicle_bp.set_attribute('color', '64,81,181')
        self.vehicle = self.world.spawn_actor(vehicle_bp, self.world.get_map().get_spawn_points()[21])
        self.actors.append(self.vehicle)
        self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
        self.max_steer = math.radians(self.vehicle.get_physics_control().wheels[0].max_steer_angle)
        poses = {'left': (0.5, -0.6, 1.3, -5, -45), 'center': (0.5, 0, 1.3, -5, 0),
                 'right': (0.5, 0.6, 1.3, -5, 45), 'chase': (-5.5, 0, 2.8, -10, 0)}
        for name, (x, y, z, pitch, yaw) in poses.items():
            bp = blueprints.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '800' if name == 'chase' else '512')
            bp.set_attribute('image_size_y', '600' if name == 'chase' else '256')
            bp.set_attribute('fov', '90' if name == 'chase' else '45')
            bp.set_attribute('sensor_tick', '0.0')
            sensor = self.world.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z),
                                            carla.Rotation(pitch=pitch, yaw=yaw)), attach_to=self.vehicle)
            self.actors.append(sensor)
            self.queues[name] = queue.Queue(maxsize=4)
            sensor.listen(self.queues[name].put)
        self.get_logger().info('Town10 ready with left/center/right cameras and chase view.')

    def wait_pipeline(self):
        deadline = time.monotonic() + float(self.get_parameter('startup_timeout').value)
        while rclpy.ok():
            if (self.image_pubs['center'].get_subscription_count() > 0
                    and self.count_publishers('/pbb/perception/lanes') > 0
                    and self.count_publishers('/pbb/control/command') > 0
                    and self.count_subscribers('/pbb/perception/lanes') > 0
                    and self.count_subscribers('/pbb/decision/plan') > 0):
                return True
            if time.monotonic() >= deadline:
                raise RuntimeError('Perception/decision/control topic connections did not become ready')
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def run(self):
        self.setup_world()
        if not self.wait_pipeline():
            return
        self.get_logger().info('ROS topic pipeline connected. Starting synchronous driving.')
        previous_status = None
        while rclpy.ok():
            frame = self.world.tick(10.0)
            snapshot = self.world.get_snapshot()
            stamp_ns = round(snapshot.timestamp.elapsed_seconds * 1e9)
            stamp = Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
            state = FrameState(header=Header(stamp=stamp, frame_id='carla_vehicle'),
                               session_id=self.session, frame=frame,
                               speed_mps=float(self.vehicle.get_velocity().length()), dt=1.0 / 30,
                               max_steer_rad=self.max_steer)
            self.expected, self.command = state, None
            self.clock_pub.publish(Clock(clock=stamp))
            self.state_pub.publish(state)
            for name, images in self.queues.items():
                deadline = time.monotonic() + 10
                while True:
                    image = images.get(timeout=max(0.01, deadline - time.monotonic()))
                    if image.frame == frame:
                        break
                    if image.frame > frame or time.monotonic() > deadline:
                        raise RuntimeError(f'Camera frame mismatch: {name}')
                rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1].copy()
                header = Header(stamp=stamp, frame_id=f'pbb_{name}_camera_optical')
                self.image_pubs[name].publish(image_message(rgb, header))
                if name == 'center' and self.front_pub.get_subscription_count():
                    ok, encoded = cv2.imencode('.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 1])
                    if ok:
                        self.front_pub.publish(CompressedImage(header=header, format='rgb8; png compressed bgr8', data=encoded.tobytes()))
            deadline = time.monotonic() + float(self.get_parameter('command_timeout').value)
            while rclpy.ok() and self.command is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not rclpy.ok():
                break
            applied = self.command or stop_command(state, 'COMMAND TIMEOUT - STOP')
            self.vehicle.apply_control(carla.VehicleControl(throttle=applied.throttle,
                                                           brake=applied.brake, steer=applied.steer))
            self.applied_pub.publish(applied)
            if applied.status != previous_status:
                self.get_logger().info(f'frame={frame} {applied.status} target={applied.target_kmh:.1f} km/h')
                previous_status = applied.status

    def cleanup(self):
        self.client.set_timeout(2.0)
        if self.vehicle is not None:
            try:
                self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
            except RuntimeError:
                pass
        for actor in reversed(self.actors):
            try:
                if actor.type_id.startswith('sensor.') and actor.is_listening:
                    actor.stop()
                actor.destroy()
            except RuntimeError:
                pass
        if self.world is not None and self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except RuntimeError:
                pass
        self.get_logger().info('Owned CARLA actors destroyed; world settings restored when available.')


def main():
    rclpy.init()
    node = None
    try:
        node = Bridge()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
