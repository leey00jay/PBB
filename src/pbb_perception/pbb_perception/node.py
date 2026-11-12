"""Three matching RGB cameras + state -> lanes, overlay and curve preview."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from pbb_interfaces.msg import FrameState, LaneEstimate
from pbb_common.ros import FrameJoin, IMAGE_QOS, QOS, image_array, image_message, spin_node, valid_state
from pbb_common.wire import encode_lanes


class Perception(Node):
    def __init__(self):
        super().__init__('pbb_perception')
        from pbb_perception.camera_geometry import CameraGeometry
        from pbb_perception.lane_detector import LaneDetector
        from pbb_perception.lane_perception import LanePerception
        from pbb_perception.lane_preview import draw_observation
        from pbb_perception.night_vision import NightVision
        weather = self.declare_parameter('weather', 'ClearNoon', ParameterDescriptor(read_only=True)).value
        night = weather.endswith('Night')
        enabled = self.declare_parameter('night_enhancement', True, ParameterDescriptor(read_only=True)).value
        gamma = self.declare_parameter('night_gamma', .65, ParameterDescriptor(read_only=True)).value
        self.night_vision = NightVision(enabled=night and enabled, gamma=gamma)
        self.long_visibility = not night
        default_model = Path(get_package_share_directory('pbb_perception')) / 'models/fastai_model.pth'
        model_path = self.declare_parameter('model_path', str(default_model)).value
        detector = LaneDetector(cam_geom=CameraGeometry(image_width=512, image_height=256),
                                model_path=model_path)
        self.algorithm = LanePerception(detector, long_straight_visibility=self.long_visibility)
        self.session = None
        self.draw_boundary = draw_observation
        self.join = FrameJoin(('image', 'left', 'right', 'state'))
        self.lanes = self.create_publisher(LaneEstimate, '/pbb/perception/lanes', QOS)
        self.overlay = self.create_publisher(Image, '/pbb/perception/overlay', IMAGE_QOS)
        self.preview = self.create_publisher(Image, '/pbb/perception/boundary', IMAGE_QOS)
        self.create_subscription(FrameState, '/pbb/vehicle/state',
                                 lambda m: self.receive('state', m.header, m), QOS)
        self.create_subscription(Image, '/pbb/camera/center/image_raw',
                                 lambda m: self.receive('image', m.header, m), IMAGE_QOS)
        for side in ('left', 'right'):
            self.create_subscription(Image, f'/pbb/camera/{side}/image_raw',
                                     lambda m, s=side: self.receive(s, m.header, m), IMAGE_QOS)
        self.get_logger().info('Perception ready: matching left/center/right images and vehicle state.')
        self.get_logger().info(f'Night brightness: {self.night_vision.enabled}, gamma={gamma}; daylight long visibility: {self.long_visibility}')

    def receive(self, name, header, value):
        row = self.join.put(name, header, value)
        if row is None:
            return
        original = image_array(row['image'])
        rgb = self.night_vision.apply(original)
        state = row['state']
        if not valid_state(state):
            return
        if self.session is not None and self.session != state.session_id:
            from pbb_perception.lane_perception import LanePerception
            self.algorithm = LanePerception(self.algorithm.detector, long_straight_visibility=self.long_visibility)
        self.session = state.session_id
        observation = self.algorithm.observe(rgb,
            vehicle_pose=(state.pose_x_m, state.pose_y_m, state.yaw_rad),
            side_images={side: image_array(row[side]) for side in ('left', 'right')},
            paint_rgb=original)
        self.lanes.publish(encode_lanes(observation, state))
        self.overlay.publish(image_message(observation.overlay, row['image'].header))
        preview = self.draw_boundary(rgb, observation, self.algorithm.last_diagnostics,
                                     self.algorithm.detector.cg)
        self.preview.publish(image_message(preview, row['image'].header))


def main():
    spin_node(Perception)
