"""Central RGB + matching state -> lane curves, overlay and boundary preview."""
import cv2
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from pbb_interfaces.msg import FrameState, LaneEstimate
from pbb_common.ros import FrameJoin, IMAGE_QOS, QOS, image_array, image_message, spin_node
from pbb_common.wire import encode_lanes


class Perception(Node):
    def __init__(self):
        super().__init__('pbb_perception')
        from pbb_perception.camera_geometry import CameraGeometry
        from pbb_perception.lane_detector import LaneDetector
        from pbb_perception.lane_perception import LanePerception
        from pbb_perception.road_boundary_recovery import BoundaryTracker, draw_result
        default_model = Path(get_package_share_directory('pbb_perception')) / 'models/fastai_model.pth'
        model_path = self.declare_parameter('model_path', str(default_model)).value
        detector = LaneDetector(cam_geom=CameraGeometry(image_width=512, image_height=256),
                                model_path=model_path)
        self.algorithm = LanePerception(detector)
        self.boundary = BoundaryTracker()
        self.draw_boundary = draw_result
        self.join = FrameJoin(('image', 'state'))
        self.lanes = self.create_publisher(LaneEstimate, '/pbb/perception/lanes', QOS)
        self.overlay = self.create_publisher(Image, '/pbb/perception/overlay', IMAGE_QOS)
        self.preview = self.create_publisher(Image, '/pbb/perception/boundary', IMAGE_QOS)
        self.create_subscription(FrameState, '/pbb/vehicle/state',
                                 lambda m: self.receive('state', m.header, m), QOS)
        self.create_subscription(Image, '/pbb/camera/center/image_raw',
                                 lambda m: self.receive('image', m.header, m), IMAGE_QOS)
        self.get_logger().info('Perception ready: subscribed to center image and matching vehicle state.')

    def receive(self, name, header, value):
        row = self.join.put(name, header, value)
        if row is None:
            return
        rgb = image_array(row['image'])
        state = row['state']
        observation = self.algorithm.observe(rgb)
        self.lanes.publish(encode_lanes(observation, state))
        self.overlay.publish(image_message(observation.overlay, row['image'].header))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        result = self.boundary.update(bgr, timestamp)
        preview = cv2.cvtColor(self.draw_boundary(bgr, result), cv2.COLOR_BGR2RGB)
        self.preview.publish(image_message(preview, row['image'].header))


def main():
    spin_node(Perception)
