"""LaneEstimate topic -> DrivePlan topic, using the team's existing planner."""
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from pbb_interfaces.msg import DrivePlan, LaneEstimate
from pbb_common.ros import QOS, spin_node, valid_state, stamp_key
from pbb_decision.lane_planner import LanePlanner
from pbb_common.wire import decode_lanes, encode_plan


class Decision(Node):
    def __init__(self):
        super().__init__('pbb_decision')
        weather = self.declare_parameter('weather', 'ClearNoon', ParameterDescriptor(read_only=True)).value
        self.daytime = not weather.endswith('Night')
        self.algorithm = LanePlanner(daytime=self.daytime)
        self.declare_parameter('desired_speed_kmh', 60.0)
        self.publisher = self.create_publisher(DrivePlan, '/pbb/decision/plan', QOS)
        self.create_subscription(LaneEstimate, '/pbb/perception/lanes', self.receive, QOS)
        self.last = None
        self.last_stamp = None

    def receive(self, msg):
        key = (msg.state.session_id, msg.state.frame)
        if self.last is not None and key[0] == self.last[0] and key[1] <= self.last[1]:
            return
        stamp = stamp_key(msg.state.header)
        if self.last is not None and key[0] != self.last[0]:
            self.algorithm = LanePlanner(daytime=self.daytime)
            self.last_stamp = None
        dt = msg.state.dt if self.last_stamp is None else (stamp - self.last_stamp) / 1e9
        self.last = key
        self.last_stamp = stamp
        if not valid_state(msg.state) or not 0 < dt <= 1:
            self.algorithm = LanePlanner(daytime=self.daytime)
            self.publisher.publish(DrivePlan(state=msg.state, status='INVALID STATE - STOP', stop_brake=1.0))
            return
        try:
            observation = decode_lanes(msg)
            speed = float(self.get_parameter('desired_speed_kmh').value)
            plan = self.algorithm.plan(observation, speed, dt=dt,
                vehicle_pose=(msg.state.pose_x_m, msg.state.pose_y_m, msg.state.yaw_rad))
            self.publisher.publish(encode_plan(plan, msg.state))
        except ValueError as error:
            self.algorithm = LanePlanner(daytime=self.daytime)
            self.get_logger().error(str(error))
            self.publisher.publish(DrivePlan(state=msg.state, status='INVALID LANES - STOP', stop_brake=1.0))


def main():
    spin_node(Decision)
