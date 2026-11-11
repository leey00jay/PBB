"""LaneEstimate topic -> DrivePlan topic, using the team's existing planner."""
from rclpy.node import Node
from pbb_interfaces.msg import DrivePlan, LaneEstimate
from pbb_common.ros import QOS, spin_node, valid_state
from pbb_common.wire import decode_lanes, encode_plan


class Decision(Node):
    def __init__(self):
        super().__init__('pbb_decision')
        from pbb_decision.lane_planner import LanePlanner
        self.algorithm = LanePlanner()
        self.declare_parameter('desired_speed_kmh', 80.0)
        self.publisher = self.create_publisher(DrivePlan, '/pbb/decision/plan', QOS)
        self.create_subscription(LaneEstimate, '/pbb/perception/lanes', self.receive, QOS)
        self.last = None

    def receive(self, msg):
        key = (msg.state.session_id, msg.state.frame)
        if self.last is not None and key[0] == self.last[0] and key[1] <= self.last[1]:
            return
        self.last = key
        if not valid_state(msg.state):
            self.publisher.publish(DrivePlan(state=msg.state, status='INVALID STATE - STOP', stop_brake=1.0))
            return
        try:
            observation = decode_lanes(msg)
            speed = float(self.get_parameter('desired_speed_kmh').value)
            plan = self.algorithm.plan(observation, speed, dt=msg.state.dt)
            self.publisher.publish(encode_plan(plan, msg.state))
        except ValueError as error:
            self.get_logger().error(str(error))
            self.publisher.publish(DrivePlan(state=msg.state, status='INVALID LANES - STOP', stop_brake=1.0))


def main():
    spin_node(Decision)
