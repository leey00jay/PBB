"""DrivePlan topic -> normalized VehicleControl topic."""
from rclpy.node import Node
from pbb_interfaces.msg import DrivePlan, VehicleControl
from pbb_common.ros import QOS, spin_node, stop_command, valid_state
from pbb_common.wire import decode_plan, encode_control


class Control(Node):
    def __init__(self):
        super().__init__('pbb_control')
        from pbb_control.vehicle_controller import VehicleController
        self.algorithm = VehicleController()
        self.publisher = self.create_publisher(VehicleControl, '/pbb/control/command', QOS)
        self.create_subscription(DrivePlan, '/pbb/decision/plan', self.receive, QOS)
        self.last = None

    def receive(self, msg):
        key = (msg.state.session_id, msg.state.frame)
        if self.last is not None and key[0] == self.last[0] and key[1] <= self.last[1]:
            return
        self.last = key
        try:
            if not valid_state(msg.state):
                raise ValueError('Invalid vehicle state')
            self.algorithm.max_steer_rad = msg.state.max_steer_rad
            command = self.algorithm.update(decode_plan(msg), msg.state.speed_mps, msg.state.dt)
            output = encode_control(command, msg.state)
        except ValueError as error:
            self.get_logger().error(str(error))
            self.algorithm._reset_speed()
            output = stop_command(msg.state, 'INVALID PLAN - STOP')
        self.publisher.publish(output)


def main():
    spin_node(Control)
