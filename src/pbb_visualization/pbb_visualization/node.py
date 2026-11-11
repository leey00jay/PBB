"""Display node: all images and HUD values arrive through ROS subscriptions."""
import cv2
import numpy as np
import pygame
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from pbb_interfaces.msg import FrameState, DrivePlan, VehicleControl
from pbb_common.ros import FrameJoin, IMAGE_QOS, QOS, image_array


class Display(Node):
    def __init__(self):
        super().__init__('pbb_display')
        pygame.init()
        self.screen = pygame.display.set_mode((1200, 828))
        pygame.display.set_caption('PBB | ROS 2 topics | Town10')
        self.font = pygame.font.SysFont('monospace', 15)
        self.join = FrameJoin(('left', 'center', 'right', 'chase', 'state', 'overlay', 'boundary', 'plan', 'applied'))
        self.publisher = self.create_publisher(CompressedImage, '/pbb/screen/image/compressed', IMAGE_QOS)
        self.pending = None
        self.max_error = 0.0
        self.fps = pygame.time.Clock()
        for name in ('left', 'center', 'right', 'chase'):
            self.create_subscription(Image, f'/pbb/camera/{name}/image_raw',
                                     lambda m, n=name: self.receive(n, m.header, m), IMAGE_QOS)
        for name in ('overlay', 'boundary'):
            self.create_subscription(Image, f'/pbb/perception/{name}',
                                     lambda m, n=name: self.receive(n, m.header, m), IMAGE_QOS)
        self.create_subscription(FrameState, '/pbb/vehicle/state', lambda m: self.receive('state', m.header, m), QOS)
        self.create_subscription(DrivePlan, '/pbb/decision/plan', lambda m: self.receive('plan', m.state.header, m), QOS)
        self.create_subscription(VehicleControl, '/pbb/control/applied', lambda m: self.receive('applied', m.state.header, m), QOS)
        self.screen.fill((25, 25, 25))
        self.text('Waiting for camera, perception, decision and control topics...', (30, 35))
        pygame.display.flip()

    def receive(self, name, header, msg):
        row = self.join.put(name, header, msg)
        if row is not None:
            self.pending = row

    def text(self, value, position, color=(255, 255, 255)):
        self.screen.blit(self.font.render(value, True, color), position)

    def draw_image(self, msg, position, size):
        rgb = cv2.resize(image_array(msg), size, interpolation=cv2.INTER_AREA)
        self.screen.blit(pygame.surfarray.make_surface(rgb.swapaxes(0, 1)), position)

    def render(self, row):
        self.fps.tick()
        self.screen.fill((25, 25, 25))
        self.draw_image(row['chase'], (0, 0), (800, 600))
        self.draw_image(row['overlay'], (0, 0), (400, 200))
        pygame.draw.rect(self.screen, (255, 255, 255), (410, 10, 380, 210))
        state, plan, command = row['state'], row['plan'], row['applied']
        error = '--'
        if plan.has_trajectory and len(plan.trajectory) >= 2:
            points = np.array([(p.x, p.y) for p in plan.trajectory])
            a, b = points[:-1], points[1:]
            v = b - a
            t = np.clip(-(a * v).sum(axis=1) / np.maximum((v * v).sum(axis=1), 1e-12), 0, 1)
            distance = float(np.linalg.norm(a + t[:, None] * v, axis=1).min()) * 100
            self.max_error = max(self.max_error, distance)
            error = f'{distance:.2f}'
        labels = [f'FPS (real): {self.fps.get_fps():.1f}', f'FPS (simulated): {1 / state.dt:.0f}',
                  f'speed (km/h): {state.speed_mps * 3.6:.2f}', f'target (km/h): {command.target_kmh:.2f}',
                  f'lateral error (cm): {error}', f'max lat. error (cm): {self.max_error:.2f}',
                  f'throttle / brake: {command.throttle:.2f} / {command.brake:.2f}', command.status]
        for i, label in enumerate(labels):
            self.text(label, (420, 20 + i * 23), (0, 0, 0))
        self.text('ROAD BOUNDARY / PREVIEW ONLY', (810, 12))
        self.draw_image(row['boundary'], (800, 40), (400, 200))
        self.text('Used for steering: NO', (810, 265))
        self.text('ROS 2 TOPIC PIPELINE', (810, 320))
        for i, label in enumerate(['Camera -> Perception', 'Lanes -> Decision', 'Plan -> Control',
                                   'Command -> CARLA', f'Frame: {state.frame}']):
            self.text(label, (810, 355 + 25 * i))
        for i, name in enumerate(('left', 'center', 'right')):
            self.text(f'FRONT {name.upper()}', (10 + 400 * i, 606))
            self.draw_image(row[name], (400 * i, 628), (400, 200))
        pygame.display.flip()
        if self.publisher.get_subscription_count():
            rgb = pygame.surfarray.array3d(self.screen).swapaxes(0, 1)
            ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                self.publisher.publish(CompressedImage(header=state.header, format='rgb8; jpeg compressed bgr8', data=encoded.tobytes()))

    def run(self):
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE):
                    return
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.pending is not None:
                row, self.pending = self.pending, None
                self.render(row)


def main():
    rclpy.init()
    node = None
    try:
        node = Display()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        pygame.quit()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
