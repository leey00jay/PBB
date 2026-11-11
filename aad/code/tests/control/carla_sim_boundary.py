# Code based on Carla examples, which are authored by 
# Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).

# How to run: 
# cd into the parent directory of the 'code' directory and run
# python -m code.tests.control.carla_sim


import carla
import random
from pathlib import Path
import numpy as np
import pygame
from ...util.carla_util import carla_vec_to_np_array, carla_img_to_array, CarlaSyncMode, find_weather_presets, draw_image_np, should_quit
from ...util.geometry_util import dist_point_linestring
import argparse
import cv2
import copy
from ...solutions.perception.road_boundary_recovery import BoundaryTracker, draw_result



main_image_shape = (1200, 600)


def get_trajectory_from_lane_detector(ld, image):
    # get lane boundaries using the lane detector
    img = carla_img_to_array(image)
    poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(img)
    # trajectory to follow is the mean of left and right lane boundary
    # note that we multiply with -0.5 instead of 0.5 in the formula for y below
    # according to our lane detector x is forward and y is left, but
    # according to Carla x is forward and y is right.
    x = np.arange(-2,60,1.0)
    y = -0.5*(poly_left(x)+poly_right(x))
    # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
    # hence correct x coordinates
    x += 0.5
    traj = np.stack((x,y)).T
    return traj, ld_detection_overlay(img, left_mask, right_mask)

def ld_detection_overlay(image, left_mask, right_mask):
    res = copy.copy(image)
    res[left_mask > 0.5, :] = [0,0,255]
    res[right_mask > 0.5, :] = [255,0,0]
    return res


def get_trajectory_from_map(m, vehicle):
    # get 80 waypoints each 1m apart. If multiple successors choose the one with lower waypoint.id
    wp = m.get_waypoint(vehicle.get_transform().location)
    wps = [wp]
    for _ in range(20):
        next_wps = wp.next(1.0)
        if len(next_wps) > 0:
            wp = sorted(next_wps, key=lambda x: x.id)[0]
        wps.append(wp)

    # transform waypoints to vehicle ref frame
    traj = np.array(
        [np.array([*carla_vec_to_np_array(x.transform.location), 1.]) for x in wps]
    ).T
    trafo_matrix_world_to_vehicle = np.array(vehicle.get_transform().get_inverse_matrix())

    traj = trafo_matrix_world_to_vehicle @ traj
    traj = traj.T
    traj = traj[:,:2]
    return traj

def send_control(vehicle, throttle, steer, brake,
                 hand_brake=False, reverse=False):
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)
    control = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse)
    vehicle.apply_control(control)



def main(use_lane_detector=False, ex=False, save_video=False, half_image=False, bag_path=None):
    if bag_path is None:
        return _run_simulation(use_lane_detector, ex, save_video, half_image)
    if not use_lane_detector:
        raise ValueError('--bag requires --ld so the front camera exists.')
    from ...solutions.runtime.rosbag_recorder import ScreenBagRecorder
    if bag_path == 'auto':
        from datetime import datetime
        bag_path = Path(__file__).resolve().parents[4] / 'bags' / datetime.now().strftime('town10_%Y%m%d_%H%M%S_%f')
    with ScreenBagRecorder(bag_path) as recorder:
        return _run_simulation(use_lane_detector, ex, save_video, half_image, recorder)


def _run_simulation(use_lane_detector=False, ex=False, save_video=False, half_image=False, bag_recorder=None):
    # Imports
    if use_lane_detector and not ex:
        from ...solutions.perception.lane_detector import LaneDetector
        from ...solutions.perception.camera_geometry import CameraGeometry
    elif use_lane_detector and ex:
        from ...exercises.lane_detection.lane_detector import LaneDetector
        from ...exercises.lane_detection.camera_geometry import CameraGeometry    
    if ex:
        from ...exercises.control.pure_pursuit import PurePursuitPlusPID
    else:
        from ...solutions.control.pure_pursuit import PurePursuitPlusPID

    if save_video:
        import atexit
        import imageio
        #import time
        images = []
        from tqdm import tqdm
        video_writer = imageio.get_writer('my_video.mp4', format='FFMPEG', mode='I', fps=30)
        
        def write_images_to_video(images, video_writer):
            print("Writing images to video file...")
            for img in tqdm(images): 
                video_writer.append_data(img)
            video_writer.close()
        atexit.register(lambda: write_images_to_video(images, video_writer))

    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        main_image_shape,
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    #client.load_world('Town06')
    client.load_world('Town10HD')
    world = client.get_world()

    weather_preset, _ = find_weather_presets()[0]
    world.set_weather(carla.WeatherParameters.ClearNoon)

    controller = PurePursuitPlusPID()
    # 새 검출기는 스트림마다 한 번만 생성합니다. 기존 주행 제어와 별개입니다.
    boundary_tracker = BoundaryTracker()
    lane_follower = None
    last_lane_state = None
    desired_speed_kmh = 80

    try:
        m = world.get_map()

        blueprint_library = world.get_blueprint_library()

        veh_bp = random.choice(blueprint_library.filter('vehicle.audi.tt'))
        veh_bp.set_attribute('color','64,81,181')
        vehicle = world.spawn_actor(
            veh_bp,
            m.get_spawn_points()[21])
        actor_list.append(vehicle)


        # visualization cam (no functionality)
        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-10)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)
        sensors = [camera_rgb]

        
        if use_lane_detector:
            if half_image:
            	cg = CameraGeometry(image_width=512, image_height=256)
            else:
            	cg = CameraGeometry()
           
            if not ex:
                ld = LaneDetector(model_path=Path("code/solutions/lane_detection/fastai_model.pth").absolute(), cam_geom=cg)
            else:
                 # TODO: Change this line so that it works with your lane detector implementation
                 ld = LaneDetector()
            #windshield cam
            cam_windshield_transform = carla.Transform(carla.Location(x=0.5, z=cg.height), carla.Rotation(pitch=cg.pitch_deg))
            bp = blueprint_library.find('sensor.camera.rgb')
            fov = cg.field_of_view_deg
            bp.set_attribute('image_size_x', str(cg.image_width))
            bp.set_attribute('image_size_y', str(cg.image_height))
            bp.set_attribute('fov', str(fov))
            camera_windshield = world.spawn_actor(
                bp,
                cam_windshield_transform,
                attach_to=vehicle)
            actor_list.append(camera_windshield)
            sensors.append(camera_windshield)
            if bag_recorder is not None:
                bag_recorder.info['simulation'] = {
                    'map': m.name,
                    'carla_version': client.get_server_version(),
                    'vehicle': vehicle.type_id,
                    'spawn_index': 21,
                    'fixed_delta_seconds': 1.0 / 30,
                    'desired_speed_kmh': desired_speed_kmh,
                    'front_camera': {
                        'width': cg.image_width, 'height': cg.image_height,
                        'horizontal_fov_deg': fov,
                        'location_vehicle_m': [0.5, 0.0, cg.height],
                        'pitch_deg': cg.pitch_deg,
                        'intrinsic_matrix': cg.intrinsic_matrix.tolist(),
                    },
                }
            if not ex:
                from ...solutions.runtime.lane_following import LaneFollower
                lane_follower = LaneFollower(ld)
                wheels = vehicle.get_physics_control().wheels
                lane_follower.max_steer_rad = np.deg2rad(wheels[0].max_steer_angle)


        frame = 0
        max_error = 0
        FPS = 30
        # Create a synchronous mode context.
        with CarlaSyncMode(world, *sensors, fps=FPS) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick(FPS)
                
                # Advance the simulation and wait for the data. 
                tick_response = sync_mode.tick(timeout=10.0)

                speed = np.linalg.norm(carla_vec_to_np_array(vehicle.get_velocity()))
                if use_lane_detector:
                    snapshot, image_rgb, image_windshield = tick_response
                    # 기존 함수는 RGB를 반환하고, 새 검출기는 BGR을 입력받습니다.
                    boundary_bgr = cv2.cvtColor(
                        carla_img_to_array(image_windshield), cv2.COLOR_RGB2BGR)
                    boundary_result = boundary_tracker.update(
                        boundary_bgr, image_windshield.timestamp)
                    boundary_preview = draw_result(boundary_bgr, boundary_result)
                    if lane_follower is not None:
                        result = lane_follower.update(
                            carla_img_to_array(image_windshield), speed,
                            desired_speed_kmh, 1.0 / FPS)
                        traj, viz = result['trajectory'], result['overlay']
                    elif frame % 2 == 0:
                        traj, viz = get_trajectory_from_lane_detector(ld, image_windshield)
                else:
                    snapshot, image_rgb = tick_response
                    traj = get_trajectory_from_map(m, vehicle)

                # get velocity and angular velocity
                vel = carla_vec_to_np_array(vehicle.get_velocity())
                forward = carla_vec_to_np_array(vehicle.get_transform().get_forward_vector())
                right = carla_vec_to_np_array(vehicle.get_transform().get_right_vector())
                up = carla_vec_to_np_array(vehicle.get_transform().get_up_vector())
                vx = vel.dot(forward)
                vy = vel.dot(right)
                vz = vel.dot(up)
                ang_vel = carla_vec_to_np_array(vehicle.get_angular_velocity())
                w = ang_vel.dot(up)
                if lane_follower is not None:
                    send_control(vehicle, result['throttle'], result['steer'], result['brake'])
                    lane_state = result['diagnostics']['status']
                    target_kmh = result['diagnostics']['target_kmh']
                    if lane_state != last_lane_state:
                        print(f"lane: {lane_state} | target: {target_kmh:.1f} km/h")
                        last_lane_state = lane_state
                else:
                    throttle, steer = controller.get_control(
                        traj, speed, desired_speed=desired_speed_kmh / 3.6, dt=1.0/FPS)
                    send_control(vehicle, throttle, steer, 0)

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Camera mode: this is error to the ESTIMATED path, not map truth.
                if traj is None:
                    error_text = "--"
                else:
                    cross_track_error = dist_point_linestring(np.array([0,0]), traj) * 100
                    max_error = max(max_error, cross_track_error)
                    error_text = f"{cross_track_error:.2f}"

                # Draw the display.
                image_rgb = copy.copy(carla_img_to_array(image_rgb))
                if use_lane_detector:
                    viz = cv2.resize(viz, (400,200), interpolation = cv2.INTER_AREA)
                    image_rgb[0:viz.shape[0], 0:viz.shape[1],:] = viz
					# white background for text
                    image_rgb[10:190,-390:-10, : ] = [255,255,255]
                draw_image_np(display, image_rgb)

                # draw txt
                dy = 20
                texts = ["FPS (real):          {}".format(int(clock.get_fps())),
                         "FPS (simulated):     {}".format(fps),
                         "speed (km/h):        {:.2f}".format(speed * 3.6),
                         "lateral error (cm):  {}".format(error_text),
                         "max lat. error (cm): {:.2f}".format(max_error)
                        ]
                
                if lane_follower is not None:
                    texts += [f"target (km/h):       {target_kmh:.2f}",
                              f"lane: {lane_state}",
                              f"throttle / brake:   {result['throttle']:.2f} / {result['brake']:.2f}"]
                for it,t in enumerate(texts):
                    display.blit(
                        font.render(t, True, (0,0,0)), (image_rgb.shape[1]-380, 20+dy*it))

                # 오른쪽 패널은 경계 검출 확인용입니다. 차량 조향에는 사용하지 않습니다.
                pygame.draw.rect(display, (25, 25, 25), (800, 0, 400, 600))
                display.blit(font.render("ROAD BOUNDARY / PREVIEW ONLY", True,
                                         (255, 255, 255)), (810, 12))
                if use_lane_detector:
                    preview_rgb = cv2.cvtColor(boundary_preview, cv2.COLOR_BGR2RGB)
                    preview_rgb = cv2.resize(preview_rgb, (400, 200),
                                             interpolation=cv2.INTER_AREA)
                    preview_surface = pygame.surfarray.make_surface(preview_rgb.swapaxes(0, 1))
                    display.blit(preview_surface, (800, 40))
                    error_px = boundary_result['center_error_px']
                    error_label = "--" if error_px is None else f"{error_px:+.2f}"
                    age = boundary_result['estimate_age_s']
                    age_label = "--" if age is None else f"{age:.3f}"
                    labels = [
                        f"Status: {boundary_result['tracking_status']}",
                        f"Fresh detection: {boundary_result['detection_fresh']}",
                        f"Estimate age (s): {age_label}",
                        f"Visual center error (px): {error_label}",
                        f"Reason: {boundary_result['reason']}",
                        "Used for steering: NO"
                    ]
                else:
                    labels = ["Run with --ld to enable the preview."]
                for row, label in enumerate(labels):
                    display.blit(font.render(label, True, (255, 255, 255)),
                                 (810, 265 + 25 * row))

                if bag_recorder is not None:
                    display.blit(font.render(
                        f"REC ROSBAG: {bag_recorder.frames + 1} frames", True,
                        (255, 90, 90)), (810, 560))

                pygame.display.flip()
                if bag_recorder is not None:
                    bag_recorder.write(
                        pygame.surfarray.array3d(display).swapaxes(0, 1),
                        carla_img_to_array(image_windshield),
                        snapshot.timestamp.elapsed_seconds)

                frame += 1
                if save_video and frame > 0:
                    print("frame=",frame)
                    imgdata = pygame.surfarray.array3d(pygame.display.get_surface())
                    imgdata = imgdata.swapaxes(0,1)
                    images.append(imgdata)
                

    finally:

        print('destroying actors.')
        for actor in reversed(actor_list):
            if actor.type_id.startswith('sensor.') and actor.is_listening:
                actor.stop()
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Town10 camera lane following with a separate road-boundary preview. The boundary preview does not control the vehicle.')
    parser.add_argument("--ld", action="store_true", help="Use reference trajectory from your LaneDetector class")
    parser.add_argument("--ex", action="store_true", help="Run student code")
    parser.add_argument("--vid", action="store_true", help="Save video after simulation")
    parser.add_argument("--bag", nargs="?", const="auto", metavar="FOLDER", help="Save screen and raw front camera to a new ROS 2 bag folder. Requires --ld. With no folder, use PBB/bags/town10_<timestamp>.")
    parser.add_argument("--half_image", action="store_true", help="Pass images with (width, height) = (512,256) to lane detector instead of the default (1024,512). This will speed up the simulation, but might hurt accuracy.")
    args = parser.parse_args()
    if args.bag is not None and not args.ld:
        parser.error('--bag requires --ld to record the front camera.')

    try:
        main(use_lane_detector = args.ld, ex = args.ex, save_video=args.vid, half_image=args.half_image, bag_path=args.bag)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
