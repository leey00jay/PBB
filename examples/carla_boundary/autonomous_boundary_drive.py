#!/usr/bin/env python3
"""Automatic red/green and lane-centering trials with actual vision steering.

CARLA autopilot is disabled. Uploaded boundary estimates steer on marked road;
an explicitly logged map assist bridges intersections or missing detections.
Traffic lights come from the CARLA API, not an image classifier.
"""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import signal
import sys
import time
import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageTk

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE / '.carla-python'))
sys.path.insert(0, str(BASE / 'road_boundary_recovery'))
import carla
from road_boundary_recovery import BoundaryTracker, draw_result
from boundary_controller import vision_target, pursuit_steer, target_speed, longitudinal, clip

DT = .05
WIDTH, HEIGHT = 1280, 720


def dot(a, b):
    return a.x*b.x + a.y*b.y + a.z*b.z


def length(a):
    return math.sqrt(dot(a, a))


def build_route(start, metres=110):
    route = [start]
    for _ in range(int(metres)):
        options = route[-1].next(1)
        if not options:
            break
        yaw = route[-1].transform.rotation.yaw
        route.append(min(options, key=lambda p: abs((p.transform.rotation.yaw-yaw+180) % 360-180)))
    return route


def summarize(rows, events, outcome):
    moving = [r for r in rows if r['speed_kmh'] > 1]
    road = [r for r in moving if not r['junction']]
    errors = [abs(r['lane_error_m']) for r in road]
    modes = Counter(r['steering_source'] for r in moving)
    vision = [r for r in road if r['steering_source'].startswith('VISION')]
    red = [r for r in rows if r['signal'] == 'Red']
    red_stop = next((e for e in events if e['type'] == 'red_stop_confirmed'), None)
    green_start = next((e for e in events if e['type'] == 'green_departure'), None)
    return {'outcome': outcome, 'frames': len(rows), 'moving_frames': len(moving),
            'red_stop_confirmed': red_stop is not None,
            'green_departure_confirmed': green_start is not None,
            'green_departure_delay_s': green_start['delay_s'] if green_start else None,
            'red_front_crossed_stop_reference': any(r['front_gap_m'] < -.1 for r in red),
            'stop_front_gap_m': red_stop['front_gap_m'] if red_stop else None,
            'moving_steering_sources': dict(modes),
            'vision_steering_percent_moving': round(100*sum(v for k,v in modes.items() if k.startswith('VISION'))/max(len(moving),1),2),
            'nonjunction_lane_abs_mean_m': round(float(np.mean(errors)),4) if errors else None,
            'nonjunction_lane_abs_p95_m': round(float(np.percentile(errors,95)),4) if errors else None,
            'nonjunction_lane_abs_max_m': round(max(errors),4) if errors else None,
            'vision_nonjunction_abs_mean_m': round(float(np.mean([abs(r['lane_error_m']) for r in vision])),4) if vision else None,
            'nonjunction_vehicle_outside_lane_frames': sum(r['lane_clearance_m'] < 0 for r in road),
            'nonjunction_within_0_5m_percent': round(100*sum(e<=.5 for e in errors)/max(len(errors),1),2),
            'manual_driving_inputs': 0, 'events': events}


def run(args):
    cv2.setNumThreads(2)
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10)
    world = client.get_world()
    carla_map = world.get_map()
    if not carla_map.name.endswith('/Town02_Opt'):
        raise RuntimeError('This configured repeatable scenario requires Town02_Opt')
    old_settings = world.get_settings()
    if old_settings.synchronous_mode:
        raise RuntimeError('Another client owns synchronous mode')
    vehicles = [v for v in world.get_actors().filter('vehicle.*') if v.attributes.get('role_name') == 'carla-demo']
    if len(vehicles) != 1:
        raise RuntimeError('Expected exactly one carla-demo vehicle')
    vehicle = vehicles[0]
    if list(world.get_actors().filter('sensor.camera.rgb')):
        raise RuntimeError('Close the earlier camera controller before this test')
    lights = list(world.get_actors().filter('traffic.traffic_light'))
    saved_lights = [(l, l.get_state(), l.is_frozen()) for l in lights]
    # Identify the northbound signal near the known Town02 starting lane by geometry.
    candidates = [(l, p) for l in lights for p in l.get_stop_waypoints()
                  if abs(p.transform.location.x + 7.46) < 1 and abs(p.transform.location.y - 170.69) < 2]
    if len(candidates) != 1:
        raise RuntimeError('Could not uniquely identify the test approach signal')
    light, stop_wp = candidates[0]
    stop_transform = stop_wp.transform
    start_wp = stop_wp.previous(35)[0]
    route = build_route(start_wp)
    route_xy = np.array([[p.transform.location.x, p.transform.location.y] for p in route])
    physics = vehicle.get_physics_control()
    wheels = physics.wheels
    front_axle = (wheels[0].position + wheels[1].position) * .5
    rear_axle = (wheels[2].position + wheels[3].position) * .5
    wheelbase = length(front_axle-rear_axle)/100
    max_steer = max(w.max_steer_angle for w in wheels)
    output = BASE / 'autonomous-tests' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True)
    metadata = {'map': carla_map.name, 'carla_version': client.get_server_version(),
                'controller': 'boundary_controller.py; CARLA autopilot OFF',
                'vision_detector_sha256': hashlib.sha256((BASE/'road_boundary_recovery/road_boundary_recovery.py').read_bytes()).hexdigest(),
                'traffic_signal_source': 'CARLA API; controlled red-to-green scenario',
                'map_assist_enabled': not args.vision_only,
                'map_assist_policy': 'junction or invalid/uncalibrated visual corridor',
                'evaluation_reference': 'CARLA map route center; moving speed > 1 km/h; junction excluded from lane metrics',
                'lane_center_test_threshold_m': .5, 'cycles': args.cycles,
                'camera': {'width': WIDTH, 'height': HEIGHT, 'fov': 90, 'mount_x': 1.5, 'mount_z': 1.6},
                'wheelbase_m': wheelbase, 'max_steer_deg': max_steer,
                'fixed_delta_seconds': DT, 'cruise_kmh': args.speed,
                'signal_id': light.id, 'manual_driving_inputs': 0}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    root = tk.Tk()
    root.title('CARLA 자동 검증 — 검출값 조향 · 신호 정지/출발 · 차선 중앙 유지')
    root.configure(bg='#111827')
    canvas = tk.Label(root, bg='black'); canvas.pack()
    status = tk.StringVar(value='자동 시험 준비 중 — 수동 운전 불필요')
    tk.Label(root,textvariable=status,bg='#111827',fg='white',padx=12,pady=9,anchor='w').pack(fill='x')
    help_text = tk.StringVar(value='VISION: 첨부 검출값으로 조향 | MAP ASSIST: 지도 보조 | Space: 시험 중단 | Esc: 닫기')
    tk.Label(root,textvariable=help_text,bg='#1f2937',fg='white',padx=12,pady=8).pack(fill='x')
    stop_requested = False
    restart_requested = False
    def stop(*_):
        nonlocal stop_requested
        stop_requested = True
    def restart():
        nonlocal restart_requested, stop_requested
        restart_requested = True; stop_requested = True
    root.protocol('WM_DELETE_WINDOW',stop)
    root.bind('<Escape>',stop);root.bind('<space>',stop)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    images = queue.Queue()
    sensors = []
    collision_events = []
    all_rows, trials = [], []
    writer = log = None
    settings_changed = False
    cleanup = {}
    error = None
    video_index = 0
    started_wall = time.monotonic()

    def park():
        vehicle.apply_control(carla.VehicleControl(brake=1,hand_brake=True))

    def tick(wait_image=True):
        root.update_idletasks(); root.update()
        if stop_requested:
            raise InterruptedError('Stopped by user')
        number = world.tick(10)
        if not sensors or not wait_image:
            return None
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            try:
                image = images.get(timeout=max(.01,deadline-time.monotonic()))
            except queue.Empty as exc:
                raise RuntimeError(f'Camera timeout waiting for simulation frame {number}') from exc
            if image.frame == number:
                return image
            if image.frame > number:
                raise RuntimeError('Unexpected camera frame ordering')
        raise RuntimeError('Camera timeout')

    try:
        vehicle.set_autopilot(False,8000)
        park()
        world.wait_for_tick(10)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        world.apply_settings(settings)
        settings_changed = True
        world.tick(10)
        light.freeze(True)  # CARLA freezes all lights; all states are saved/restored.
        for member in light.get_group_traffic_lights():
            member.set_state(carla.TrafficLightState.Red)
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        for key,value in [('image_size_x',str(WIDTH)),('image_size_y',str(HEIGHT)),('fov','90'),('sensor_tick','0.0')]:
            bp.set_attribute(key,value)
        camera = world.spawn_actor(bp,carla.Transform(carla.Location(x=1.5,z=1.6)),attach_to=vehicle)
        camera.listen(images.put)
        sensors.append(camera)
        collision = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=vehicle)
        collision.listen(lambda e: collision_events.append({'frame':e.frame,'other_actor_id':e.other_actor.id,'impulse':length(e.normal_impulse)}))
        sensors.append(collision)
        writer = cv2.VideoWriter(str(output/'review.mp4'),cv2.VideoWriter_fourcc(*'avc1'),1/DT,(WIDTH,HEIGHT))
        if not writer.isOpened():raise RuntimeError('H264 encoder unavailable')
        log = (output/'frames.jsonl').open('w',buffering=1)
        print(f'OUTPUT={output}',flush=True)
        print(json.dumps(metadata,indent=2),flush=True)
        for trial in range(args.cycles):
            if stop_requested:break
            offset = [0.0,-.3,.3][trial%3]
            park();light.set_state(carla.TrafficLightState.Red)
            transform = carla.Transform(start_wp.transform.location + start_wp.transform.get_right_vector()*offset + carla.Location(z=.3),start_wp.transform.rotation)
            vehicle.set_simulate_physics(False);vehicle.set_transform(transform)
            tick(wait_image=False)
            vehicle.set_simulate_physics(True)
            vehicle.set_target_velocity(carla.Vector3D());vehicle.set_target_angular_velocity(carla.Vector3D())
            park()
            for _ in range(12):tick(wait_image=False)
            tracker=BoundaryTracker()
            trial_rows,events=[],[]
            phase='RED_APPROACH';still_since=None;green_time=None;green_position=None
            departed=False;red_confirmed=False;last_steer=0.0;trial_start=None
            red_stop_latched=False
            outcome='timeout';first_collision_count=len(collision_events)
            last_mode=None
            for step in range(int(args.timeout/DT)):
                image=tick()
                snapshot=world.get_snapshot()
                if snapshot.frame != image.frame:raise RuntimeError('Sensor/state frames are not aligned')
                if trial_start is None:trial_start=image.timestamp
                elapsed=image.timestamp-trial_start
                pose=vehicle.get_transform();location=pose.location;speed=length(vehicle.get_velocity())
                index=int(np.argmin(np.sum((route_xy-np.array([location.x,location.y]))**2,axis=1)))
                reference=route[index];ref_pose=reference.transform
                lane_error=dot(location-ref_pose.location,ref_pose.get_right_vector())
                heading=math.radians((pose.rotation.yaw-ref_pose.rotation.yaw+180)%360-180)
                bbox=vehicle.bounding_box
                footprint=abs(math.cos(heading))*bbox.extent.y+abs(math.sin(heading))*bbox.extent.x
                clearance=reference.lane_width/2-abs(lane_error)-footprint
                vertices=bbox.get_world_vertices(pose)
                stop_forward=stop_transform.get_forward_vector()
                front_gap=min(dot(stop_transform.location-v,stop_forward) for v in vertices)
                signal_state=str(light.get_state())
                bgr=np.frombuffer(image.raw_data,dtype=np.uint8).reshape(HEIGHT,WIDTH,4)[:,:,:3].copy()
                result=tracker.update(bgr,image.timestamp)
                frame_event=None
                target=vision_target(result,WIDTH,HEIGHT)
                if target is not None and not reference.is_junction:
                    source='VISION' if result['detection_fresh'] else 'VISION_HELD'
                elif not args.vision_only:
                    source='MAP_ASSIST_JUNCTION' if reference.is_junction else 'MAP_ASSIST_MISSING'
                    goal=route[min(index+8,len(route)-1)].transform.location-location
                    target=(dot(goal,pose.get_forward_vector()),dot(goal,pose.get_right_vector()))
                else:
                    source='NO_PATH_STOP';target=None
                desired=target_speed(args.speed/3.6,signal_state,front_gap)
                # Once nearly stopped near the red-light reference, remain
                # stopped until green instead of repeatedly creeping forward.
                if signal_state=='Red' and elapsed>3 and speed<.2 and .1<front_gap<3.5:
                    red_stop_latched=True
                elif signal_state=='Green':
                    red_stop_latched=False
                if red_stop_latched:desired=0
                if target is None:desired=0
                steer=pursuit_steer(target,wheelbase,max_steer) if target else last_steer
                steer=clip(steer,last_steer-.04,last_steer+.04)
                last_steer=steer
                throttle,brake=longitudinal(speed,desired)
                # Evaluation is measured before this newly computed command is applied.
                row={'trial':trial+1,'initial_offset_m':offset,'frame_id':image.frame,'video_frame':video_index,
                     'timestamp_s':image.timestamp,'trial_elapsed_s':elapsed,'phase':phase,'signal':signal_state,
                     'speed_kmh':speed*3.6,'target_speed_kmh':desired*3.6,'front_gap_m':front_gap,
                     'lane_error_m':lane_error,'lane_width_m':reference.lane_width,'lane_clearance_m':clearance,
                     'heading_error_deg':math.degrees(heading),'junction':reference.is_junction,
                     'steering_source':source,'steer_command':steer,'throttle_command':throttle,'brake_command':brake,
                     'applied_steer_previous':vehicle.get_control().steer,
                     'detector':result,'location':{'x':location.x,'y':location.y,'z':location.z}}
                if signal_state=='Red':
                    if speed<.10 and .1<front_gap<5 and elapsed>3:
                        if still_since is None:still_since=image.timestamp
                        phase='RED_HOLD'
                        if image.timestamp-still_since>=2 and not red_confirmed:
                            red_confirmed=True
                            events.append({'type':'red_stop_confirmed','time_s':elapsed,'front_gap_m':round(front_gap,3),'hold_s':round(image.timestamp-still_since,3),'frame_id':image.frame,'video_frame':video_index})
                            frame_event='red-stop'
                            light.set_state(carla.TrafficLightState.Green)
                            green_time=image.timestamp+DT;green_position=location
                            phase='GREEN_DEPART'
                            print(f'Trial {trial+1}: red stop confirmed, gap={front_gap:.2f} m; switching green',flush=True)
                    else:
                        still_since=None
                        phase='RED_APPROACH'
                    if front_gap<-.1:
                        outcome='red_stop_reference_crossed';break
                elif green_time is not None and not departed and speed>.5 and location.distance(green_position)>.3:
                    departed=True
                    events.append({'type':'green_departure','time_s':elapsed,'delay_s':round(image.timestamp-green_time,3),'frame_id':image.frame,'video_frame':video_index})
                    frame_event='green-departure'
                    print(f'Trial {trial+1}: green departure after {image.timestamp-green_time:.2f} s',flush=True)
                vehicle.apply_control(carla.VehicleControl(throttle=throttle,brake=brake,steer=steer))
                trial_rows.append(row);all_rows.append(row);log.write(json.dumps(row)+'\n')
                annotated=draw_result(bgr,result)
                cv2.rectangle(annotated,(0,0),(WIDTH,154),(18,24,32),-1)
                color=(0,220,0) if signal_state=='Green' else (40,40,255)
                cv2.putText(annotated,f'Trial {trial+1}/{args.cycles} | {signal_state} | {row["phase"]} | {elapsed:.1f}s',(14,30),cv2.FONT_HERSHEY_SIMPLEX,.8,color,2)
                cv2.putText(annotated,f'{source} | speed {speed*3.6:.1f} km/h | steer {steer:+.3f}',(14,62),cv2.FONT_HERSHEY_SIMPLEX,.72,(255,255,255),2)
                cv2.putText(annotated,f'Map-measured lane error: {lane_error:+.3f} m | front stop gap: {front_gap:.2f} m',(14,94),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
                cv2.putText(annotated,'Actual boundary steering | Signal: simulator API | Map assist labeled | Autopilot OFF',(14,126),cv2.FONT_HERSHEY_SIMPLEX,.57,(180,220,255),1)
                writer.write(annotated);video_index+=1
                if frame_event:
                    cv2.imwrite(str(output/f'trial-{trial+1}-{frame_event}.jpg'),annotated)
                if step%2==0:
                    preview=ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(cv2.resize(annotated,(960,540)),cv2.COLOR_BGR2RGB)))
                    canvas.configure(image=preview);canvas.image=preview
                    status.set(f'{trial+1}/{args.cycles} 자동 시험 · {signal_state} · {source} · 속도 {speed*3.6:.1f}km/h · 중심 오차 {lane_error:+.2f}m')
                if step%100==0:
                    cv2.imwrite(str(output/'latest.jpg'),annotated)
                    print(f'Trial {trial+1} t={elapsed:.1f} {signal_state} {source} speed={speed*3.6:.1f} error={lane_error:+.3f} gap={front_gap:.1f}',flush=True)
                if source!=last_mode:
                    events.append({'type':'steering_mode','time_s':round(elapsed,3),'source':source})
                    if source in ('VISION','MAP_ASSIST_JUNCTION'):
                        cv2.imwrite(str(output/f'trial-{trial+1}-{source}.jpg'),annotated)
                    last_mode=source
                if reference.is_junction:
                    phase='JUNCTION'
                elif green_time is not None and departed:
                    phase='GREEN_CRUISE'
                if front_gap < -45:
                    outcome='route_complete';break
                if len(collision_events)>first_collision_count:
                    outcome='collision';break
                if abs(lane_error)>1.5:
                    outcome='excessive_lateral_error';break
                world.get_spectator().set_transform(carla.Transform(location-pose.get_forward_vector()*7+carla.Location(z=3.5),carla.Rotation(pitch=-15,yaw=pose.rotation.yaw)))
            park()
            trial_result={'trial':trial+1,'initial_offset_m':offset,**summarize(trial_rows,events,outcome),
                          'collisions':collision_events[first_collision_count:]}
            trials.append(trial_result)
            (output/f'trial-{trial+1}.json').write_text(json.dumps(trial_result,indent=2))
            print('TRIAL_RESULT='+json.dumps(trial_result),flush=True)
            if outcome!='route_complete':break
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
        print('ERROR='+error,flush=True)
    finally:
        try:park()
        except Exception as exc:cleanup['park_error']=str(exc)
        for sensor in sensors:
            try:sensor.stop();sensor.destroy()
            except Exception as exc:cleanup[f'sensor_{sensor.id}']=str(exc)
        if writer:writer.release()
        if log:log.close()
        for saved_light,state,_ in saved_lights:
            try:saved_light.set_state(state)
            except Exception as exc:cleanup['light_error']=str(exc)
        try:
            if saved_lights:light.freeze(saved_lights[0][2])
        except Exception as exc:cleanup['freeze_error']=str(exc)
        if settings_changed:
            try:world.apply_settings(old_settings)
            except Exception as exc:cleanup['world_settings_error']=str(exc)
        overall=summarize(all_rows,[], 'see_trials')
        for field in ('red_stop_confirmed','green_departure_confirmed','green_departure_delay_s','stop_front_gap_m','events'):
            overall.pop(field)
        overall['red_stop_trials']=sum(t['red_stop_confirmed'] for t in trials)
        overall['green_departure_trials']=sum(t['green_departure_confirmed'] for t in trials)
        summary={**metadata,'trials_completed':len(trials),'trials':trials,'overall':overall,
                 'total_video_frames':video_index,'wall_seconds':round(time.monotonic()-started_wall,2),
                 'error':error,'cleanup_errors':cleanup,'collisions':collision_events}
        summary['all_trials_passed']=len(trials)==args.cycles and not error and all(
            t['outcome']=='route_complete' and t['red_stop_confirmed'] and t['green_departure_confirmed']
            and not t['red_front_crossed_stop_reference'] and not t['collisions']
            and t['nonjunction_vehicle_outside_lane_frames']==0
            and t['nonjunction_lane_abs_max_m']<=.5 for t in trials)
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        print('FINAL_SUMMARY='+json.dumps(summary),flush=True)
        print(f'OUTPUT={output}',flush=True)
    if args.keep_open and not stop_requested:
        status.set(f'자동 시험 완료: {len(trials)}/{args.cycles} · '+('통과' if summary['all_trials_passed'] else '결과 확인 필요')+' · 차량 정차')
        help_text.set('운전 조작 없이 다시 검증하려면 아래 버튼을 누르세요. Esc: 닫기')
        tk.Button(root,text='자동 시험 다시 실행',command=restart,padx=20,pady=8).pack(pady=8)
        while not stop_requested:
            root.update_idletasks();root.update();time.sleep(.05)
    root.destroy()
    return restart_requested, summary['all_trials_passed']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles',type=int,default=3)
    parser.add_argument('--speed',type=float,default=12)
    parser.add_argument('--timeout',type=float,default=60)
    parser.add_argument('--vision-only',action='store_true',help='Stop if vision fails; disable map assistance')
    parser.add_argument('--keep-open',action='store_true')
    args=parser.parse_args()
    if not 1<=args.cycles<=10 or not 0<args.speed<=20 or args.timeout<=0:
        parser.error('cycles 1..10, speed (0,20], timeout > 0 required')
    while True:
        restart, passed = run(args)
        if not restart:
            raise SystemExit(0 if passed else 1)


if __name__=='__main__':main()
