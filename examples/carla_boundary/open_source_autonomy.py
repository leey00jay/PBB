#!/usr/bin/env python3
"""CARLA 0.9.16 BehaviorAgent + optional uploaded boundary steering.

Upstream agents are vendored, unchanged, under third_party/carla_agents (MIT).
Signals, route planning, vehicle and pedestrian detection use CARLA world data.
Boundary-based steering is used on suitable straight road segments and logged.
"""
import argparse
from collections import Counter
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import queue
import signal
import subprocess
import sys
import time
import tkinter as tk

BASE=Path(__file__).resolve().parent
sys.path[:0]=[str(BASE/'.autonomy-deps'),str(BASE/'.carla-python'),
              str(BASE/'third_party/carla_agents'),str(BASE/'road_boundary_recovery')]
import carla
import cv2
import numpy as np
from PIL import Image,ImageTk
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.local_planner import RoadOption
from road_boundary_recovery import BoundaryTracker,draw_result
from boundary_controller import vision_target,pursuit_steer,clip

DT=.05
W,H=1280,720


def norm(v):return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
def dot(a,b):return a.x*b.x+a.y*b.y+a.z*b.z


class BoundaryBehaviorAgent(BehaviorAgent):
    """Keep upstream decisions/braking; allow visual steering on straight road."""
    def __init__(self,vehicle,speed,agent_only=False):
        super().__init__(vehicle,behavior='cautious',opt_dict={
            'dt':DT,'base_tlight_threshold':8,'max_brake':.6,'max_throttle':.55,
            'base_min_distance':1.2,'distance_ratio':.30,
            'lateral_control_dict':{'K_P':1.2,'K_I':0,'K_D':.05,'dt':DT},
            'longitudinal_control_dict':{'K_P':.35,'K_I':.03,'K_D':0,'dt':DT}})
        self.cruise_speed=speed
        self._behavior.max_speed=speed
        self._behavior.tailgate_counter=-1
        self.agent_only=agent_only
        physics=vehicle.get_physics_control()
        wheels=physics.wheels
        self.wheelbase=norm((wheels[0].position+wheels[1].position-wheels[2].position-wheels[3].position)*.5)/100
        self.max_steer=max(w.max_steer_angle for w in wheels)

    def control_step(self,detection):
        pose=self._vehicle.get_transform()
        current=self._map.get_waypoint(pose.location)
        upcoming=list(self.get_local_planner().get_plan())[:7]
        # Slow before bends, including map curves that are not junctions.
        curvature=max((abs((wp.transform.rotation.yaw-current.transform.rotation.yaw+180)%360-180)
                       for wp,_ in upcoming),default=0)
        self._behavior.max_speed=min(self.cruise_speed,8) if curvature>10 else self.cruise_speed
        control=super().run_step()
        original=float(control.steer)
        source='BEHAVIOR_AGENT'
        upcoming=list(self.get_local_planner().get_plan())[:7]
        straight=bool(upcoming) and not current.is_junction and all(
            not wp.is_junction and option in (RoadOption.LANEFOLLOW,RoadOption.STRAIGHT)
            and abs((wp.transform.rotation.yaw-pose.rotation.yaw+180)%360-180)<8
            for wp,option in upcoming)
        target=vision_target(detection,W,H)
        visual=None
        if control.brake>.05:
            source='BEHAVIOR_AGENT_BRAKE'
        elif not self.agent_only and straight and target is not None:
            visual=pursuit_steer(target,self.wheelbase,self.max_steer)
            if abs(visual-original)<=.12:
                prior=self._vehicle.get_control().steer
                control.steer=clip(visual,prior-.04,prior+.04)
                self.get_local_planner()._vehicle_controller.past_steering=control.steer
                source='BOUNDARY_VISION' if detection['detection_fresh'] else 'BOUNDARY_HELD'
            else:source='BEHAVIOR_AGENT_DISAGREEMENT'
        elif not self.agent_only:
            source='BEHAVIOR_AGENT_TURN' if not straight else 'BEHAVIOR_AGENT_NO_DETECTION'
        return control,{'steering_source':source,'upstream_steer':original,'vision_steer':visual,
                        'target_speed_kmh':self._behavior.max_speed}


def connect():
    client=carla.Client('127.0.0.1',2000);client.set_timeout(3)
    try:client.get_world()
    except RuntimeError:
        print('CARLA 서버를 시작합니다.',flush=True)
        (BASE/'carla-runs').mkdir(exist_ok=True)
        with (BASE/'carla-runs/open-source-launch.log').open('a') as log:
            supervisor=subprocess.Popen([sys.executable,str(BASE/'launch_carla.py')],cwd=BASE,
                                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            if supervisor.poll() is not None:raise RuntimeError('CARLA startup failed; see carla-runs/open-source-launch.log')
            try:client.get_world();break
            except RuntimeError:time.sleep(1)
        else:raise RuntimeError('CARLA startup timed out')
    client.set_timeout(15)
    if client.get_server_version()!=client.get_client_version():raise RuntimeError('CARLA version mismatch')
    return client


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=0,help='Total run duration; 0 keeps driving')
    parser.add_argument('--record-seconds',type=float,default=150,help='Save video/log for this many simulation seconds')
    parser.add_argument('--speed',type=float,default=15)
    parser.add_argument('--agent-only',action='store_true',help='Use upstream steering without visual override')
    parser.add_argument('--reset',action='store_true',help='Reset our car to spawn 0')
    args=parser.parse_args()
    if not 0<args.speed<=25 or args.seconds<0 or args.record_seconds<=0:parser.error('Invalid speed or duration')
    cv2.setNumThreads(2)
    lock=(BASE/'.opensource-autonomy.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('오픈소스 자동주행 창이 이미 실행 중입니다.')
    client=connect();world=client.get_world();map_=world.get_map();old_settings=world.get_settings()
    if old_settings.synchronous_mode:raise RuntimeError('Another client owns synchronous mode')
    if not map_.name.endswith('/Town02_Opt'):raise RuntimeError('This configured route requires Town02_Opt')
    existing=[v for v in world.get_actors().filter('vehicle.*') if v.attributes.get('role_name')=='opensource-autonomy']
    vehicle=existing[0] if existing else None
    if vehicle and any(a.parent and a.parent.id==vehicle.id for a in world.get_actors().filter('sensor.*')):
        raise RuntimeError('Another sensor client is using the autonomy vehicle')
    created=False
    if vehicle is None:
        bp=world.get_blueprint_library().find('vehicle.tesla.model3');bp.set_attribute('role_name','opensource-autonomy')
        vehicle=world.try_spawn_actor(bp,map_.get_spawn_points()[0])
        if vehicle is None:raise RuntimeError('Spawn 0 occupied; no existing vehicle was moved')
        created=True
    vehicle.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
    world.wait_for_tick(10)
    output=BASE/'opensource-runs'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True)
    metadata={'upstream':'CARLA BehaviorAgent 0.9.16','license':'MIT',
              'source':'https://github.com/carla-simulator/carla/tree/0.9.16/PythonAPI/carla/agents',
              'upstream_files_modified':False,'map':map_.name,'vehicle_id':vehicle.id,
              'traffic_manager_autopilot':False,'vision_steering_enabled':not args.agent_only,
              'signals_and_obstacles':'CARLA world API, not camera-only perception',
              'speed_limit_kmh':args.speed,'fixed_delta_seconds':DT,'record_seconds':args.record_seconds,
              'source_hashes':{n:hashlib.sha256((BASE/n).read_bytes()).hexdigest()
                               for n in ['open_source_autonomy.py','boundary_controller.py','road_boundary_recovery/road_boundary_recovery.py']}}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    root=tk.Tk();root.title('오픈소스 자율주행 — CARLA BehaviorAgent + 경계 검출');root.configure(bg='#111827')
    canvas=tk.Label(root,bg='black');canvas.pack()
    status=tk.StringVar(value='경로 계획 중 — 자동으로 출발합니다')
    tk.Label(root,textvariable=status,bg='#111827',fg='white',padx=12,pady=9,anchor='w').pack(fill='x')
    tk.Label(root,text='파랑/노랑: 경계 검출 · BOUNDARY: 영상 조향 · BEHAVIOR: 오픈소스 조향 · Space: 일시정지/계속 · Esc: 종료',
             bg='#1f2937',fg='white',padx=12,pady=8).pack(fill='x')
    stopped=False;paused=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    def toggle(*_):
        nonlocal paused
        paused=not paused
    buttons=tk.Frame(root,bg='#111827');buttons.pack(fill='x')
    tk.Button(buttons,text='일시정지 / 계속',command=toggle,padx=12).pack(side='left',padx=8,pady=6)
    tk.Button(buttons,text='주행 종료',command=stop,padx=12).pack(side='right',padx=8,pady=6)
    root.bind('<space>',toggle);root.bind('<Escape>',stop);root.protocol('WM_DELETE_WINDOW',stop)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    frames=queue.Queue();sensors=[];collisions=[];lane_crossings=[]
    writer=log=None;settings_changed=False;recording=True;rows=[];events=[];error=None
    first_time=None;last_time=None;distance=0;previous_location=None;goal_count=0
    red_episode=None;green_time=None;stop_since=None;red_logged=False
    junction_entry=None;turn_count=0;last_source=None

    def finish_recording(reason):
        nonlocal recording
        if not recording:return
        recording=False
        if writer:writer.release()
        if log:log.close()
        moving=[r for r in rows if r['speed_kmh']>1]
        road=[r for r in moving if not r['junction']]
        modes=Counter(r['steering_source'] for r in moving)
        errs=[abs(r['lane_error_m']) for r in road]
        summary={**metadata,'end_reason':reason,'frames':len(rows),'elapsed_s':rows[-1]['elapsed_s'] if rows else 0,
                 'travelled_m':round(distance,2),'destinations_reached':goal_count,'turns_completed':turn_count,
                 'steering_sources_moving':dict(modes),
                 'vision_percent_moving':round(100*sum(v for k,v in modes.items() if k.startswith('BOUNDARY'))/max(len(moving),1),2),
                 'road_center_abs_mean_m':round(float(np.mean(errs)),4) if errs else None,
                 'road_center_abs_p95_m':round(float(np.percentile(errs,95)),4) if errs else None,
                 'road_center_abs_max_m':round(max(errs),4) if errs else None,
                 'vehicle_outside_lane_road_frames':sum(r['lane_clearance_m']<0 for r in road),
                 'red_stops':sum(e['event']=='red_stop' for e in events),
                 'green_departures':sum(e['event']=='green_departure' for e in events),
                 'collisions':list(collisions),'lane_crossing_events':list(lane_crossings),
                 'events':list(events),'error':error,'manual_driving_inputs':0}
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        print('RECORDING_SAVED='+json.dumps(summary),flush=True)

    def tick(image_required=True):
        root.update_idletasks();root.update()
        if stopped:raise InterruptedError('User closed the window')
        frame_id=world.tick(15)
        if not image_required:return None
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            try:image=frames.get(timeout=max(.01,deadline-time.monotonic()))
            except queue.Empty as exc:raise RuntimeError('Camera timed out') from exc
            if image.frame==frame_id:return image
            if image.frame>frame_id:raise RuntimeError('Camera/simulation frame mismatch')
        raise RuntimeError('Camera frame missing')

    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=DT
        world.apply_settings(settings);settings_changed=True
        if args.reset and not created:
            vehicle.set_simulate_physics(False);vehicle.set_transform(map_.get_spawn_points()[0]);world.tick(10)
            vehicle.set_simulate_physics(True);vehicle.set_target_velocity(carla.Vector3D());vehicle.set_target_angular_velocity(carla.Vector3D())
        for _ in range(12):world.tick(10)
        agent=BoundaryBehaviorAgent(vehicle,args.speed,args.agent_only)
        destinations=[map_.get_waypoint(carla.Location(x=x,y=y)).transform.location for x,y in
                      [(20,191.56),(41.85,220),(10,302.55),(-3.45,145)]]
        goal_index=0
        if not created and not args.reset:
            distances=[vehicle.get_location().distance(p) for p in destinations]
            goal_index=(int(np.argmin(distances))+1)%len(destinations)
        def plan_next():
            nonlocal goal_index
            target=destinations[goal_index%len(destinations)]
            agent.set_destination(target,start_location=vehicle.get_location())
            print(f'DESTINATION {goal_index%len(destinations)+1}: {target}; {len(agent.get_local_planner().get_plan())} route points',flush=True)
            goal_index+=1
        plan_next()
        bp=world.get_blueprint_library().find('sensor.camera.rgb')
        for k,v in [('image_size_x',str(W)),('image_size_y',str(H)),('fov','90'),('sensor_tick','0.0')]:bp.set_attribute(k,v)
        camera=world.spawn_actor(bp,carla.Transform(carla.Location(x=1.5,z=1.6)),attach_to=vehicle)
        camera.listen(frames.put);sensors.append(camera)
        collision=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=vehicle)
        collision.listen(lambda e:collisions.append({'frame':e.frame,'other_id':e.other_actor.id,'impulse':norm(e.normal_impulse)}));sensors.append(collision)
        lane_sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.lane_invasion'),carla.Transform(),attach_to=vehicle)
        lane_sensor.listen(lambda e:lane_crossings.append({'frame':e.frame,'markings':[str(x.type) for x in e.crossed_lane_markings]}));sensors.append(lane_sensor)
        for _ in range(4):tick(False)
        writer=cv2.VideoWriter(str(output/'drive.mp4'),cv2.VideoWriter_fourcc(*'avc1'),1/DT,(W,H))
        if not writer.isOpened():raise RuntimeError('Could not create H264 recording')
        log=(output/'frames.jsonl').open('w',buffering=1)
        tracker=BoundaryTracker()
        print(f'OUTPUT={output}',flush=True)
        iteration=0
        while not stopped:
            image=tick();snapshot=world.get_snapshot()
            if snapshot.frame!=image.frame:raise RuntimeError('Camera/state alignment failed')
            last_time=image.timestamp
            if first_time is None:first_time=last_time
            elapsed=last_time-first_time
            pose=vehicle.get_transform();loc=pose.location;speed=norm(vehicle.get_velocity())
            if previous_location is not None:distance+=loc.distance(previous_location)
            previous_location=loc
            wp=map_.get_waypoint(loc);right=wp.transform.get_right_vector()
            lane_error=dot(loc-wp.transform.location,right)
            heading=math.radians((pose.rotation.yaw-wp.transform.rotation.yaw+180)%360-180)
            box=vehicle.bounding_box
            clearance=wp.lane_width/2-abs(lane_error)-abs(math.cos(heading))*box.extent.y-abs(math.sin(heading))*box.extent.x
            if wp.is_junction and junction_entry is None:junction_entry=(pose.rotation.yaw,elapsed)
            elif not wp.is_junction and junction_entry is not None:
                change=abs((pose.rotation.yaw-junction_entry[0]+180)%360-180)
                if change>35:
                    turn_count+=1;events.append({'event':'turn_completed','elapsed_s':elapsed,'heading_change_deg':change})
                junction_entry=None
            bgr=np.frombuffer(image.raw_data,dtype=np.uint8).reshape(H,W,4)[:,:,:3].copy()
            result=tracker.update(bgr,last_time)
            if agent.done():
                goal_count+=1;events.append({'event':'destination_reached','elapsed_s':elapsed});plan_next()
            active_red=agent.traffic_light_manager()
            active_light=agent._last_traffic_light
            if active_red and active_light is not None:
                if red_episode is None or red_episode.id!=active_light.id:
                    red_episode=active_light;green_time=None;stop_since=None;red_logged=False
                if speed<.1:
                    if stop_since is None:stop_since=last_time
                    if last_time-stop_since>=1 and not red_logged:
                        red_logged=True;events.append({'event':'red_stop','elapsed_s':elapsed,'light_id':active_light.id,'video_frame':len(rows)})
                        print(f'RED STOP light={active_light.id} at {elapsed:.1f}s',flush=True)
                else:stop_since=None
            if red_episode is not None and red_episode.get_state()==carla.TrafficLightState.Green:
                if green_time is None:green_time=last_time
                if red_logged and speed>.5:
                    events.append({'event':'green_departure','elapsed_s':elapsed,'light_id':red_episode.id,
                                   'delay_s':round(last_time-green_time,3),'video_frame':len(rows)})
                    print(f'GREEN DEPARTURE light={red_episode.id} delay={last_time-green_time:.2f}s',flush=True)
                    red_episode=None;red_logged=False
            current_light=active_light or red_episode or vehicle.get_traffic_light()
            signal_name=str(current_light.get_state()) if current_light is not None else 'None'
            control,details=agent.control_step(result)
            if paused:control=carla.VehicleControl(brake=1,hand_brake=True);details['steering_source']='PAUSED'
            vehicle.apply_control(control)
            row={'frame_id':image.frame,'timestamp_s':last_time,'elapsed_s':elapsed,
                 'speed_kmh':speed*3.6,'lane_error_m':lane_error,'lane_clearance_m':clearance,'junction':wp.is_junction,
                 'road_id':wp.road_id,'lane_id':wp.lane_id,'signal':signal_name,
                 'light_id':current_light.id if current_light else None,'red_hazard':bool(active_red),
                 'steer':control.steer,'throttle':control.throttle,'brake':control.brake,**details,
                 'detector':result,'location':{'x':loc.x,'y':loc.y,'z':loc.z}}
            annotated=draw_result(bgr,result)
            cv2.rectangle(annotated,(0,0),(W,148),(18,24,32),-1)
            cv2.putText(annotated,'OPEN SOURCE: CARLA BehaviorAgent 0.9.16 (MIT)',(14,29),cv2.FONT_HERSHEY_SIMPLEX,.74,(255,255,255),2)
            cv2.putText(annotated,f'{details["steering_source"]} | {speed*3.6:.1f} km/h | light: {signal_name}',(14,62),cv2.FONT_HERSHEY_SIMPLEX,.7,(100,230,255),2)
            cv2.putText(annotated,f'Lane error {lane_error:+.2f} m | goals {goal_count} | turns {turn_count} | {elapsed:.1f}s',(14,94),cv2.FONT_HERSHEY_SIMPLEX,.65,(230,240,255),2)
            cv2.putText(annotated,'Route/signals/obstacles: simulator data | BOUNDARY: uploaded camera detector steers',(14,126),cv2.FONT_HERSHEY_SIMPLEX,.56,(180,210,230),1)
            if recording:
                rows.append(row);log.write(json.dumps(row)+'\n');writer.write(annotated)
                if details['steering_source']!=last_source:
                    cv2.imwrite(str(output/(details['steering_source']+'.jpg')),annotated)
                if elapsed>=args.record_seconds:finish_recording('recording_window_complete_driving_continues')
            last_source=details['steering_source']
            if iteration%2==0:
                preview=ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(cv2.resize(annotated,(960,540)),cv2.COLOR_BGR2RGB)))
                canvas.configure(image=preview);canvas.image=preview
                status.set(f'{"일시정지" if paused else "자동주행 중"} · {speed*3.6:.1f} km/h · 목적지 {goal_count}개 · 회전 {turn_count}회 · '+('녹화 중' if recording else '검증 영상 저장 완료'))
            if iteration%100==0:
                cv2.imwrite(str(output/'latest.jpg'),annotated)
                print(f't={elapsed:.1f}s speed={speed*3.6:.1f} source={details["steering_source"]} error={lane_error:+.2f} signal={signal_name} goals={goal_count} turns={turn_count}',flush=True)
            world.get_spectator().set_transform(carla.Transform(loc-pose.get_forward_vector()*7+carla.Location(z=3.5),carla.Rotation(pitch=-15,yaw=pose.rotation.yaw)))
            if collisions:raise RuntimeError('Collision detected; stopped for inspection')
            if args.seconds and elapsed>=args.seconds:break
            iteration+=1
    except InterruptedError:
        pass
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}';print('ERROR='+error,flush=True)
    finally:
        try:vehicle.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
        except Exception:pass
        for sensor in sensors:
            try:sensor.stop();sensor.destroy()
            except Exception:pass
        if settings_changed:
            try:world.apply_settings(old_settings)
            except Exception as exc:print('SETTINGS RESTORE ERROR:',exc,flush=True)
        finish_recording('window_closed_or_duration_reached' if not error else 'error')
        (output/'exit.json').write_text(json.dumps({'error':error,'parked':True,'elapsed_s':(last_time-first_time) if first_time else 0},indent=2))
        root.destroy()
    if error:raise SystemExit(1)


if __name__=='__main__':main()
