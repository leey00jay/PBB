#!/usr/bin/env python3
"""Run the unchanged uploaded BoundaryTracker on a live CARLA RGB camera."""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import os
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40, help='Simulation seconds to record')
    parser.add_argument('--autopilot', action='store_true', help='Use CARLA Traffic Manager during recording')
    parser.add_argument('--keep-open', action='store_true', help='Switch to manual driving after recording')
    parser.add_argument('--reset-spawn', type=int, help='Reset the demo vehicle to this map spawn index')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    cv2.setNumThreads(2)
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10)
    world = client.get_world()
    if client.get_client_version() != client.get_server_version():
        raise RuntimeError('CARLA client/server versions differ')
    vehicles = [v for v in world.get_actors().filter('vehicle.*')
                if v.attributes.get('role_name') == 'carla-demo']
    if not vehicles:
        raise RuntimeError('Run spawn_vehicle.py --seconds 0 first')
    vehicle = vehicles[0]
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output = BASE / 'boundary-tests' / stamp
    output.mkdir(parents=True, exist_ok=False)
    metadata = {'server_version': client.get_server_version(), 'map': world.get_map().name,
                'vehicle_id': vehicle.id, 'camera': {'width': 1280, 'height': 720, 'fov': 90,
                'sensor_tick': 0.05, 'mount_m': {'x': 1.5, 'y': 0, 'z': 1.6}},
                'detector_sha256': hashlib.sha256((BASE / 'road_boundary_recovery' /
                    'road_boundary_recovery.py').read_bytes()).hexdigest(),
                'driving_controller': 'CARLA Traffic Manager' if args.autopilot else 'manual',
                'detector_controls_vehicle': False, 'requested_simulation_seconds': args.seconds,
                'process_id': os.getpid(), 'reset_spawn_index': args.reset_spawn}
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
    frames = queue.Queue(maxsize=1)
    dropped = 0
    tracker = BoundaryTracker()
    counts, reasons = Counter(), Counter()
    processing = []
    first_timestamp = last_timestamp = None
    latest = latest_raw = None
    last_received = time.monotonic()
    recorded = valid = fresh = 0
    moving_frames = moving_valid = 0
    recording = True
    autopilot = False
    held = set()
    releases = {}
    steering = 0.0
    camera = root = None
    raw_writer = result_writer = log = None
    start_location = vehicle.get_location()
    distance = 0.0
    last_location = start_location
    saved_statuses = set()
    autopilot_events = []
    tm = client.get_trafficmanager(8000) if args.autopilot else None

    def park(reason='park'):
        nonlocal autopilot
        if autopilot:
            autopilot_events.append({'event': 'autopilot_stopped', 'reason': reason,
                                     'timestamp_s': last_timestamp})
            print(f'Autopilot stopped: {reason}', flush=True)
        vehicle.set_autopilot(False, 8000)
        vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        autopilot = False

    def finish_recording(reason):
        nonlocal recording
        if not recording:
            return
        recording = False
        if raw_writer:
            raw_writer.release()
        if result_writer:
            result_writer.release()
        if log:
            log.close()
        summary = {**metadata, 'end_reason': reason, 'frames': recorded,
                   'simulation_seconds': round((last_timestamp or 0) - (first_timestamp or 0), 3),
                   'tracking_status_counts': dict(counts), 'reason_counts': dict(reasons),
                   'path_valid_frames': valid, 'fresh_detection_frames': fresh,
                   'path_output_percent': round(100 * valid / max(recorded, 1), 2),
                   'fresh_detection_percent': round(100 * fresh / max(recorded, 1), 2),
                   'moving_frames': moving_frames, 'moving_path_valid_frames': moving_valid,
                   'moving_path_output_percent': round(100 * moving_valid / moving_frames, 2) if moving_frames else None,
                   'dropped_queue_frames': dropped, 'travelled_m': round(distance, 2),
                   'autopilot_events': autopilot_events,
                   'processing_ms_mean': round(float(np.mean(processing)), 2) if processing else None,
                   'processing_ms_p95': round(float(np.percentile(processing, 95)), 2) if processing else None,
                   'accuracy_measured': False,
                   'video_note': '20 fps playback; results.jsonl contains authoritative sensor timestamps'}
        (output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        print(f'RESULT_DIR={output}', flush=True)

    def receive(image):
        nonlocal dropped
        try:
            frames.put_nowait(image)
        except queue.Full:
            try:
                frames.get_nowait()
                dropped += 1
            except queue.Empty:
                pass
            try:
                frames.put_nowait(image)
            except queue.Full:
                dropped += 1

    try:
        park()
        if args.reset_spawn is not None:
            spawn_points = world.get_map().get_spawn_points()
            if not 0 <= args.reset_spawn < len(spawn_points):
                raise ValueError('Invalid spawn index')
            vehicle.set_simulate_physics(False)
            vehicle.set_transform(spawn_points[args.reset_spawn])
            world.wait_for_tick(10)
            vehicle.set_simulate_physics(True)
            vehicle.set_target_velocity(carla.Vector3D())
            vehicle.set_target_angular_velocity(carla.Vector3D())
            park()
            for _ in range(10):
                world.wait_for_tick(10)
            last_location = vehicle.get_location()
            print(f'Reset vehicle to spawn {args.reset_spawn}: {vehicle.get_transform()}', flush=True)
        root = tk.Tk()
        root.title('CARLA 도로 경계 복구 검증 — 실시간 카메라')
        root.configure(bg='#111827')
        canvas = tk.Label(root, bg='black', takefocus=True)
        canvas.pack()
        status = tk.StringVar(value='카메라 연결 중…')
        tk.Label(root, textvariable=status, bg='#111827', fg='white', anchor='w', padx=12, pady=8).pack(fill='x')
        tk.Label(root, text='화면 클릭 → ↑ 전진 · ↓ 브레이크/후진 · ←→ 조향 · Space 정지 · S 캡처 · Esc 닫기',
                 bg='#1f2937', fg='white', padx=12, pady=8).pack(fill='x')
        canvas.bind('<Button-1>', lambda event: canvas.focus_set())

        def press(event):
            key = event.keysym
            if key in releases:
                root.after_cancel(releases.pop(key))
            held.add(key)
            if key in ('Up', 'Down', 'Left', 'Right', 'space') and autopilot:
                park('manual_key_' + key)
            if key == 'Escape':
                root.destroy()
            elif key.lower() == 's' and latest is not None:
                name = datetime.datetime.now().strftime('%H%M%S-%f')
                cv2.imwrite(str(output / f'capture-{name}.png'), latest)
                cv2.imwrite(str(output / f'capture-{name}-raw.png'), latest_raw)
            return 'break'

        def release(event):
            key = event.keysym
            def done():
                held.discard(key)
                releases.pop(key, None)
            if key in releases:
                root.after_cancel(releases[key])
            releases[key] = root.after_idle(done)

        def control():
            nonlocal steering
            try:
                if autopilot:
                    if time.monotonic() - last_received > 2:
                        park('camera_timeout')
                else:
                    velocity = vehicle.get_velocity()
                    forward = vehicle.get_transform().get_forward_vector()
                    signed_speed = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
                    direction = int('Up' in held) - int('Down' in held)
                    turn = int('Right' in held) - int('Left' in held)
                    steering = np.clip(steering + turn * .06, -.65, .65) if turn else steering * .6
                    current = vehicle.get_control()
                    command = carla.VehicleControl(reverse=current.reverse, gear=current.gear, steer=float(steering))
                    if root.focus_get() is None or 'space' in held or time.monotonic() - last_received > 2:
                        command.brake = 1.0
                        command.hand_brake = True
                        command.steer = 0.0
                    elif direction and direction * signed_speed < -.15:
                        command.brake = 1.0
                    elif direction:
                        command.reverse = direction < 0
                        command.gear = -1 if command.reverse else 1
                        command.throttle = .4 if abs(signed_speed) < (3 if command.reverse else 8) else 0
                    else:
                        command.brake = .6
                        command.hand_brake = abs(signed_speed) < .15
                    vehicle.apply_control(command)
                transform = vehicle.get_transform()
                world.get_spectator().set_transform(carla.Transform(
                    transform.location - transform.get_forward_vector() * 7 + carla.Location(z=3.5),
                    carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)))
            except RuntimeError as exc:
                print(f'CONTROL ERROR: {exc}', flush=True)
                root.destroy()
                return
            root.after(50, control)

        def refresh():
            nonlocal latest, latest_raw, last_received, first_timestamp, last_timestamp
            nonlocal recorded, valid, fresh, distance, last_location, autopilot
            nonlocal moving_frames, moving_valid
            try:
                image = frames.get_nowait()
            except queue.Empty:
                if time.monotonic() - last_received > 2:
                    status.set('카메라 응답 없음 — 차량 정지')
                root.after(10, refresh)
                return
            if last_timestamp is not None and image.timestamp <= last_timestamp:
                root.after(10, refresh)
                return
            try:
                last_received = time.monotonic()
                bgr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, :3].copy()
                started = time.perf_counter()
                result = tracker.update(bgr, image.timestamp)
                process_ms = (time.perf_counter() - started) * 1000
                if first_timestamp is None:
                    first_timestamp = image.timestamp
                    if args.autopilot:
                        tm.set_random_device_seed(7)
                        tm.auto_lane_change(vehicle, False)
                        tm.vehicle_percentage_speed_difference(vehicle, 55)
                        vehicle.apply_control(carla.VehicleControl())
                        vehicle.set_autopilot(True, tm.get_port())
                        autopilot = True
                        autopilot_events.append({'event': 'autopilot_started', 'timestamp_s': image.timestamp})
                        last_received = time.monotonic()
                        print('Autopilot started', flush=True)
                last_timestamp = image.timestamp
                elapsed = last_timestamp - first_timestamp
                velocity = vehicle.get_velocity()
                speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**.5 * 3.6
                location = vehicle.get_location()
                distance += location.distance(last_location)
                last_location = location
                annotated = draw_result(bgr, result)
                mode = 'CARLA AUTOPILOT' if autopilot else 'MANUAL / PARKED'
                cv2.putText(annotated, f'LIVE CARLA | {mode} | {speed:.1f} km/h | {elapsed:.1f}s',
                            (12, 114), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
                latest_raw, latest = bgr, annotated
                if recording:
                    row = {**result, 'frame_id': image.frame, 'timestamp_s': image.timestamp,
                           'elapsed_s': elapsed, 'speed_kmh': speed, 'processing_ms': process_ms,
                           'vehicle_location': {'x': location.x, 'y': location.y, 'z': location.z}}
                    log.write(json.dumps(row) + '\n')
                    raw_writer.write(bgr)
                    result_writer.write(annotated)
                    recorded += 1
                    valid += int(result['path_valid'])
                    fresh += int(result['detection_fresh'])
                    if speed > 1:
                        moving_frames += 1
                        moving_valid += int(result['path_valid'])
                    counts[result['tracking_status']] += 1
                    reasons[result['reason']] += 1
                    processing.append(process_ms)
                    state = result['tracking_status']
                    if state not in saved_statuses:
                        cv2.imwrite(str(output / f'{state}.png'), annotated)
                        cv2.imwrite(str(output / f'{state}-raw.png'), bgr)
                        saved_statuses.add(state)
                    if recorded % 100 == 0:
                        cv2.imwrite(str(output / 'latest.png'), annotated)
                        print(f'frames={recorded} sim_s={elapsed:.1f} status={dict(counts)}', flush=True)
                    if elapsed >= args.seconds:
                        park('recording_complete')
                        finish_recording('recording_duration_reached')
                        if not args.keep_open:
                            root.destroy()
                            return
                preview = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)))
                canvas.configure(image=preview)
                canvas.image = preview
                text = '자동 주행 검증 중' if autopilot else '수동 운전 / 정차'
                suffix = f'기록 {recorded}프레임' if recording else '검증 기록 저장 완료'
                status.set(f'{text} · {speed:.1f} km/h · {result["tracking_status"]} · {suffix}')
            except Exception as exc:
                print(f'CAMERA ERROR: {exc}', flush=True)
                root.destroy()
                return
            root.after(10, refresh)

        root.bind('<KeyPress>', press)
        root.bind('<KeyRelease>', release)
        root.bind('<FocusOut>', lambda event: held.clear())
        signal.signal(signal.SIGTERM, lambda *_: root.after(0, root.destroy))
        signal.signal(signal.SIGINT, lambda *_: root.after(0, root.destroy))
        raw_writer = cv2.VideoWriter(str(output / 'raw.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 20, (1280, 720))
        result_writer = cv2.VideoWriter(str(output / 'annotated.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 20, (1280, 720))
        if not raw_writer.isOpened() or not result_writer.isOpened():
            raise RuntimeError('Cannot open MP4 recording')
        log = (output / 'results.jsonl').open('w', buffering=1)
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        for key, value in [('image_size_x', '1280'), ('image_size_y', '720'), ('fov', '90'), ('sensor_tick', '0.05')]:
            bp.set_attribute(key, value)
        camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=vehicle)
        camera.listen(receive)
        print(f'LIVE camera={camera.id} vehicle={vehicle.id} output={output}', flush=True)
        root.after(10, refresh)
        root.after(50, control)
        root.after(500, root.lift)
        root.after(600, canvas.focus_set)
        root.mainloop()
    finally:
        try:
            park()
        except RuntimeError:
            pass
        if camera:
            try:
                camera.stop()
                camera.destroy()
            except RuntimeError:
                pass
        finish_recording('window_closed_or_error')
        if root:
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == '__main__':
    main()
