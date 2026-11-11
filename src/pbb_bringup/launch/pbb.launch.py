"""Start the CARLA server and five independent ROS 2 topic-based nodes."""
from datetime import datetime
import math
from pathlib import Path
import socket
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    config = lambda name: LaunchConfiguration(name).perform(context)
    root = Path(config('project_root')).expanduser().resolve()
    drive, display, record = [config(name) == 'true' for name in ('drive', 'display', 'record')]
    timeout, command_timeout, speed = [float(config(name)) for name in ('startup_timeout', 'command_timeout', 'desired_speed_kmh')]
    if not all(math.isfinite(x) for x in (timeout, command_timeout, speed)) or min(timeout, command_timeout) <= 0 or speed < 0:
        raise RuntimeError('Invalid timeout or desired speed')
    if record and config('record_mode') == 'video' and not display:
        raise RuntimeError('Video recording requires display:=true; use record_mode:=all for headless data')
    if record and not drive:
        raise RuntimeError('record:=true requires drive:=true')
    carla_root = Path(config('carla_root')).expanduser().resolve()
    python = Path(config('python_executable')).expanduser().absolute()
    for path in [carla_root / 'CarlaUE4.sh'] + ([python] if drive else []):
        if not path.is_file():
            raise RuntimeError(f'Missing required file: {path}')
    if drive:
        from ament_index_python.packages import get_package_share_directory
        for package in ('pbb_perception', 'pbb_decision', 'pbb_control', 'pbb_carla', 'pbb_visualization'):
            get_package_share_directory(package)
        get_package_share_directory('pbb_interfaces')
    with socket.socket() as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(('127.0.0.1', 2000)) == 0:
            raise RuntimeError('2000번 포트가 사용 중입니다. 기존 CARLA 서버를 먼저 종료하세요.')

    def stop_on_exit(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(f'{event.process_name} exited with code {event.returncode}')
        return [Shutdown(reason=f'{event.process_name} finished')]

    def supervise(action):
        return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=stop_on_exit))

    server = ExecuteProcess(cmd=[sys.executable, '-u', '-m', 'pbb_bringup.server',
                            '--carla-root', str(carla_root), '--shutdown-grace', '10' if drive else '0'],
                            name='carla_server', output='screen', sigterm_timeout='20', sigkill_timeout='20')
    actions = [supervise(server), server]
    if not drive:
        return actions
    parameters = {'use_sim_time': True,
                  'desired_speed_kmh': speed, 'command_timeout': command_timeout, 'startup_timeout': timeout}
    roles = ['perception', 'decision', 'control', 'bridge'] + (['display'] if display else [])
    packages = {'perception': 'pbb_perception', 'decision': 'pbb_decision', 'control': 'pbb_control',
                'bridge': 'pbb_carla', 'display': 'pbb_visualization'}
    nodes = [Node(package=packages[role], executable=role, name='pbb_' + role,
                  prefix=[str(python) + ' -u'], parameters=[parameters], output='screen',
                  sigterm_timeout='15', sigkill_timeout='5') for role in roles]
    ready = ExecuteProcess(cmd=[str(python), '-u', '-m', 'pbb_bringup.readiness',
                                '--timeout', str(timeout)], name='carla_ready', output='screen')
    extra = []
    if record:
        bag = (root / 'bags' / datetime.now().strftime('ros2_town10_%Y%m%d_%H%M%S_%f')
               if config('bag_path') == 'auto' else Path(config('bag_path')).expanduser().resolve())
        if bag.exists():
            raise RuntimeError(f'Bag destination already exists: {bag}')
        bag.parent.mkdir(parents=True, exist_ok=True)
        topics = ['/clock', '/pbb/vehicle/state', '/pbb/perception/lanes', '/pbb/decision/plan',
                  '/pbb/control/command', '/pbb/control/applied', '/pbb/perception/overlay',
                  '/pbb/perception/boundary', '/pbb/front/image/compressed', '/pbb/screen/image/compressed']
        topics += [f'/pbb/camera/{name}/image_raw' for name in ('left', 'center', 'right', 'chase')]
        if config('record_mode') == 'video':
            topics = ['/clock', '/pbb/front/image/compressed', '/pbb/screen/image/compressed']
        recorder = ExecuteProcess(cmd=['/opt/ros/humble/bin/ros2', 'bag', 'record', '--use-sim-time',
                                       '-o', str(bag), *topics], name='pbb_bag_record', output='screen',
                                  sigterm_timeout='15', sigkill_timeout='5')
        actions.insert(0, supervise(recorder))
        extra = [LogInfo(msg=f'ROS topic recording: {bag}'), recorder]

    def start_nodes(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError('CARLA readiness check failed; nodes were not started')
        return [LogInfo(msg='CARLA ready. Starting independent ROS 2 nodes.'), *extra, *nodes]

    actions[0:0] = [supervise(node) for node in nodes]
    actions.insert(0, RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_nodes)))
    actions.append(ready)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(Path.cwd()), description='Workspace root for bags'),
        DeclareLaunchArgument('python_executable', default_value=str(Path.cwd() / '.venv/bin/python'),
                              description='Python with CARLA, Torch, OpenCV and pygame'),
        DeclareLaunchArgument('carla_root', default_value=str(Path.home() / 'CARLA')),
        DeclareLaunchArgument('drive', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('display', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('record', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('record_mode', default_value='all', choices=['all', 'video'],
                              description='all: all 14 topics; video: screen, front image and clock'),
        DeclareLaunchArgument('bag_path', default_value='auto', description='New bag folder, or auto'),
        DeclareLaunchArgument('desired_speed_kmh', default_value='80.0'),
        DeclareLaunchArgument('startup_timeout', default_value='120.0'),
        DeclareLaunchArgument('command_timeout', default_value='5.0'),
        OpaqueFunction(function=_start),
    ])
