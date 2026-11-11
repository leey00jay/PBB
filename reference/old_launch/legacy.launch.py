"""Launch CARLA, wait for RPC readiness, then start Town10 driving."""

import math
from pathlib import Path
import socket
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def _start(context):
    root = Path(__file__).resolve().parents[1]
    carla_root = Path(LaunchConfiguration('carla_root').perform(context)).expanduser().resolve()
    drive = LaunchConfiguration('drive').perform(context) == 'true'
    record = LaunchConfiguration('record').perform(context) == 'true'
    timeout = float(LaunchConfiguration('startup_timeout').perform(context))
    if not math.isfinite(timeout) or timeout <= 0:
        raise RuntimeError('startup_timeout must be a finite positive number')
    if record and not drive:
        raise RuntimeError('record:=true requires drive:=true')
    required = [root / 'start_carla.py', carla_root / 'CarlaUE4.sh']
    python = root / '.venv/bin/python'
    if drive:
        required += [python, root / 'scripts/wait_for_carla.py',
                     root / 'aad/code/tests/control/carla_sim_boundary.py']
    for path in required:
        if not path.is_file():
            raise RuntimeError(f'실행에 필요한 파일이 없습니다: {path}')
    with socket.socket() as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(('127.0.0.1', 2000)) == 0:
            raise RuntimeError('2000번 포트가 사용 중입니다. 기존 CARLA 서버를 먼저 종료하세요.')

    def stop_on_exit(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(f'{event.process_name} exited with code {event.returncode}.')
        return [Shutdown(reason=f'{event.process_name} finished.')]

    server = ExecuteProcess(
        cmd=[sys.executable, '-u', str(root / 'start_carla.py'),
             '--carla-root', str(carla_root), '--shutdown-grace', '5' if drive else '0'],
        name='carla_server', cwd=str(root), output='screen',
        sigterm_timeout='15', sigkill_timeout='5')
    actions = [RegisterEventHandler(OnProcessExit(target_action=server, on_exit=stop_on_exit)),
               LogInfo(msg='Starting CARLA. Ctrl+C stops the launched system.'), server]
    if not drive:
        return actions

    command = [str(python), '-u', '-m', 'code.tests.control.carla_sim_boundary',
               '--ld', '--half_image', '--three-cameras']
    if record:
        command.append('--bag')
    driving = ExecuteProcess(
        cmd=command, name='pbb_driving', cwd=str(root / 'aad'), output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        sigterm_timeout='15', sigkill_timeout='5')
    ready = ExecuteProcess(
        cmd=[str(python), '-u', str(root / 'scripts/wait_for_carla.py'),
             '--timeout', str(timeout)], name='carla_ready', output='screen')

    def start_when_ready(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError('CARLA readiness check failed. Driving was not started.')
        return [LogInfo(msg='CARLA ready. Starting Town10 with three front cameras.'), driving]

    # Register handlers before starting processes so early failures are caught.
    actions[0:0] = [
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_when_ready)),
        RegisterEventHandler(OnProcessExit(target_action=driving, on_exit=stop_on_exit)),
    ]
    actions.append(ready)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('carla_root', default_value=str(Path.home() / 'CARLA'),
                              description='Folder containing CarlaUE4.sh'),
        DeclareLaunchArgument('drive', default_value='true', choices=['true', 'false'],
                              description='Start Town10 driving with three front cameras'),
        DeclareLaunchArgument('record', default_value='false', choices=['true', 'false'],
                              description='Record screen and center camera to a new ROS 2 bag'),
        DeclareLaunchArgument('startup_timeout', default_value='120',
                              description='Seconds to wait for CARLA RPC readiness'),
        OpaqueFunction(function=_start),
    ])
