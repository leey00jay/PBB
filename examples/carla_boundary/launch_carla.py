#!/usr/bin/env python3
"""Launch a small CARLA map and stop it before system memory becomes critical."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

BASE = Path(__file__).resolve().parent
CARLA = Path(os.environ.get('CARLA_ROOT', str(Path.home() / 'CARLA'))).expanduser()


def memory():
    return {k: int(v.split()[0]) for k, v in
            (line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offscreen', action='store_true')
    args = parser.parse_args()
    mem = memory()
    if mem['SwapTotal'] < 15 * 1024**2:
        raise SystemExit('먼저 setup_carla_swap.sh로 16GiB 스왑을 활성화하세요.')
    if mem['MemAvailable'] < 6 * 1024**2:
        raise SystemExit('실행 전 사용 가능한 RAM이 최소 6GiB 필요합니다.')
    for port in (2000, 2001, 2002):
        with socket.socket() as sock:
            sock.settimeout(0.5)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise SystemExit(f'Port {port} is in use; refusing a duplicate launch.')
    subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
                   check=True, timeout=10)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    run_dir = BASE / 'carla-runs' / stamp
    run_dir.mkdir(parents=True)
    env = os.environ.copy()
    env.update(__NV_PRIME_RENDER_OFFLOAD='1',
               __VK_LAYER_NV_optimus='NVIDIA_only',
               __GLX_VENDOR_LIBRARY_NAME='nvidia',
               VK_ICD_FILENAMES='/usr/share/vulkan/icd.d/nvidia_icd.json')
    command = [str(CARLA / 'CarlaUE4.sh'),
               '-quality-level=Low', '-windowed', '-ResX=960', '-ResY=540',
               '-nosound', '-carla-rpc-port=2000', '-graphicsadapter=0',
               '-log', '-stdout', '-FullStdOutLogOutput',
               f'-abslog={run_dir / "engine.log"}']
    if args.offscreen:
        command.append('-RenderOffScreen')
    def prefer_killing_carla():
        Path('/proc/self/oom_score_adj').write_text('500')
    with (run_dir / 'server.log').open('w') as log:
        child = subprocess.Popen(command, cwd=CARLA, env=env,
                                 stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True, preexec_fn=prefer_killing_carla)
    info = {'supervisor_pid': os.getpid(), 'server_group': child.pid,
            'run_dir': str(run_dir), 'command': command, 'status': 'running'}
    state = BASE / 'carla-runs' / 'current.json'
    state.write_text(json.dumps(info, indent=2))
    print(f'CARLA PID {child.pid}; logs: {run_dir}', flush=True)
    print('RAM 여유가 2GiB 미만이면 자동 종료합니다. Ctrl+C로 종료할 수 있습니다.', flush=True)
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        with (run_dir / 'memory.jsonl').open('w', buffering=1) as metrics:
            last_report = 0
            while child.poll() is None and not stopping:
                mem = memory()
                row = {'time': datetime.datetime.now().isoformat(timespec='seconds'),
                       'available_mib': round(mem['MemAvailable'] / 1024),
                       'swap_used_mib': round((mem['SwapTotal'] - mem['SwapFree']) / 1024)}
                metrics.write(json.dumps(row) + '\n')
                if mem['MemAvailable'] < 2 * 1024**2:
                    info['stop_reason'] = 'available RAM below 2 GiB'
                    print('메모리 보호: CARLA를 종료합니다.', flush=True)
                    break
                if mem['SwapFree'] < 1024**2:
                    info['stop_reason'] = 'free swap below 1 GiB'
                    print('스왑 여유 부족: CARLA를 종료합니다.', flush=True)
                    break
                if time.monotonic() - last_report >= 15:
                    print(json.dumps(row), flush=True)
                    last_report = time.monotonic()
                time.sleep(0.5)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        info.update(status='stopped', returncode=child.returncode)
        state.write_text(json.dumps(info, indent=2))
        print(f'CARLA stopped: {child.returncode}', flush=True)


if __name__ == '__main__':
    main()
