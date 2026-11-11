"""Start the local CARLA server and clean up its process group on exit."""

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def stop_server(process, timeout=5.0):
    """Stop only the CARLA process group created by this launcher."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait()
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.1)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Start the PBB CARLA server.')
    parser.add_argument('--carla-root', type=Path, default=Path.home() / 'CARLA')
    parser.add_argument('--shutdown-grace', type=float, default=0.0,
                        help='Keep the server alive briefly while clients clean up')
    args = parser.parse_args(argv)
    if not math.isfinite(args.shutdown_grace) or args.shutdown_grace < 0:
        parser.error('--shutdown-grace must be a finite nonnegative number')
    carla_folder = args.carla_root.expanduser().resolve()
    executable = carla_folder / 'CarlaUE4.sh'
    if not executable.is_file():
        parser.error(f'CARLA 실행 파일이 없습니다: {executable}')
    with socket.socket() as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(('127.0.0.1', 2000)) == 0:
            parser.error('2000번 포트가 사용 중입니다. 기존 CARLA 서버를 먼저 종료하세요.')
    # Keep this notebook's NVIDIA settings and existing server window size.
    environment = os.environ.copy()
    environment.update({
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__VK_LAYER_NV_optimus': 'NVIDIA_only',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        'VK_ICD_FILENAMES': '/usr/share/vulkan/icd.d/nvidia_icd.json',
    })
    command = [str(executable), '-quality-level=low', '-windowed',
               '-ResX=800', '-ResY=600', '-nosound', '-carla-rpc-port=2000',
               '-graphicsadapter=0']
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    old_handlers = {sig: signal.signal(sig, request_stop)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    process = None
    try:
        process = subprocess.Popen(command, cwd=carla_folder, env=environment,
                                   start_new_session=True)
        print('CARLA를 시작합니다. 종료하려면 Ctrl+C를 누르세요.', flush=True)
        while not stopping and process.poll() is None:
            time.sleep(0.1)
        return 0 if stopping else process.returncode
    finally:
        if process is not None:
            if stopping and args.shutdown_grace and process.poll() is None:
                print('주행 프로그램이 센서를 정리하도록 잠시 기다립니다.', flush=True)
                deadline = time.monotonic() + args.shutdown_grace
                while process.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.1)
            print('CARLA 서버를 종료합니다.', flush=True)
            stop_server(process)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
