"""Read-only CARLA RPC readiness check; does not change or tick the world."""

import argparse
import math
import time


def wait_for_server(client_factory, timeout):
    deadline = time.monotonic() + timeout
    print(f'CARLA 연결을 기다립니다 (최대 {timeout:g}초).', flush=True)
    last_error = None
    while time.monotonic() < deadline:
        # Reconnect after startup failures; an old streaming connection may be stale.
        client = client_factory()
        client.set_timeout(max(0.1, min(10.0, deadline - time.monotonic())))
        try:
            version = client.get_server_version()
            world = client.get_world()
            map_name = world.get_map().name
        except RuntimeError as error:
            last_error = error
            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
        else:
            print(f'CARLA READY: {version} / {map_name}', flush=True)
            return True
    print(f'CARLA 연결 시간 초과: {last_error}', flush=True)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a finite positive number')
    import carla
    return 0 if wait_for_server(lambda: carla.Client('127.0.0.1', 2000), args.timeout) else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
