"""Check readiness gating and process cleanup without starting CARLA or a GPU."""

import importlib.util
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wait_for_carla', ROOT / 'src/pbb_bringup/pbb_bringup/readiness.py')
readiness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(readiness)


class ReadinessTests(unittest.TestCase):
    def test_world_must_be_readable_before_ready(self):
        client = Mock()
        client.get_server_version.return_value = 'test'
        client.get_world.side_effect = [RuntimeError('loading'), Mock()]
        with patch.object(readiness.time, 'sleep'):
            factory = Mock(return_value=client)
            self.assertTrue(readiness.wait_for_server(factory, 1))
        self.assertEqual(factory.call_count, 2)
        self.assertEqual(client.get_world.call_count, 2)

    def test_timeout_is_failure(self):
        client = Mock()
        client.get_server_version.side_effect = RuntimeError('offline')
        self.assertFalse(readiness.wait_for_server(lambda: client, 0.01))
        client.get_world.assert_not_called()


class ProcessTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as probe:
            probe.settimeout(0.2)
            if probe.connect_ex(('127.0.0.1', 2000)) == 0:
                self.skipTest('An existing CARLA server is using port 2000')
        self.temp = tempfile.TemporaryDirectory(prefix='pbb-launch-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.env = dict(os.environ, PBB_TEST_ROOT=str(self.root),
                        ROS_LOG_DIR=str(self.root / 'ros-logs'))
        self.carla = self.root / 'CARLA'
        self.carla.mkdir()
        fake = self.carla / 'CarlaUE4.sh'
        fake.write_text('''#!/usr/bin/python3
import os, signal, time
from pathlib import Path
root = Path(os.environ['PBB_TEST_ROOT'])
def stop(*args):
    (root / 'server_stopped').touch()
    raise SystemExit(0)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
(root / 'server_started').write_text(str(os.getpid()))
while True: time.sleep(0.05)
''')
        fake.chmod(0o755)
        self.process = None
        self.addCleanup(self.cleanup_process)

    def cleanup_process(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.communicate()

    def await_file(self, name, timeout=8):
        deadline = time.monotonic() + timeout
        while not (self.root / name).exists():
            if self.process.poll() is not None:
                self.fail(self.process.communicate()[0])
            if time.monotonic() > deadline:
                self.fail(f'Timed out waiting for {name}')
            time.sleep(0.05)

    def test_server_handles_sigterm_and_stops_owned_child(self):
        self.process = subprocess.Popen(
            [sys.executable, str(ROOT / 'start_carla.py'), '--carla-root', str(self.carla)],
            env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.await_file('server_started')
        self.process.terminate()
        output = self.process.communicate(timeout=10)[0]
        self.assertEqual(self.process.returncode, 0, output)
        self.assertTrue((self.root / 'server_stopped').exists(), output)

    def prepare_launch(self, fail_ready=False, delayed_ready=False):
        try:
            from launch import LaunchDescription  # noqa: F401
        except ImportError:
            self.skipTest('Source ROS 2 Humble to run launch integration checks')
        # Exercise the maintained launch; replace only GPU-dependent executables.
        prefix = self.root / 'fake_install'
        roles = {'perception': 'pbb_perception', 'decision': 'pbb_decision',
                 'control': 'pbb_control', 'bridge': 'pbb_carla', 'display': 'pbb_visualization'}
        for role, package in roles.items():
            marker = prefix / 'share/ament_index/resource_index/packages' / package
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.touch()
            (prefix / 'share' / package).mkdir(parents=True)
            executable = prefix / 'lib' / package / role
            executable.parent.mkdir(parents=True)
            executable.write_text("""#!/usr/bin/python3
import os, signal, time
from pathlib import Path
root = Path(os.environ['PBB_TEST_ROOT'])
assert (root / 'ready').exists(), 'Node started before CARLA readiness'
role = Path(__file__).name
(root / ('node_' + role)).touch()
def stop(*args): raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while True:
    if role == 'display' and len(list(root.glob('node_*'))) == 5: break
    time.sleep(0.05)
""")
            executable.chmod(0o755)
        package = self.root / 'pbb_bringup'
        package.mkdir()
        (package / '__init__.py').write_text(
            '__path__.append(' + repr(str(ROOT / 'src/pbb_bringup/pbb_bringup')) + ')')
        (package / 'readiness.py').write_text("""import os, time
from pathlib import Path
root = Path(os.environ['PBB_TEST_ROOT'])
while not (root / 'server_started').exists(): time.sleep(0.02)
time.sleep(DELAY)
if FAIL: raise SystemExit(1)
(root / 'ready').touch()
""".replace('DELAY', '60' if delayed_ready else '0.1').replace('FAIL', str(fail_ready)))
        self.env['PYTHONPATH'] = str(self.root) + os.pathsep + self.env.get('PYTHONPATH', '')
        self.env['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + self.env.get('AMENT_PREFIX_PATH', '')
        self.process = subprocess.Popen(
            ['/opt/ros/humble/bin/ros2', 'launch', str(ROOT / 'src/pbb_bringup/launch/pbb.launch.py'),
             f'carla_root:={self.carla}', f'project_root:={self.root}',
             f'python_executable:={sys.executable}'], env=self.env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def test_ready_starts_all_five_packages_and_exit_stops_server(self):
        self.prepare_launch()
        output = self.process.communicate(timeout=40)[0]
        self.assertEqual(self.process.returncode, 0, output)
        self.assertTrue((self.root / 'server_stopped').exists(), output)
        self.assertEqual({p.name for p in self.root.glob('node_*')},
                         {'node_perception', 'node_decision', 'node_control', 'node_bridge', 'node_display'})

    def test_readiness_failure_stops_server_without_driving(self):
        self.prepare_launch(fail_ready=True)
        output = self.process.communicate(timeout=40)[0]
        self.assertNotEqual(self.process.returncode, 0, output)
        self.assertFalse(list(self.root.glob('node_*')), output)
        self.assertTrue((self.root / 'server_stopped').exists(), output)

    def test_ctrl_c_during_startup_cannot_start_driving(self):
        self.prepare_launch(delayed_ready=True)
        self.await_file('server_started')
        self.process.send_signal(signal.SIGINT)
        output = self.process.communicate(timeout=40)[0]
        self.assertFalse(list(self.root.glob('node_*')), output)
        self.assertTrue((self.root / 'server_stopped').exists(), output)


if __name__ == '__main__':
    unittest.main()
