#!/usr/bin/env python3
"""Stop only the CARLA instance started by this folder's launch_carla.py."""
import json
import os
from pathlib import Path
import signal

base = Path(__file__).resolve().parent
state_path = base / 'carla-runs/current.json'
if not state_path.exists():
    raise SystemExit('No CARLA instance was started from this folder.')
state = json.loads(state_path.read_text())
pid = state['supervisor_pid']
cmdline = Path(f'/proc/{pid}/cmdline')
if state['status'] != 'running' or not cmdline.exists():
    raise SystemExit('CARLA is already stopped.')
if os.fsencode(str(base / 'launch_carla.py')) not in cmdline.read_bytes().split(b'\0'):
    raise SystemExit('Process identity changed; refusing to stop it.')
os.kill(pid, signal.SIGTERM)
print('CARLA 종료 요청을 보냈습니다.')
