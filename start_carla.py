"""Compatibility shortcut; the maintained server launcher lives in pbb_bringup."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parent / 'src/pbb_bringup/pbb_bringup/server.py'), run_name='__main__')
