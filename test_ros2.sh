#!/usr/bin/env bash
set -e
pbb_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$pbb_root/install/setup.bash"
cd "$pbb_root"
pbb_python="$pbb_root/.venv/bin/python"
[[ -x "$pbb_python" ]] || pbb_python=/usr/bin/python3
/usr/bin/python3 -m unittest discover -s tests -p test_bringup.py -v
for pbb_part in perception decision control; do
    "$pbb_python" -m unittest discover -s "src/pbb_${pbb_part}/test" -p 'test_*.py' -v
done
"$pbb_python" -m unittest discover -s tests -p test_lane_following.py -v
"$pbb_python" -m unittest discover -s tests -p test_ros_topics.py -v
