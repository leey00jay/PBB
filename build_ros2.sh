#!/usr/bin/env bash
set -e
pbb_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$pbb_root"
# Ignore reference materials and local nested checkouts; build only src packages.
PATH="/usr/bin:/bin:$PATH" /usr/bin/colcon build --symlink-install --base-paths src     --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
