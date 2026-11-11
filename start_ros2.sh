#!/usr/bin/env bash
set -e
pbb_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$pbb_root/install/setup.bash" ]]; then
    bash "$pbb_root/build_ros2.sh"
fi
source "$pbb_root/install/setup.bash"
cd "$pbb_root"
exec /opt/ros/humble/bin/ros2 launch pbb_bringup pbb.launch.py     "project_root:=$pbb_root" "python_executable:=$pbb_root/.venv/bin/python" "$@"
