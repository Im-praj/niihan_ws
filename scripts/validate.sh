#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_root/.validation/ros-log}"
export PYTHONPATH="$workspace_root/src/niihan_description:$workspace_root/src/niihan_dashboard:${PYTHONPATH:-}"
python3 -m pytest -q -p no:cacheprovider "$workspace_root/src/niihan_description/test" "$workspace_root/src/niihan_dashboard/test"
node "$workspace_root/src/niihan_dashboard/test/test_web_viewer2d.js"
xacro "$workspace_root/src/niihan_description/urdf/niihan.urdf.xacro" > /dev/null
gz sdf -k "$workspace_root/src/niihan_description/worlds/niihan_construction_site.sdf"
