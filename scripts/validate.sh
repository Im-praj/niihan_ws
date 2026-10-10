#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/environment.sh"
source "$workspace_root/install/local_setup.bash"
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_root/.validation/ros-log}"
export PYTHONPATH="$workspace_root/src/niihan_description:$workspace_root/src/niihan_dashboard:$workspace_root/src/niihan_bringup:$workspace_root/src/niihan_slam:${PYTHONPATH:-}"
# Only built-in pytest plugins are needed; user-installed entry points may target
# a different pytest version (for example anyio versus Ubuntu's system pytest).
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider "$workspace_root/src/niihan_description/test" "$workspace_root/src/niihan_dashboard/test" "$workspace_root/src/niihan_bringup/test" "$workspace_root/src/niihan_slam/test"
node "$workspace_root/src/niihan_dashboard/test/test_web_viewer2d.js"
xacro "$workspace_root/src/niihan_description/urdf/niihan.urdf.xacro" > /dev/null
gz sdf -k "$workspace_root/src/niihan_description/worlds/niihan_construction_site.sdf"

gz sdf -k "$workspace_root/src/niihan_description/worlds/niihan_portable_site.sdf"
xacro "$workspace_root/src/niihan_description/urdf/niihan.urdf.xacro" minimal_sensors:=true > /dev/null
test -s "$workspace_root/install/niihan_dashboard/share/niihan_dashboard/web/vendor/three.min.js"

core_test="$(mktemp /tmp/niihan-core-test.XXXXXX)"
trap 'rm -f "$core_test"' EXIT
gcc -std=c11 -Wall -Wextra -Werror "$workspace_root/src/niihan_bringup/firmware/niihan_core.c" "$workspace_root/src/niihan_bringup/firmware/test_core.c" -lm -o "$core_test"
"$core_test"
