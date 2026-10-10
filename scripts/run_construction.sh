#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
source "$workspace_root/install/setup.bash"
exec ros2 launch niihan_description niihan_full_system.launch.py world:=niihan_construction_site.sdf seed:=42 "$@"
