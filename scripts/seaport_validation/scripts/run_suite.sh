#!/usr/bin/env bash
set -eo pipefail
seaport_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$seaport_script_dir/../../.." && pwd)"
source "$workspace_root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
source "$workspace_root/scripts/environment.sh"
source "$workspace_root/install/local_setup.bash"
exec python3 "$seaport_script_dir/run_suite.py" "$@"
