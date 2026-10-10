#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: ./run.sh [ROS launch arguments, e.g. seed:=42]'
  echo 'Launches visible Gazebo, RViz and dashboard. Headless mode is not supported here.';exit 0
fi
if [[ "${NIIHAN_RUN_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION \
    NIIHAN_RUN_CLEAN_ENV=1 \
    bash --noprofile --norc "$root/run.sh" "$@"
fi
[[ -n "${DISPLAY:-}" ]] || { echo 'No DISPLAY. Run from a graphical desktop session.' >&2;exit 1; }
[[ -f "$root/install/local_setup.bash" ]] || {
  echo 'Workspace not built. Run ./setup.sh first.' >&2;exit 1;
}
for arg in "$@"; do
  case "$arg" in headless:=*|rviz:=*) echo 'run.sh keeps Gazebo and RViz visible; remove that override.';exit 2;; esac
done
source "$root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
if [[ -f "$root/.setup/build-profile" ]] && [[ "$(cat "$root/.setup/build-profile")" != "$NIIHAN_ROS_DISTRO" ]]; then
  echo 'Build profile differs from the native ROS version. Rerun ./setup.sh --clean.' >&2;exit 1
fi
source "$root/scripts/environment.sh"
source "$root/install/local_setup.bash"
python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
for name in ['niihan_slam','niihan_dashboard','glim_ros','ros_gz_sim','nav2_bringup']:
    get_package_share_directory(name)
PY
python3 - <<'PY'
import socket
for port in (8080,8081):
    with socket.socket() as s:
        try:s.bind(('127.0.0.1',port))
        except OSError:raise SystemExit(f'Port {port} is occupied. Stop the previous dashboard before launching.')
PY
if [[ -f "$root/.setup/validated-commit" ]] && [[ "$(cat "$root/.setup/validated-commit")" != "$(git -C "$root" rev-parse HEAD)" ]]; then
  echo 'Source changed since setup validation. Rerun ./setup.sh to validate the new revision.' >&2
fi
mkdir -p "$root/.setup/logs"
export ROS_LOG_DIR="$root/.setup/logs/ros-$(date +%Y%m%d-%H%M%S)"
echo 'Dashboard: http://localhost:8080 — Ctrl+C stops the stack.'
# Open only after HTTP is responding; no browser download or global process kill.
if command -v xdg-open >/dev/null; then
  (for attempt in $(seq 1 90); do
    if curl -fsS http://127.0.0.1:8080/ >/dev/null 2>&1; then
      xdg-open http://localhost:8080 >/dev/null 2>&1;break
    fi
    sleep 1
  done) &
  browser_wait=$!
  trap 'kill "$browser_wait" 2>/dev/null || true' EXIT
fi
ros2 launch niihan_slam slam_simulation.launch.py headless:=false rviz:=true "$@"
