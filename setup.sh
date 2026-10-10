#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  cat <<'HELP'
Usage: ./setup.sh [--check] [--no-system] [--use-system-deps] [--clean]
Default: install system dependencies (sudo), build pinned CPU dependencies locally,
build the simulation, validate it. Ubuntu 22.04 amd64 only.
--check            Read-only readiness checks; no installation/build.
--no-system        Skip apt installation; require prerequisites already installed.
--use-system-deps  Reuse existing GTSAM 4.3 and gtsam_points 1.2.2 instead of building.
--clean            Move old build/install/log into .setup/backups before rebuilding.
Logs are under .setup/logs. Never run this script with sudo.
HELP
  exit 0
fi
check=false;system=true;reuse=false;clean=false
for arg in "$@"; do
  case "$arg" in
    --check) check=true;; --no-system) system=false;;
    --use-system-deps) reuse=true;; --clean) clean=true;;
    *) echo "Unknown option: $arg (see ./setup.sh --help)" >&2;exit 2;;
  esac
done
if [[ "${NIIHAN_SETUP_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION \
    NIIHAN_SETUP_CLEAN_ENV=1 \
    bash --noprofile --norc "$root/setup.sh" "$@"
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || "$(uname -m)" != x86_64 ]]; then
  echo 'Supported platform: Ubuntu 22.04 amd64. This host is not supported.' >&2;exit 1
fi
if $check; then
  [[ -f /opt/ros/humble/setup.bash ]] || { echo 'Missing ROS Humble';exit 1; }
  source /opt/ros/humble/setup.bash
  source "$root/scripts/environment.sh"
  for tool in cmake git colcon node gcc xacro gz; do
    command -v "$tool" >/dev/null || { echo "Missing command: $tool";exit 1; }
  done
  python3 - <<'PY'
import importlib
from ament_index_python.packages import get_package_share_directory
for name in ['numpy','scipy','yaml','websockets','pytest','serial','cv2']:
    importlib.import_module(name)
for name in ['ros_gz_sim','ros_gz_bridge','rviz2','cv_bridge','sensor_msgs_py']:
    get_package_share_directory(name)
print('System prerequisites available.')
PY
  if [[ -f "$root/install/local_setup.bash" ]]; then
    source "$root/install/local_setup.bash"
    python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
for name in ['niihan_description','niihan_dashboard','niihan_bringup','niihan_slam','glim_ros','nav2_bringup']:
    print(name, get_package_share_directory(name))
PY
    echo 'Workspace package lookup passed. Run ./scripts/validate.sh for full checks.'
  else
    echo 'Workspace is not built. Run ./setup.sh';exit 1
  fi
  exit 0
fi
[[ $EUID != 0 ]] || { echo 'Run as your normal user; sudo is used only for apt.';exit 1; }
mkdir -p "$root/.setup/logs"
logfile="$root/.setup/logs/setup-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$logfile") 2>&1
trap 'code=$?; echo "Setup failed (exit $code). Log: $logfile"; exit "$code"' ERR
echo "Setup log: $logfile"
if $system; then
  sudo apt-get update
  sudo apt-get install -y --no-remove curl ca-certificates gnupg software-properties-common
  sudo add-apt-repository -y universe
  # Configure official repositories only if the relevant package has no candidate.
  if ! apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$root/.setup/ros.key"
    sudo install -m 644 "$root/.setup/ros.key" /usr/share/keyrings/niihan-ros-archive-keyring.gpg
    printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/niihan-ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main' | sudo tee /etc/apt/sources.list.d/niihan-ros2.list >/dev/null
  fi
  if ! apt-cache show gz-harmonic >/dev/null 2>&1; then
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o "$root/.setup/gazebo.gpg"
    sudo install -m 644 "$root/.setup/gazebo.gpg" /usr/share/keyrings/niihan-gazebo-archive-keyring.gpg
    printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/niihan-gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' | sudo tee /etc/apt/sources.list.d/niihan-gazebo.list >/dev/null
  fi
  sudo apt-get update
  sudo apt-get install -y --no-remove \
    build-essential cmake git pkg-config python3-colcon-common-extensions python3-rosdep \
    python3-numpy python3-scipy python3-yaml python3-aiohttp python3-websockets \
    python3-pytest python3-serial python3-opencv nodejs \
    libeigen3-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev libopencv-dev libomp-dev \
    ros-humble-ros-base ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-ros-gzharmonic gz-harmonic ros-humble-rviz2 ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
    ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-octomap-server \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-sensor-msgs-py \
    ros-humble-rosbag2 ros-humble-teleop-twist-keyboard \
    gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-plugins-good xdg-utils
fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS Humble is missing; rerun without --no-system.';exit 1; }
source /opt/ros/humble/setup.bash
source "$root/scripts/environment.sh"
git -C "$root" submodule update --init src/geometry2 src/glim src/glim_ros2 src/navigation2
if $clean; then
  backup="$root/.setup/backups/$(date +%Y%m%d-%H%M%S)"
  mkdir -p "$backup"
  for dir in build install log; do [[ ! -e "$root/$dir" ]] || mv "$root/$dir" "$backup/"; done
  echo "Previous build preserved in $backup"
fi
if ! $reuse; then
  "$root/scripts/build_dependencies.sh"
else
  echo 'Using existing system GTSAM/gtsam_points; CMake checks versions during build.'
fi
source "$root/scripts/environment.sh"
if $system; then
  [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
  rosdep update --rosdistro humble
  # Resolve only the supported dependency closure, excluding upstream system
  # tests that require Gazebo Classic (incompatible with this Harmonic profile).
  colcon --log-base "$root/.setup/logs/selection" list --base-paths \
    "$root/src/niihan_description" "$root/src/niihan_dashboard" \
    "$root/src/niihan_bringup" "$root/src/niihan_slam" "$root/src/glim" \
    "$root/src/glim_ros2" "$root/src/navigation2" "$root/src/geometry2" \
    --packages-up-to niihan_slam nav2_bringup tf2_ros --paths-only \
    > "$root/.setup/dependency-paths.txt"
  mapfile -t dependency_paths < "$root/.setup/dependency-paths.txt"
  [[ ${#dependency_paths[@]} -gt 0 ]]
  rosdep install --from-paths "${dependency_paths[@]}" \
    --ignore-src --rosdistro humble -y
fi
"$root/scripts/build_simulation.sh"
"$root/scripts/validate.sh"
printf '%s\n' "$(git -C "$root" rev-parse HEAD)" > "$root/.setup/validated-commit"
echo "Setup and validation passed. Launch with: $root/run.sh"
