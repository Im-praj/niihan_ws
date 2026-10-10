#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Start in a fresh shell; do not source another NIIHAN workspace first.
source "$root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/environment.sh"
cd "$root"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
git submodule update --init src/glim src/glim_ros2 src/navigation2 src/geometry2
if [[ "$NIIHAN_ROS_DISTRO" == humble ]]; then
"$root/scripts/apply_nav2_patch.sh"
colcon build --base-paths src/geometry2 --symlink-install --parallel-workers 1 --packages-up-to tf2_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
colcon build --base-paths src/navigation2 --symlink-install --parallel-workers 1 --packages-up-to nav2_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
else
  echo 'Jazzy uses native apt Nav2 and geometry2 binaries; Humble patches are not applied.'
fi
colcon build --base-paths src/glim src/glim_ros2 src/niihan_description src/niihan_dashboard src/niihan_bringup src/niihan_slam --symlink-install --parallel-workers 1 --packages-select glim glim_ros niihan_description niihan_dashboard niihan_bringup niihan_slam --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=OFF
