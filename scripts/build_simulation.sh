#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Start in a fresh shell; do not source another NIIHAN workspace first.
source /opt/ros/humble/setup.bash
cd "$root"
git submodule update --init src/glim src/glim_ros2
colcon build --symlink-install --parallel-workers 1 --packages-select glim glim_ros niihan_description niihan_dashboard niihan_bringup niihan_slam --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=OFF
