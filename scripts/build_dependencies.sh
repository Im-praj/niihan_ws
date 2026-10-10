#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/platform.sh"
niihan_select_platform
source "$root/scripts/environment.sh"
deps="$root/.deps"
[[ "$NIIHAN_ROS_DISTRO" != jazzy ]] || deps="$deps/jazzy"
mkdir -p "$deps/src"
jobs="${NIIHAN_BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'NIIHAN_BUILD_JOBS must be a positive integer';exit 2; }
checkout() {
  local name="$1" url="$2" sha="$3" path="$deps/src/$1"
  if [[ ! -d "$path/.git" ]]; then git clone "$url" "$path"; fi
  if [[ -n "$(git -C "$path" status --porcelain)" ]]; then
    echo "Dependency source has changes: $path. Inspect before retrying.";exit 1
  fi
  git -C "$path" checkout --detach "$sha"
}
checkout gtsam https://github.com/borglab/gtsam.git 3ad4b4c3cb28394c9597f48fa02dad361c8450e3
cmake -S "$deps/src/gtsam" -B "$deps/build/gtsam" \
  -DCMAKE_INSTALL_PREFIX="$deps/install" -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$deps/build/gtsam" --parallel "$jobs"
cmake --install "$deps/build/gtsam"
source "$root/scripts/environment.sh"
checkout gtsam_points https://github.com/koide3/gtsam_points.git 9d32e7dbecf6015560d84b4901d6b0a6f483ec46
cmake -S "$deps/src/gtsam_points" -B "$deps/build/gtsam_points" \
  -DCMAKE_INSTALL_PREFIX="$deps/install" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTS=OFF
cmake --build "$deps/build/gtsam_points" --parallel "$jobs"
cmake --install "$deps/build/gtsam_points"
