#!/usr/bin/env bash
# Select native ROS/Ubuntu pairs. Optional function arguments support fixture tests.
niihan_select_platform() {
  local release_file="${1:-/etc/os-release}" architecture="${2:-$(uname -m)}"
  local ID VERSION_ID
  source "$release_file"
  [[ "$ID" == ubuntu && "$architecture" == x86_64 ]] || {
    echo 'Supported targets: Ubuntu 22.04/Humble and Ubuntu 24.04/Jazzy on amd64.' >&2;return 1;
  }
  case "$VERSION_ID" in
    22.04) NIIHAN_ROS_DISTRO=humble;NIIHAN_UBUNTU_CODENAME=jammy;NIIHAN_GZ_BRIDGE=ros-humble-ros-gzharmonic;;
    24.04) NIIHAN_ROS_DISTRO=jazzy;NIIHAN_UBUNTU_CODENAME=noble;NIIHAN_GZ_BRIDGE=ros-jazzy-ros-gz;;
    *) echo "Unsupported Ubuntu version: $VERSION_ID" >&2;return 1;;
  esac
  export NIIHAN_ROS_DISTRO NIIHAN_UBUNTU_CODENAME NIIHAN_GZ_BRIDGE
}
niihan_source_ros() {
  [[ -f "/opt/ros/$NIIHAN_ROS_DISTRO/setup.bash" ]] || {
    echo "Missing ROS $NIIHAN_ROS_DISTRO. Run ./setup.sh to install prerequisites." >&2;return 1;
  }
  source "/opt/ros/$NIIHAN_ROS_DISTRO/setup.bash"
}
