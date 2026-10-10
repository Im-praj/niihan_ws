#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/platform.sh"
niihan_select_platform
niihan_source_ros
source "$root/scripts/environment.sh"
source "$root/install/local_setup.bash"
python3 - <<'PY'
import importlib
from ament_index_python.packages import get_package_share_directory
for name in ['niihan_description','niihan_bringup','niihan_dashboard','niihan_slam','glim_ros','nav2_bringup']:
    print(name,get_package_share_directory(name))
for name in ['niihan_slam.map_bridge','niihan_slam.saved_localizer','niihan_dashboard.ros_bridge']:
    importlib.import_module(name)
PY
ros2 pkg executables glim_ros | rg -q '^glim_ros glim_rosnode$'
xacro "$root/install/niihan_description/share/niihan_description/urdf/niihan.urdf.xacro" minimal_sensors:=true >/dev/null
# ELF loader checks, not a substitute for motion/SLAM acceptance.
while IFS= read -r -d '' library; do
  if ldd "$library" 2>/dev/null | rg -q '=> not found'; then
    echo "Unresolved runtime library: $library" >&2;exit 1
  fi
done < <(find "$root/install" "$root/.binary/deps" -type f -name '*.so*' -print0)
echo 'Binary runtime package/import/link checks passed.'
