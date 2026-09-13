#!/usr/bin/env bash
set -euo pipefail

map_name="${1:-base_patrol}"
map_dir="${2:-$HOME/.ros/niihan_maps}"

mkdir -p "$map_dir"
ros2 run nav2_map_server map_saver_cli -f "$map_dir/$map_name"
printf 'Saved 2D map to %s.yaml and %s.pgm\n' "$map_dir/$map_name" "$map_dir/$map_name"