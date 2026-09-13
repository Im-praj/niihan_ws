#!/usr/bin/env bash
set -euo pipefail

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard "$@"