#!/usr/bin/env bash
# Sourced by supported entry points; local GLIM dependencies take precedence.
niihan_deps_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.deps"
niihan_prefix="$niihan_deps_root/install"
if [[ "${NIIHAN_ROS_DISTRO:-${ROS_DISTRO:-humble}}" == jazzy ]]; then
  niihan_prefix="$niihan_deps_root/jazzy/install"
fi
niihan_binary_lib="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.binary/deps/lib"
if [[ -d "$niihan_binary_lib" ]]; then
  export LD_LIBRARY_PATH="$niihan_binary_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [ -d "$niihan_prefix" ]; then
  export CMAKE_PREFIX_PATH="$niihan_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$niihan_prefix/lib:$niihan_prefix/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset niihan_prefix niihan_deps_root niihan_binary_lib
