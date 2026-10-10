#!/usr/bin/env bash
# Sourced by supported entry points; local GLIM dependencies take precedence.
niihan_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.deps/install"
if [ -d "$niihan_prefix" ]; then
  export CMAKE_PREFIX_PATH="$niihan_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$niihan_prefix/lib:$niihan_prefix/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset niihan_prefix
