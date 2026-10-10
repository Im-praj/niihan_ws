#!/usr/bin/env bash
# Apply compatibility changes per file so prior patch revisions can be upgraded.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
patch="$root/patches/nav2-humble.patch"
paths="$(git -C "$root/src/navigation2" apply --numstat "$patch")"
while IFS=$'\t' read -r added removed path; do
  [[ -n "$path" ]] || continue
  if git -C "$root/src/navigation2" apply --include="$path" --check "$patch" 2>/dev/null; then
    git -C "$root/src/navigation2" apply --include="$path" "$patch"
  elif ! git -C "$root/src/navigation2" apply --include="$path" --reverse --check "$patch" 2>/dev/null; then
    echo "Nav2 file differs from its pinned compatibility patch: $path. Inspect changes before building." >&2
    exit 1
  fi
done <<< "$paths"
