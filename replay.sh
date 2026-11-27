#!/usr/bin/env bash
if [[ ${1:-} == --offline ]]; then
  root="$(cd -- "$(dirname -- "$0")" && pwd -P)"
  source "$root/env.sh" || exit 1
  shift
  exec python3 "$root/runtime/offline.py" "$@"
fi
exec bash "$(dirname -- "$0")/runtime/entry.sh" replay "$@"
