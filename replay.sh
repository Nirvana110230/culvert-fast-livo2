#!/usr/bin/env bash
if [[ ${1:-} == --pgo || ${1:-} == --structure ]]; then
  root="$(cd -- "$(dirname -- "$0")" && pwd -P)"
  program=run.py
  if [[ $1 == --structure ]]; then program=structure_run.py; fi
  shift
  export PYTHONPATH="${FAST_LIVO2_PGO_PREFIX:-$root/backend/pydeps}${PYTHONPATH:+:$PYTHONPATH}"
  export OPENBLAS_NUM_THREADS=1
  exec python3 "$root/backend/$program" "$@"
fi
if [[ ${1:-} == --offline ]]; then
  root="$(cd -- "$(dirname -- "$0")" && pwd -P)"
  source "$root/env.sh" || exit 1
  shift
  exec python3 "$root/runtime/offline.py" "$@"
fi
exec bash "$(dirname -- "$0")/runtime/entry.sh" replay "$@"
