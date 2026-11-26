#!/usr/bin/env bash
set -eo pipefail
mainline="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$mainline/env.sh"
if [[ ${1:-} == replay && -n ${FAST_LIVO2_REPLAY_DOMAIN:-} ]]; then
  [[ $FAST_LIVO2_REPLAY_DOMAIN =~ ^[0-9]{1,3}$ && 10#$FAST_LIVO2_REPLAY_DOMAIN -le 232 ]] || {
    echo "FAST_LIVO2_REPLAY_DOMAIN 需要 0 到 232 的整数" >&2
    exit 2
  }
  export ROS_DOMAIN_ID=$((10#$FAST_LIVO2_REPLAY_DOMAIN))
fi
exec python3 -u "$mainline/runtime/session.py" "$@"
