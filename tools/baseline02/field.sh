#!/usr/bin/env bash
# All acquisition and finalization run on the board. Supply the printed FIELD after reconnecting.
set -eo pipefail
umask 077
HERE=$(cd -- "$(dirname -- "$0")" && pwd -P)
SELF=$HERE/field.sh
ROOT=/home/orangepi/fast_livo2
BASE=/data/robot/maps/fast_livo2/runs
socket=fast-livo2-field
tm() { tmux -L "$socket" -f /dev/null "$@"; }
has() { tm has-session -t "=$1" 2>/dev/null; }
check() { python3 -B "$HERE/check.py" "$@"; }
die() { echo "$*" >&2; exit 1; }

case "${1:-}" in
    preflight) check preflight; exit ;;
    start) exec flock -n -o "/run/user/$(id -u)/baseline02-start.lock" "$SELF" _create ;;
    _create)
        if tm list-sessions -F '#{session_name}' 2>/dev/null | grep -E '^b02_' >/dev/null; then
            die "已有 Baseline 02 作业，请先核对状态。"
        fi
        tmp=$(mktemp)
        trap 'rm -f -- "$tmp"' EXIT
        if ! check preflight >"$tmp"; then cat "$tmp" >&2; die "启测前检查未通过。"; fi
        field=$(mktemp -d "$BASE/corridor_baseline02_$(date -u +%Y%m%dT%H%M%SZ)_XXXXXX")
        cp "$tmp" "$field/preflight.json"
        cat /proc/sys/kernel/random/boot_id >"$field/boot_id.txt"
        (cd "$HERE" && sha256sum field.sh monitor.py check.py b1-files.sha256) >"$field/tools.sha256"
        echo "$field"
        tm new-session -d -s "b02_map_${field##*/}" "$SELF" _map "$field"
        exit ;;
esac

action=${1:-}
field=${2:-}
[[ "$field" =~ ^/data/robot/maps/fast_livo2/runs/corridor_baseline02_[0-9]{8}T[0-9]{6}Z_[a-zA-Z0-9]{6}$ &&
   -f "$field/boot_id.txt" && "$(realpath -- "$field")" == "$field" ]] || die "需要本轮完整 FIELD 路径。"
id=${field##*/}
bag=b02_bag_$id
mon=b02_mon_$id
map=b02_map_$id
stop=b02_stop_$id
shift 2
case "$action" in
    event) check event "$field" "$@"; exit ;;
    ready) check ready "$field"; exit ;;
    check) check products "$field"; exit ;;
    status)
        echo "FIELD=$field"
        [[ "$(< /proc/sys/kernel/random/boot_id)" == "$(< "$field/boot_id.txt")" ]] || echo 'Boot ID 已改变'
        for job in "$map" "$bag" "$mon" "$stop"; do
            if has "$job"; then echo "$job active"; else echo "$job inactive"; fi
        done
        "$ROOT/status.sh" --json
        check closed "$field"
        tail -n 1 "$field/resources_2s.jsonl" 2>/dev/null || true
        for file in lifecycle.log recorder.exit monitor.exit final.status; do
            if [[ -f "$field/$file" ]]; then tail -n 3 "$field/$file"; fi
        done
        exit ;;
esac
[[ "$(< /proc/sys/kernel/random/boot_id)" == "$(< "$field/boot_id.txt")" ]] || die "启动周期已改变，保留原目录供恢复。"

case "$action" in
    _record)
        exec >"$field/recorder.log" 2>&1
        source "$ROOT/env.sh"
        mapfile -t topics < <(PYTHONPATH="$HERE" python3 -B -c 'from check import TOPICS;print(*TOPICS,sep="\n")')
        [[ ${#topics[@]} == 7 ]]
        # The foreground child owns the recorder. Keep the parent for its actual exit code.
        trap ':' INT
        rc=0
        bash -c 'echo "$$" >"$1"; shift; exec "$@"' _ "$field/recorder.pid" \
            ros2 bag record -s sqlite3 -b 536870912 -p 100 -o "$field/raw_bag" "${topics[@]}" || rc=$?
        echo "$rc" >"$field/recorder.exit"
        exit "$rc" ;;
    _monitor)
        rc=0
        nice -n 10 python3 -B "$HERE/monitor.py" "$field" >"$field/monitor.log" 2>&1 || rc=$?
        echo "$rc" >"$field/monitor.exit"
        exit "$rc" ;;
    _map)
        exec >>"$field/lifecycle.log" 2>&1
        finish() {
            local rc=$? bagrc monrc
            trap - EXIT
            set +e
            if has "$bag"; then
                tm send-keys -t "=$bag:0.0" C-c
                while has "$bag"; do sleep 1; done
            fi
            touch "$field/monitor.stop"
            while has "$mon"; do sleep 1; done
            check event "$field" SESSION_END > /dev/null || rc=1
            bagrc=$(cat "$field/recorder.exit" 2>/dev/null)
            monrc=$(cat "$field/monitor.exit" 2>/dev/null)
            [[ ( "$bagrc" == 0 || "$bagrc" == 130 ) && "$monrc" == 0 ]] || rc=1
            grep -q 'Recording stopped' "$field/recorder.log" || rc=1
            local runs=("$field"/????????T??????Z_*)
            if [[ ${#runs[@]} == 1 && -f "${runs[0]}/session.json" ]]; then
                "$ROOT/view.sh" --run "${runs[0]}" --check >"$field/map_check.log" 2>&1 || rc=1
            else rc=1; fi
            (source "$ROOT/env.sh" && ros2 bag info "$field/raw_bag") >"$field/bag_info.log" 2>&1 || rc=1
            check products "$field" >"$field/integrity.json" || rc=1
            echo "$rc" >"$field/final.status"
            echo "final.status=$rc $(date -Is)"
            exit "$rc"
        }
        trap finish EXIT
        check event "$field" SESSION_START > /dev/null
        tm new-session -d -s "$mon" "$SELF" _monitor "$field"
        for ((i=0;i<12;i++)); do
            [[ -s "$field/resources_2s.jsonl" && -s "$field/journal.jsonl" ]] && break
            has "$mon" || die "监测未启动"
            sleep 1
        done
        [[ -s "$field/resources_2s.jsonl" && -s "$field/journal.jsonl" ]] || die "监测未落盘"
        [[ ! -e "$field/stop.request" ]] || exit 1
        tm new-session -d -s "$bag" "$SELF" _record "$field"
        for ((i=0;i<20;i++)); do
            grep -Eq 'Listening for topics|Recording\.\.\.' "$field/recorder.log" 2>/dev/null && break
            has "$bag" || die "录包未启动"
            sleep 1
        done
        grep -Eq 'Listening for topics|Recording\.\.\.' "$field/recorder.log" || die "录包未监听"
        [[ ! -e "$field/stop.request" ]] || exit 1
        "$ROOT/start.sh" --runs-dir "$field" >"$field/start.log" 2>&1
        ;;
    stop)
        if [[ -f "$field/final.status" ]]; then cat "$field/final.status"; exit; fi
        has "$map" || die "找不到本轮建图托管作业，请核对 status 和正式入口状态。"
        touch "$field/stop.request"
        if ! has "$stop"; then tm new-session -d -s "$stop" "$SELF" _stop "$field"; fi
        echo "已提交板端正常收尾，等待 final.status。" ;;
    _stop)
        exec >>"$field/stop.log" 2>&1
        while has "$map"; do
            current=$(python3 -B -c 'import json,sys;from pathlib import Path;p=Path(sys.argv[1]);print(json.loads(p.read_bytes())["run"] if p.exists() else "")' "$ROOT/state/active.json")
            if [[ -n "$current" && "$(dirname -- "$current")" == "$field" ]]; then
                "$ROOT/stop.sh"
                break
            fi
            sleep 1
        done ;;
    *) die "用法: field.sh preflight|start 或 field.sh status|ready|event|stop|check FIELD ..." ;;
esac
