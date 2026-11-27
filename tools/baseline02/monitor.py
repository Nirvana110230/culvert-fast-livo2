"""Board-local, subscriber-free sampling. Units and clocks are documented in the runbook."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


HZ = os.sysconf("SC_CLK_TCK")
BOOT = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
POLICIES = sorted(Path("/sys/devices/system/cpu/cpufreq").glob("policy*"))
ZONES = {p.joinpath("type").read_text().strip(): p for p in
         Path("/sys/class/thermal").glob("thermal_zone*")}


def read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def cpu_ticks():
    result = {}
    for line in Path("/proc/stat").read_text().splitlines():
        name, *values = line.split()
        if name.startswith("cpu"):
            ticks = list(map(int, values[:8]))  # guest time is already in user/nice
            result[name] = (sum(ticks), ticks[3], ticks[4])
    return result


def cpu_percent(now, before):
    result = {}
    for name, (total, idle, wait) in now.items():
        if name in before:
            old_total, old_idle, old_wait = before[name]
            dt = total - old_total
            if dt > 0:
                io = max(0, wait - old_wait)
                result[name] = dict(busy_pct=round(100 * (dt-idle+old_idle-io)/dt, 3),
                                    iowait_pct=round(100*io/dt, 3))
    return result


def kb_fields(path, wanted):
    fields = {}
    for line in (read(path) or "").splitlines():
        key, _, values = line.partition(":")
        if key in wanted:
            fields[key] = int(values.split()[0])
    return fields


def process_sample(pid, identity, previous, elapsed, pss):
    proc = Path("/proc") / str(pid)
    stat = read(proc / "stat")
    if not stat:
        return None
    fields = stat.rsplit(")", 1)[1].split()
    if fields[0] in ("Z", "X") or (identity and
            (identity["boot_id"] != BOOT or str(identity["ticks"]) != fields[19])):
        return None
    ticks = int(fields[11]) + int(fields[12])
    key = (pid, fields[19])
    row = dict(pid=pid, start_ticks=int(fields[19]), threads=int(fields[17]),
               cpu_ticks=ticks, cpu_pct=None)
    if key in previous and elapsed > 0:
        row["cpu_pct"] = round((ticks-previous[key])*100/HZ/elapsed, 3)
    row.update(kb_fields(proc/"status", {"VmRSS", "VmHWM"}))
    if pss:
        row.update(kb_fields(proc/"smaps_rollup", {"Pss"}))
    return key, ticks, row


def process_list(folder):
    result = {"monitor": (os.getpid(), None)}
    rec = read(folder/"recorder.pid")
    if rec and rec.isdecimal():
        pid = int(rec)
        try:
            argv = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if os.fsencode(folder/"raw_bag") in argv and b"record" in argv:
                result["recorder"] = (pid, None)
        except OSError:
            pass
    for path in folder.glob("????????T??????Z_*/processes.json"):
        for name, ident in json.loads(path.read_bytes()).items():
            if name in ("algorithm", "camera", "driver", "archive") and "returncode" not in ident:
                result[name] = (ident["pid"], ident)
    return result


def archive_counts(folder):
    for path in folder.glob("????????T??????Z_*/map/status.json"):
        with path.open() as stream:
            status = json.load(stream)
            fields = {k:status.get(k) for k in ("geometry_points","rgb_points","poses","gravity_ready",
                                               "frame","output_frame","tf_verified")}
            fields["status_file_mtime_ns"] = os.fstat(stream.fileno()).st_mtime_ns
            return fields
    return None


def smart():
    try:
        raw = subprocess.check_output(["sudo", "-n", "nvme", "smart-log", "/dev/nvme0",
                                       "-o", "json"], timeout=3, stderr=subprocess.STDOUT)
        data = json.loads(raw)
        return {k: data.get(k) for k in ("temperature", "critical_warning", "avail_spare",
                "percent_used", "media_errors", "num_err_log_entries", "unsafe_shutdowns",
                "data_units_written", "power_cycles", "warning_temp_time", "critical_comp_time")}
    except (subprocess.SubprocessError, OSError, ValueError) as exc:
        return {"error": str(exc)}


def main(folder):
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    clock = time.monotonic
    start = clock()
    prior_time, prior_cpu, prior_disk, prior_proc = start, {}, None, {}
    next_pss = next_smart = start
    samples = 0
    journal_args = ["journalctl", "--boot", "--since=-1 min", "--follow", "--no-pager",
                    "--output=json", "--output-fields=MESSAGE,PRIORITY,_SYSTEMD_UNIT,SYSLOG_IDENTIFIER",
                    "_TRANSPORT=kernel", "+", "_SYSTEMD_UNIT=NetworkManager.service", "+",
                    "_SYSTEMD_UNIT=chrony.service", "+", "_SYSTEMD_UNIT=systemd-udevd.service", "+",
                    "_SYSTEMD_UNIT=systemd-oomd.service", "+", "SYSLOG_IDENTIFIER=baseline02"]
    with (folder/"journal.jsonl").open("xb") as log, (folder/"resources_2s.jsonl").open("x") as out:
        journal = subprocess.Popen(journal_args, stdout=log)
        try:
            subprocess.run(["logger", "-t", "baseline02", f"monitor start {folder.name}"], check=True)
            while not stopping and not (folder/"monitor.stop").exists():
                now = clock()
                wall = time.time_ns()
                if journal.poll() is not None:
                    raise RuntimeError(f"journalctl exited {journal.returncode}")
                elapsed = now-prior_time
                cpu = cpu_ticks()
                disk = list(map(int, Path("/sys/class/block/nvme0n1/stat").read_text().split()))
                io = {}
                if prior_disk and elapsed > 0:
                    io = dict(write_MiB_s=round((disk[6]-prior_disk[6])*512/2**20/elapsed, 4),
                              util_pct=round((disk[9]-prior_disk[9])/10/elapsed, 3))
                row = dict(wall_ns=wall, mono_ns=round(now*1e9), boot_id=BOOT, interval_s=elapsed,
                           cpu=cpu_percent(cpu, prior_cpu), nvme=io,
                           disk_free_bytes=shutil.disk_usage(folder).free,
                           memory_KiB=kb_fields(Path("/proc/meminfo"), {"MemTotal","MemAvailable",
                               "Cached","Buffers","SwapTotal","SwapFree","Dirty","Writeback"}),
                           loadavg=Path("/proc/loadavg").read_text().split()[:3],
                           policy_kHz={p.name: {f:read(p/f) for f in ("affected_cpus",
                               "cpuinfo_cur_freq","scaling_cur_freq","scaling_max_freq")} for p in POLICIES},
                           thermal_mC={name:read(p/"temp") for name,p in ZONES.items()},
                           net={n:{k:read(Path("/sys/class/net")/n/k) for k in ("carrier","operstate",
                               "statistics/rx_errors","statistics/rx_dropped")} for n in ("enP3p49s0","wlan0")},
                           archive_counts=archive_counts(folder), processes={})
                current = {}
                for name,(pid,ident) in process_list(folder).items():
                    item = process_sample(pid, ident, prior_proc, elapsed, now >= next_pss)
                    if item:
                        key, ticks, row["processes"][name] = item
                        current[key] = ticks
                if now >= next_pss:
                    next_pss = now + 19
                if now >= next_smart:
                    row["smart"] = smart()
                    next_smart = now + 39
                out.write(json.dumps(row, separators=(",",":"))+"\n")
                out.flush()
                if samples % 5 == 0:
                    os.fsync(out.fileno())
                    os.fsync(log.fileno())
                samples += 1
                prior_time, prior_cpu, prior_disk, prior_proc = now, cpu, disk, current
                time.sleep(max(0, 2-(clock()-now)))
        finally:
            journal.terminate()
            journal.wait()
            os.fsync(out.fileno())
            os.fsync(log.fileno())
    print(json.dumps(dict(samples=samples, elapsed_s=clock()-start, boot_id=BOOT)), flush=True)


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve(strict=True))
