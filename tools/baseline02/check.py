"""Read-only B1 preflight, closed-shard and archive checks; append-only field events."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = Path("/home/orangepi/fast_livo2")
sys.path.insert(0, str(ROOT/"runtime"))
from run_files import ALGORITHM, boot_id, digest, latest_csv_row, read_json, same_process, snapshot
from hardware import CAMERA, DRIVER, require_free_pipeline

TAG = "stage-b1-20261127"
COMMIT = "082098ff69052e02397807094cdfa9c4a412b9e7"
ELF = "a4e4320cac0668987bf15a4a8199430c50a5a65cd008ef00aace1a960f70aeb2"
RESOLVED = "92c6a9b4d22e5b26873d43b62e3915d1e33dcc42fac0fdb5ede9250eb791bd2d"
TOPICS = ("/livox/lidar", "/livox/imu", "/hik_camera/image_timeshare",
          "/hik_camera/camera_info_timeshare", "/hardware_sync/camera_metadata",
          "/hardware_sync/camera_events", "/hardware_sync/camera_clock")
EVENTS = ("SESSION_START MOVE_START CHECKPOINT_P01 CHECKPOINT_P02 NEW_AREA_START "
          "REVISIT_START LIFT_START LIFT_END RETURN_P0 MOVE_END SESSION_END").split()


def command(*args):
    return subprocess.check_output(args, text=True, timeout=15).strip()


def preflight():
    release = read_json(ROOT/"records/stage_b1/deployment.json")
    with tempfile.TemporaryDirectory() as temp:
        params, manifest = snapshot(Path(temp), ROOT/"configs")
    elf = digest(ALGORITHM)
    checksums = {}
    mismatches = []
    for line in Path(__file__).with_name("b1-files.sha256").read_text().splitlines():
        expected, name = line.split(maxsplit=1)
        checksums[name] = digest(ROOT/name)
        if checksums[name] != expected:
            mismatches.append(name)
    source_matches = {p:digest(ROOT/p)==h for p,h in release["board_source_changes"].items()}
    identity = (release["tag"]==TAG and release["git_commit"]==COMMIT and
                elf==release["install_elf_sha256"]==ELF and
                manifest["resolved_sha256"]==release["resolved_sha256"]==RESOLVED and
                not mismatches and all(source_matches.values()))
    mount = command("findmnt", "-n", "-o", "TARGET,SOURCE,FSTYPE,OPTIONS", "-T", "/data").split()
    available = shutil.disk_usage("/data").free
    time_sync = command("timedatectl","show","-p","NTPSynchronized","--value")
    chrony = command("chronyc","-c","tracking").split(",")
    carrier = Path("/sys/class/net/enP3p49s0/carrier").read_text().strip()
    route = json.loads(command("ip","-j","-4","route","get","192.168.2.163"))[0]
    wired = carrier=="1" and route.get("dev")=="enP3p49s0" and route.get("prefsrc")=="192.168.2.100"
    radar = None
    errors = []
    active = read_json(ROOT/"state/active.json")
    busy = bool(active or (ROOT/"state/hardware_active.json").exists())
    try:
        require_free_pipeline(True)
    except RuntimeError as exc:
        busy = True
        errors.append(str(exc))
    bag_pids = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = path.read_bytes().split(b"\0")
        except OSError:
            continue
        if b"bag" in argv and b"record" in argv and any(Path(os.fsdecode(a)).name=="ros2" for a in argv):
            bag_pids.append(int(path.parent.name))
    if wired and not busy:
        try:
            radar = json.loads(command("python3", str(ROOT/"runtime/sensors/radar_device.py"), "status"))["before"]["values"]["current_mode"]
        except (ValueError, KeyError, subprocess.SubprocessError) as exc:
            errors.append(str(exc))
    camera = any((p/"idVendor").read_text().strip()=="2bdf" and
                 (p/"idProduct").read_text().strip()=="0001"
                 for p in Path("/sys/bus/usb/devices").iterdir() if (p/"idProduct").exists())
    gates = dict(b1_identity=identity, time_sync=time_sync=="yes" and chrony[-1]=="Normal",
                 nvme_rw=mount[:3]==["/data","/dev/nvme0n1p1","ext4"] and "rw" in mount[3].split(",")
                         and os.access("/data/robot/maps/fast_livo2/runs",os.W_OK),
                 space_110_GiB=available>=110*2**30, idle=not busy and not bag_pids,
                 camera_usb=camera, radar_route=wired, radar_idle=radar==2)
    return dict(wall_ns=time.time_ns(),mono_ns=time.monotonic_ns(),boot_id=boot_id(),
                gates=gates,ready=all(gates.values()),errors=errors,release=release,
                elf_sha256=elf,resolved_sha256=manifest["resolved_sha256"],keys=len(params),
                files_sha256=checksums,mismatches=mismatches,source_matches=source_matches,
                sensor_elf_sha256={p.name:digest(p) for p in (CAMERA,DRIVER)},
                mount=mount,available_bytes=available,
                planned_max_minutes=max(0,(available/2**30-25)*1024/(42.06*1.1*60)),
                chrony_leap=chrony[-1],radar_mode=radar,
                bag_pids=bag_pids,radar_route_device=route.get("dev"))


def mark(field, name, at, note):
    wall, mono = time.time_ns(), time.monotonic_ns()
    if at:
        stamp = datetime.fromisoformat(at.replace("Z","+00:00"))
        if stamp.utcoffset() is None:
            raise ValueError("--at requires an explicit UTC offset")
        delta = stamp.astimezone(timezone.utc)-datetime(1970,1,1,tzinfo=timezone.utc)
        event_ns = (delta.days*86400+delta.seconds)*10**9+delta.microseconds*1000
    else:
        event_ns = wall
    record = dict(event=name,source="phone_backfill" if at else "live",event_wall_ns=event_ns,
                  event_mono_ns=None if at else mono, recorded_wall_ns=wall,recorded_mono_ns=mono,
                  recorder_boot_id=boot_id(),session_boot_id=(field/"boot_id.txt").read_text().strip(),
                  supplied_time=at,note=note)
    with (field/"events.jsonl").open("a") as out:
        fcntl.flock(out,fcntl.LOCK_EX)
        out.write(json.dumps(record,ensure_ascii=False)+"\n")
        out.flush()
        os.fsync(out.fileno())
    return record


def shard_counts(path):
    if not path.is_file():
        raise FileNotFoundError(path)
    with sqlite3.connect(path.as_uri()+"?mode=ro&immutable=1", uri=True) as db:
        return dict(db.execute("SELECT t.name, count(*) FROM messages m JOIN topics t "
                               "ON t.id=m.topic_id GROUP BY t.name"))


def closed_counts(field):
    paths = sorted((field/"raw_bag").glob("raw_bag_*.db3"),key=lambda p:int(p.stem.rsplit("_",1)[1]))
    counts = Counter()
    for path in reversed(paths[-3:-1]):
        counts.update(shard_counts(path))
        if all(counts[t] for t in TOPICS):
            break
    return dict(closed_shard_counts=counts,seven_recorded=all(counts[t]>0 for t in TOPICS),
                shards=len(paths),db_bytes=sum(p.stat().st_size for p in paths))


def moving_gate(field):
    active = read_json(ROOT/"state/active.json")
    run = Path(active.get("run","."))
    if run.parent != field or not same_process(active):
        return dict(ready=False,reason="本轮 B1 尚未活动")
    def progress():
        row = latest_csv_row(run/"Log/processing.csv")
        mapping = read_json(run/"map/status.json")
        values = {k:int(row.get(k,0)) for k in ("lidar_rx","imu_rx","image_rx","synced","visual_update_frames")}
        values["poses"] = mapping.get("poses",0)
        values["bag_bytes"] = sum(p.stat().st_size for p in (field/"raw_bag").glob("*.db3"))
        return values
    before = progress()
    time.sleep(3)
    after = progress()
    archive = read_json(run/"map/status.json")
    args = read_json(run/"active_parameters.json")
    samples = field/"resources_2s.jsonl"
    # Inspect only the latest complete sample, even after a long disconnection.
    with samples.open("rb") as stream:
        stream.seek(max(0,stream.seek(0,2)-32768))
        sample = json.loads(stream.read().split(b"\n")[-2])
    counts = closed_counts(field)
    gates = dict(progress=all(after[k]>v for k,v in before.items()),
                 own_boot=sample["boot_id"]==boot_id()==(field/"boot_id.txt").read_text().strip(),
                 identity=read_json(run/"algorithm_identity.json").get("sha256")==ELF and
                          digest(run/"resolved.yaml")==RESOLVED and bool(args.get("parameters"))
                          and args.get("missing_required")==[] and args.get("mismatches")==[],
                 gravity=archive.get("gravity_ready") and archive.get("tf_verified"),
                 monitor=0 <= time.monotonic_ns()-sample["mono_ns"] < 8_000_000_000 and
                         {"algorithm","driver","camera","archive","recorder"}<=sample["processes"].keys(),
                 journal=(field/"journal.jsonl").stat().st_size>0,
                 seven_recorded=counts["seven_recorded"],
                 active=same_process(active) and not (field/"stop.request").exists())
    return dict(ready=all(gates.values()),gates=gates,before=before,after=after,bag=counts)


def products(field):
    run, = (p.parent for p in field.glob("????????T??????Z_*/session.json"))
    bag = field/"raw_bag"
    meta = yaml.safe_load((bag/"metadata.yaml").read_bytes())["rosbag2_bagfile_information"]
    listed = meta["relative_file_paths"]
    sizes = {p.name:p.stat().st_size for p in bag.glob("*.db3")}
    counts, per_shard = Counter(), {}
    for name in sizes:
        entries = shard_counts(bag/name)
        counts.update(entries)
        per_shard[name] = sum(entries.values())
    expected = {t["topic_metadata"]["name"]:t["message_count"] for t in meta["topics_with_message_count"]}
    session = read_json(run/"session.json")
    manifest = read_json(run/"map/manifest.json")
    transform = read_json(run/"gravity_transform.json")
    sensors = read_json(run/"sensors/session.json")
    radar = sensors.get("radar_idle",{})
    camera = sensors.get("camera",{})
    active_params = read_json(run/"active_parameters.json")
    procs = read_json(run/"processes.json")
    files = ("map_rgb.pcd","map_geometry.pcd","trajectory.tum")
    map_hashes = {name:digest(run/"map"/name) for name in files}
    ident = manifest.get("identity",{})
    checks = dict(bag_format=meta["storage_identifier"]=="sqlite3",
                  shards=sorted(sizes)==sorted(listed) and all(sizes.values()),
                  per_shard=per_shard=={f["path"]:f["message_count"] for f in meta["files"]},
                  messages=counts==expected and set(expected)==set(TOPICS) and
                           all(expected.values()) and sum(counts.values())==meta["message_count"],
                  identity=ident.get("session_id")==run.name and ident.get("algorithm_sha256")==ELF
                           and ident.get("resolved_sha256")==digest(run/"resolved.yaml")==RESOLVED,
                  effective_params=bool(active_params.get("parameters")) and
                      all(active_params["parameters"].get(k)==v for k,v in
                          yaml.safe_load((run/"resolved.yaml").read_bytes())["/**"]["ros__parameters"].items()),
                  gravity=manifest.get("output_frame")=="map_gravity" and manifest.get("gravity_applied")
                          and manifest.get("tf_check",{}).get("verified") and bool(manifest.get("transform"))
                          and transform.get("transform")==manifest["transform"] and transform.get("identity")==ident,
                  map_hashes=all(h==manifest["files"][n]["sha256"] for n,h in map_hashes.items()),
                  session=session["state"]=="complete" and not session["errors"] and
                          read_json(run/"map/result.json").get("status")=="complete",
                  radar=radar.get("state_and_wire_check") and radar.get("mode")==2 and
                        radar.get("wire",{}).get("passed") and radar["wire"]["counts"]=={"imu":0,"point":0},
                  camera=camera.get("exit_code")==0 and camera.get("settings_restored") and not sensors.get("errors"),
                  exited=all(k in procs for k in ("archive","algorithm","camera","driver")) and
                         all("returncode" in p and not p["forced"] and not same_process(p) for p in procs.values()))
    return dict(pass_all=all(checks.values()),checks=checks,run=str(run),topic_counts=counts,
                shard_counts=per_shard,db_bytes=sum(sizes.values()),map_sha256=map_hashes,
                process_exit={n:p.get("returncode") for n,p in procs.items()})


if __name__=="__main__":
    cli = argparse.ArgumentParser()
    sub = cli.add_subparsers(dest="action",required=True)
    sub.add_parser("preflight")
    for name in ("closed","products","event","ready"):
        p = sub.add_parser(name)
        p.add_argument("field",type=Path)
        if name=="event":
            p.add_argument("name",choices=EVENTS)
            p.add_argument("--at")
            p.add_argument("--note",default="")
    a = cli.parse_args()
    if a.action=="preflight":
        output = preflight()
    elif a.action=="event":
        output = mark(a.field,a.name,a.at,a.note)
    elif a.action=="products":
        output = products(a.field)
    elif a.action=="ready":
        output = moving_gate(a.field)
    else:
        output = closed_counts(a.field)
    print(json.dumps(output,indent=2,ensure_ascii=False))
    sys.exit(int(output.get("ready",output.get("pass_all",True)) is not True))
