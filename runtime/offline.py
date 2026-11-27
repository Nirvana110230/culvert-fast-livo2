"""Run the production mapper on a Livox ROS 2 bag at the estimator's own pace."""
import argparse
import csv
import json
import os
import resource
import shutil
import subprocess
import time
from pathlib import Path

import yaml

from gravity import GravityTransform
from run_files import ALGORITHM, ROOT, digest, read_json, snapshot, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path, help="new NVMe session directory")
    parser.add_argument("--params", type=Path, help="recorded resolved.yaml")
    parser.add_argument("--imu-noise", type=float, nargs=2, metavar=("ACC", "GYR"))
    parser.add_argument("--lio", action="store_true", help="scan-clock LIO, without the camera")
    parser.add_argument("--strict-windows", action="store_true", help="use half-open image time windows")
    args = parser.parse_args()
    run = args.output.resolve()
    bag = args.bag.resolve(strict=True)
    if not (bag / "metadata.yaml").is_file():
        parser.error("expected a bag directory with metadata.yaml")
    if args.imu_noise and not all(0 < v < float("inf") for v in args.imu_noise):
        parser.error("noise parameters must be positive finite values")
    run.mkdir(parents=True, exist_ok=False)
    (run / "keyframes").mkdir()
    (run / "Log/result").mkdir(parents=True)
    config = run / "resolved.yaml"
    if args.params:
        shutil.copyfile(args.params, config)
    else:
        snapshot(run, ROOT / "configs")
    if args.imu_noise or args.lio:
        cfg = yaml.safe_load(config.read_bytes())
        params = cfg["/**"]["ros__parameters"]
        if args.imu_noise:
            params["imu.acc_cov"], params["imu.gyr_cov"] = args.imu_noise
        if args.lio:
            params["common.img_en"] = 0
        config.write_text(yaml.safe_dump(cfg, sort_keys=True), encoding="utf-8")
    identity = dict(session_id=run.name, executable=str(ALGORITHM), sha256=digest(ALGORITHM))
    write_json(run / "algorithm_identity.json", identity)
    config_id = digest(config)
    env = dict(os.environ, FAST_LIVO2_OFFLINE_BAG=str(bag), FAST_LIVO_KEYFRAMES=str(run / "keyframes"),
               FAST_LIVO2_SESSION_ID=run.name, FAST_LIVO2_GRAVITY_DIR=str(run),
               FAST_LIVO2_CONFIG_SHA256=config_id, FAST_LIVO2_ALGORITHM_SHA256=identity["sha256"],
               ROS_DOMAIN_ID=os.getenv("FAST_LIVO2_REPLAY_DOMAIN", "216"), MALLOC_ARENA_MAX="2")
    if args.strict_windows:
        env["FAST_LIVO_STRICT_WINDOWS"] = "1"
    manifest = dict(bag=str(bag), config_sha256=config_id, config_source=str(args.params or ROOT / "configs"),
                    imu_noise=args.imu_noise, lio=args.lio, strict_windows=env.get("FAST_LIVO_STRICT_WINDOWS") == "1",
                    input_order="rosbag2 sequential reader")
    write_json(run / "offline.json", manifest)
    started = time.monotonic()
    command = [str(ALGORITHM), "--ros-args", "--disable-external-lib-logs", "--params-file", str(config)]
    big_cores = sorted(set(range(4, 8)).intersection(os.sched_getaffinity(0)))
    if big_cores:
        command = ["taskset", "-c", ",".join(map(str, big_cores)), *command]
    with (run / "algorithm.log").open("wb") as log, (run / "resources.csv").open("w") as usage:
        rows = csv.writer(usage)
        rows.writerow(["elapsed_s", "rss_kib", "pss_kib"])
        with subprocess.Popen(command, cwd=run, env=env, stdout=log, stderr=log) as process:
            try:
                while process.poll() is None:
                    try:
                        rollup = Path(f"/proc/{process.pid}/smaps_rollup").read_text().splitlines()
                        memory = {s.split()[0]: int(s.split()[1]) for s in rollup if s.startswith(("Rss:", "Pss:"))}
                        if memory:
                            rows.writerow([round(time.monotonic()-started, 3), memory["Rss:"], memory["Pss:"]])
                            usage.flush()
                    except (FileNotFoundError, ProcessLookupError):
                        pass
                    try:
                        process.wait(2)
                    except subprocess.TimeoutExpired:
                        pass
            except KeyboardInterrupt:
                process.terminate()
            code = process.wait()
    manifest.update(exit_code=code, elapsed_s=time.monotonic()-started,
                    peak_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
                    counts=read_json(run / "Log/input_counters.json"))
    write_json(run / "offline.json", manifest)
    if code:
        raise SystemExit(f"Mapper exited with {code}, see {run / 'algorithm.log'}")
    event = json.loads((run / "gravity_events.jsonl").read_text().splitlines()[0])
    gravity = GravityTransform(event["gravity_camera_init_m_s2"])
    write_json(run / "gravity_transform.json", {"transform": gravity.record()})
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
