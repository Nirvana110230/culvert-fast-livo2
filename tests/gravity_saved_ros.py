"""Verify the actual full saved-map ROS payloads over two fresh headless openings."""
import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"runtime"))
from gravity import GRAVITY, read_json, sha256, write_json
from gravity_export import validate_map
from map_store import pcd_info, tum_rows
from saved_run_view import display, ros_stamp


def check_open(directory, result_path):
    import rclpy
    from nav_msgs.msg import Path as RosPath
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    manifest = validate_map(directory)
    expected = {}
    for kind in ("geometry", "rgb"):
        path = directory/f"map_{kind}.pcd"
        h = pcd_info(path)
        sha = hashlib.sha256()
        with path.open("rb") as f:
            f.seek(h.offset)
            for chunk in iter(lambda: f.read(1024*1024), b""):
                sha.update(chunk)
        expected[kind] = (h.count, sha.hexdigest())
    rows = list(tum_rows(directory/"trajectory.tum"))
    received = {}
    failed = []
    done = threading.Event()

    def observe():
        until = time.monotonic()+30
        while not rclpy.ok():
            if time.monotonic() > until:
                failed.append("ROS context unavailable")
                return
            time.sleep(.02)
        node = rclpy.create_node("read_full_archived_payload")
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def cloud(msg, kind):
            fact = msg.width*msg.height, hashlib.sha256(msg.data).hexdigest()
            assert msg.header.frame_id == GRAVITY and fact == expected[kind], (kind, fact)
            received[kind] = dict(frame=msg.header.frame_id, points=fact[0],
                                  bytes=len(msg.data), payload_sha256=fact[1])
        def trajectory(msg):
            assert msg.header.frame_id == GRAVITY and len(msg.poses) == len(rows)
            for pose, (stamp, p, q) in zip(msg.poses, rows):
                assert pose.header.frame_id == GRAVITY
                assert (pose.header.stamp.sec, pose.header.stamp.nanosec) == ros_stamp(stamp)
                xyz, quat = pose.pose.position, pose.pose.orientation
                assert [xyz.x,xyz.y,xyz.z] == p.tolist()
                assert [quat.x,quat.y,quat.z,quat.w] == q.tolist()
            received["trajectory"] = dict(frame=GRAVITY, poses=len(rows), times_and_full_poses_exact=True)
        def unexpected(msg):
            if msg.transforms:
                failed.append("offline viewer published a static TF")
        subscriptions = [
            node.create_subscription(PointCloud2, "/saved_experiment/rgb", lambda m: cloud(m,"rgb"), qos),
            node.create_subscription(PointCloud2, "/saved_experiment/geometry", lambda m: cloud(m,"geometry"), qos),
            node.create_subscription(RosPath, "/saved_experiment/path", trajectory, qos),
            node.create_subscription(TFMessage, "/tf_static", unexpected, qos)]
        try:
            while not done.is_set() and len(received) < 3 and not failed and rclpy.ok():
                executor.spin_once(timeout_sec=.05)
            if len(received) == 3:
                until = time.monotonic()+.3
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.03)
        except Exception as e:
            failed.append(repr(e))
        finally:
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            if len(received) == 3 or failed:
                os.kill(os.getpid(), signal.SIGINT)

    observer = threading.Thread(target=observe, daemon=True)
    observer.start()
    try:
        display(directory, True, 12)
    finally:
        done.set()
        observer.join(3)
    if failed or set(received) != {"rgb","geometry","trajectory"}:
        raise RuntimeError(f"full saved view failed: {failed}, topics={list(received)}")
    write_json(result_path, dict(received=received, static_tf_messages=0, manifest_counts=manifest["counts"]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--map", type=Path, required=True)
    p.add_argument("--report", type=Path, required=True)
    args = p.parse_args()
    directory = args.map.resolve()
    m = validate_map(directory)
    with tempfile.TemporaryDirectory(prefix="gravity-saved-") as tmp:
        openings = []
        for number in (1,2):
            result = Path(tmp)/f"opening-{number}.json"
            env = dict(os.environ, ROS_DOMAIN_ID=str(44+number), ROS_LOCALHOST_ONLY="1",
                       ROS_LOG_DIR=str(Path(tmp)/"roslog"), PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1")
            subprocess.run([sys.executable, str(Path(__file__).resolve()), "_open", str(directory), str(result)],
                           check=True, env=env, timeout=120)
            openings.append(read_json(result))
    assert openings[0]["received"] == openings[1]["received"]
    assert m == validate_map(directory)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.report, dict(verified_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                                ros_distro=os.environ.get("ROS_DISTRO"),
                                headless=True, subscriber_shares_local_dds_participant=True,
                                separate_fresh_viewer_processes=2, original_map_files_sha256_verified=True,
                                manifest_sha256=sha256(directory/"manifest.json"), openings=openings))
    print("PASS", args.report)


if __name__ == "__main__":
    if sys.argv[1:2] == ["_open"]:
        check_open(Path(sys.argv[2]), Path(sys.argv[3]))
    else:
        main()
