"""Real ROS 2/DDS + tf2 check with synthetic native estimator output in isolated domains."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"runtime"))
from gravity import BODY, CAMERA, GRAVITY, read_json, sha256, write_json
from gravity_export import validate_map
from map_store import pcd_info


def wait(predicate, desc, timeout=20):
    until = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > until:
            raise RuntimeError("timeout: " + desc)
        time.sleep(.03)


def shutdown_process(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(8)
        except subprocess.TimeoutExpired:
            p.terminate()
            p.wait(5)


class NativeTopics:
    Q = [.2, .05, -.1, math.sqrt(1-.2**2-.05**2-.1**2)]

    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        self.node = rclpy.create_node("native_gravity_test_messages")
        self.pubs = [self.node.create_publisher(PointCloud2, topic, 8) for topic in
                     ("/cloud_registered", "/cloud_effected")]
        self.odom = self.node.create_publisher(Odometry, "/aft_mapped_to_init", 8)

    def sample(self, i):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2, PointField
        for which, pub in enumerate(self.pubs):
            msg = PointCloud2()
            msg.header.frame_id = CAMERA
            msg.header.stamp.sec, msg.header.stamp.nanosec = 1700000000+i, 123456789
            names = ["x", "y", "z", "rgb" if which == 0 else "intensity"]
            msg.fields = [PointField(name=n, offset=4*j, datatype=6 if n == "rgb" else 7, count=1)
                          for j, n in enumerate(names)]
            msg.height, msg.width, msg.point_step, msg.row_step, msg.is_dense = 1, 4, 16, 64, True
            kind = "<fffI" if which == 0 else "<ffff"
            msg.data = b"".join(struct.pack(kind, i+dx, 2., -3., 0xff00ee+i+j if which == 0 else 77.+i+j)
                                for j, dx in enumerate((0., 0., .02, .5)))
            pub.publish(msg)
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = CAMERA, BODY
        msg.header.stamp.sec, msg.header.stamp.nanosec = 1700000000+i, 123456789
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y, p.z = float(i), 2., -3.
        q.x, q.y, q.z, q.w = self.Q
        self.odom.publish(msg)


def lookup_after_attach():
    import rclpy
    from tf2_ros import Buffer, TransformListener
    node = rclpy.create_node("attach_gravity_consumer")
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    buf = Buffer()
    listener = TransformListener(buf, node)
    try:
        until = time.monotonic() + 8
        while not buf.can_transform(GRAVITY, CAMERA, rclpy.time.Time()):
            if time.monotonic() > until:
                raise RuntimeError("late joining tf2 subscriber missed session TF")
            executor.spin_once(timeout_sec=.05)
        q = buf.lookup_transform(GRAVITY, CAMERA, rclpy.time.Time()).transform.rotation
        return [q.x, q.y, q.z, q.w]
    finally:
        listener.unregister()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()


def reopen_rviz(node, run):
    checks = []
    for i in range(2):
        with (run/f"rviz-{i}.log").open("w") as log:
            env = dict(os.environ, QT_QPA_PLATFORM="offscreen", LIBGL_ALWAYS_SOFTWARE="1")
            rviz = subprocess.Popen(["rviz2", "-d", str(ROOT/"runtime/continuous.rviz")], env=env,
                                    stdout=log, stderr=subprocess.STDOUT)
            try:
                def subscribed():
                    if rviz.poll() is not None:
                        log.flush()
                        raise RuntimeError((run/f"rviz-{i}.log").read_text()[-900:])
                    return any(e.node_name.startswith("rviz") for e in node.get_subscriptions_info_by_topic("/path"))
                wait(subscribed, "RViz display subscription")
                checks.append(dict(opened=True, path_subscription=True))
            finally:
                shutdown_process(rviz)
            wait(lambda: not any(e.node_name.startswith("rviz") for e in node.get_subscriptions_info_by_topic("/path")),
                 "RViz closes")
            checks[-1]["closed"] = True
    return checks


def embedded_archive(run):
    """Separate real ROS publishers share the archive's DDS participant in this fixture."""
    from archive import main as archive_main
    spec = read_json(run/"fixture_spec.json")
    def produce():
        native = None
        try:
            wait(lambda: (run/"map/ready").exists(), "ROS archive ready")
            native = NativeTopics()
            wait(lambda: all(p.get_subscription_count() for p in [native.odom,*native.pubs]),
                 "same participant subscribers")
            native.sample(0)
            wait(lambda: (run/"gravity_tf_check.json").is_file(), "actual tf2 sample")
            clients = [lookup_after_attach(), lookup_after_attach()] if spec["reopen"] else []
            rviz = reopen_rviz(native.node, run) if spec["use_rviz"] else []
            for i in range(1,5):
                native.sample(i)
                time.sleep(.08)
            write_json(run/"fixture_messages.json", dict(clients=clients, rviz=rviz))
        except Exception as exc:
            write_json(run/"fixture_error.json", dict(error=repr(exc)))
        finally:
            if native:
                native.node.destroy_node()
    worker = threading.Thread(target=produce, daemon=True)
    worker.start()
    try:
        return archive_main(run)
    finally:
        worker.join(1)


def run_case(tmp, emitter, name, gravity, domain, reopen, use_rviz, shared):
    from scipy.spatial.transform import Rotation
    run = tmp/name
    run.mkdir()
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(run/"ros_logs"),
                      PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1")
    import rclpy
    cfg = run/"resolved.yaml"
    cfg.write_text("/**:\n  ros__parameters:\n    uav.gravity_align_en: false\n")
    write_json(run/"parameters_manifest.json", dict(run=str(run), resolved_sha256=sha256(cfg)))
    write_json(run/"algorithm_identity.json", dict(session_id=name, executable=str(emitter), sha256=sha256(emitter)))
    env = dict(os.environ, FAST_LIVO2_SESSION_ID=name, FAST_LIVO2_GRAVITY_DIR=str(run),
               FAST_LIVO2_CONFIG_SHA256=sha256(cfg), FAST_LIVO2_ALGORITHM_SHA256=sha256(emitter))
    connected = []
    actual_rviz = []
    arch = None
    fixture = None
    native = None
    if not shared:
        rclpy.init(domain_id=domain)
    try:
        with (run/"archive.log").open("w") as log:
            write_json(run/"fixture_spec.json", dict(reopen=reopen, use_rviz=use_rviz))
            command = ([sys.executable, "-u", str(Path(__file__).resolve()), "_embedded_archive", str(run)]
                       if shared else [sys.executable, "-u", str(ROOT/"runtime/archive.py"), str(run)])
            arch = subprocess.Popen(command, cwd=run,
                                    env=env, stdout=log, stderr=subprocess.STDOUT)
            def ready():
                if arch.poll() is not None:
                    raise RuntimeError((run/"archive.log").read_text()[-3000:])
                if (run/"fixture_error.json").is_file():
                    raise RuntimeError(str(read_json(run/"fixture_error.json")))
                return (run/"map/ready").exists()
            wait(ready, "archive ready")
            if not shared:
                native = NativeTopics()
                wait(lambda: all(p.get_subscription_count() for p in [native.odom,*native.pubs]),
                     "native subscribers")
                native.sample(0)
            def has_pending():
                ready()
                p = run/"map/status.json"
                return p.is_file() and read_json(p)["poses"] == 1
            wait(has_pending, "preinit raw")
            fixture = subprocess.Popen([str(emitter), *map(str, gravity)], cwd=run, env=env,
                                       stdin=subprocess.PIPE, text=True)
            def tf_ready():
                if arch.poll() is not None:
                    raise RuntimeError((run/"archive.log").read_text()[-3000:])
                return (run/"gravity_tf_check.json").is_file()
            wait(tf_ready, "tf2 proof")
            if shared:
                wait(lambda: ready() and (run/"fixture_messages.json").exists(), "local DDS test messages", 45)
                probe = read_json(run/"fixture_messages.json")
                connected, actual_rviz = probe["clients"], probe["rviz"]
            else:
                if reopen:
                    connected = [lookup_after_attach(), lookup_after_attach()]
                    if use_rviz:
                        actual_rviz = reopen_rviz(native.node, run)
                for i in range(1, 5):
                    native.sample(i)
                    time.sleep(.07)
            fixture.communicate(input="reset\n" if reopen else "stop\n", timeout=5)
            assert fixture.returncode == (2 if reopen else 0)
            arch.send_signal(signal.SIGINT)
            ret = arch.wait(25)
        if ret:
            raise RuntimeError((run/"archive.log").read_text()[-3000:])
        result = read_json(run/"map/result.json")
        manifest = validate_map(run/"map")
        assert result["status"] == "complete" and result["error"] is None
        assert not (run/"raw").exists() and not (run/"in_progress").exists()
        assert manifest["counts"] == dict(rgb_points=20, geometry_points=20, poses=5)
        assert result["segment_end_reason"] == ("reinitialization_requested" if reopen else None)
        unit_g = np.asarray(gravity)/np.linalg.norm(gravity)
        target = np.array([0., 0., -1.])
        axis = np.cross(unit_g, target)
        angle = math.atan2(np.linalg.norm(axis), np.dot(unit_g, target))
        tr = Rotation.from_rotvec(axis * (angle/np.linalg.norm(axis)))
        for q in connected:
            assert (tr.inv()*Rotation.from_quat(q)).magnitude() < 1e-12
        for kind in ("rgb", "geometry"):
            p = run/"map"/("map_"+kind+".pcd")
            with p.open("rb") as f:
                f.seek(pcd_info(p).offset)
                for i in range(5):
                    for j, dx in enumerate((0., 0., .02, .5)):
                        xyz = struct.unpack("<fff", f.read(12))
                        attr = struct.unpack("<I" if kind == "rgb" else "<f", f.read(4))[0]
                        expected = tr.apply([float(np.float32(i+dx)),2.,-3.])
                        assert np.linalg.norm(xyz-expected) < 6e-7, (kind, i, j, xyz, expected)
                        assert attr == (0xff00ee+i+j if kind == "rgb" else 77.+i+j)
        rows = [l.split() for l in (run/"map/trajectory.tum").read_text().splitlines()]
        for i, row in enumerate(rows):
            assert row[0] == f"{1700000000+i}.123456789"
            np.testing.assert_allclose(np.array(row[1:4],dtype=float), tr.apply([i,2,-3]), atol=1e-13)
            assert ((tr*Rotation.from_quat(NativeTopics.Q)).inv()*Rotation.from_quat(row[4:])).magnitude() < 1e-12
        checked = subprocess.run([sys.executable, str(ROOT/"runtime/saved_run_view.py"), "--run", str(run),
                                  "--check"], check=True, capture_output=True, text=True)
        return dict(domain=domain, viewers_while_mapping=len(actual_rviz), reopen_tf_clients=connected,
                    offscreen_rviz=actual_rviz, input_gravity=gravity, tf2_sample=read_json(run/"gravity_tf_check.json"),
                    processed_counts=manifest["counts"], all_poses_verified=True, point_attributes_verified=True,
                    duplicate_xyz_and_same_voxel_distinct_attributes_preserved=True,
                    structured_event=read_json(run/"gravity_transform.json")["initialization"],
                    fixture_exit=fixture.returncode, archive_exit=ret, segment_end_reason=result["segment_end_reason"],
                    independent_saved_view_check=checked.returncode, raw_prefix_removed_after_commit=True)
    finally:
        if fixture and fixture.poll() is None:
            fixture.communicate(input="stop\n", timeout=3)
        if arch:
            shutdown_process(arch)
        if native:
            native.node.destroy_node()
        if not shared:
            rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--domain", type=int, default=41)
    parser.add_argument("--with-rviz", action="store_true")
    parser.add_argument("--in-process", action="store_true", help="受控原生消息发布者与归档器共享 DDS participant")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="fast-livo2-gravity-") as tmp_name:
        tmp = Path(tmp_name)
        profile = ROOT/"configs/sensors/fastdds_camera.xml"
        os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(profile), RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
        emitter = tmp/"gravity_fixture"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT/"src/fast_livo/include"),
                        str(Path(__file__).with_name("gravity_fixture.cpp")), "-o", str(emitter)], check=True)
        cases = [run_case(tmp, emitter, "without_viewer", [1.1, -.8, -math.sqrt(9.81**2-1.1**2-.8**2)],
                          args.domain, False, False, args.in_process),
                 run_case(tmp, emitter, "reopen", [-3.,1.5,-math.sqrt(9.81**2-3**2-1.5**2)],
                          args.domain+1, True, args.with_rviz, args.in_process)]
    args.report.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.report, dict(verified_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                                platform=sys.platform, ros_distro=os.environ.get("ROS_DISTRO"),
                                input="synthetic ROS native topics; actual compiled C++ session recorder",
                                estimator_replay=False, physical_sensors=False,
                                synthetic_publisher_shares_dds_participant=args.in_process,
                                fastdds_profile_sha256=sha256(profile), cases=cases))
    print("PASS", args.report, flush=True)


if __name__ == "__main__":
    if sys.argv[1:2] == ["_embedded_archive"]:
        sys.exit(embedded_archive(Path(sys.argv[2])))
    main()
