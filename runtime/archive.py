"""One mapping session owns its gravity TF, streaming map and final commit."""
import argparse
import collections
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np

from gravity import BODY, CAMERA, GRAVITY, GravityEvents, live_binding, unit_quaternion, write_json
from gravity_export import COUNT_FIELDS, make_manifest
from map_store import (FILES, IDENTITY_VIEW, PCDStream, blocks, extract_vectors, pcd_info, ros_cloud,
                       ros_records, transform_records, tum_rows, write_pose)


class DiskSet:
    def __init__(self, directory, transform=None):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)
        self.transform = transform
        self.frame = GRAVITY if transform else CAMERA
        self.streams = {}
        self.trajectory = (directory / FILES["poses"]).open("x", encoding="ascii", newline="\n")
        self.poses = 0

    def cloud(self, kind, records):
        if kind not in self.streams:
            view = self.transform.viewpoint(IDENTITY_VIEW) if self.transform else IDENTITY_VIEW
            self.streams[kind] = PCDStream(self.directory / FILES[kind], records.dtype, self.frame, view)
        self.streams[kind].append(records)

    def pose(self, stamp, p, q):
        write_pose(self.trajectory, stamp, p, q)
        self.poses += 1

    def counts(self):
        return {**{COUNT_FIELDS[k]: self.streams[k].count if k in self.streams else 0
                   for k in ("rgb", "geometry")}, "poses": self.poses}

    def sync(self):
        for stream in [self.trajectory, *(s.file for s in self.streams.values())]:
            if not stream.closed:
                stream.flush()
                os.fsync(stream.fileno())

    def close(self):
        try:
            self.sync()
        finally:
            for s in self.streams.values():
                s.close()
            self.trajectory.close()


def remove_owned(directory, names):
    for name in names:
        (directory / name).unlink(missing_ok=True)
    directory.rmdir()


class Capture:
    def __init__(self, run):
        self.run = Path(run).resolve()
        self.out = self.run / "map"
        self.out.mkdir(exist_ok=True)
        if any(self.out.iterdir()):
            raise ValueError("会话 map 目录已有记录，拒绝覆盖或启动第二个写入者")
        self.binding = live_binding(self.run)
        self.events = GravityEvents(self.run, self.binding)
        self.raw = DiskSet(self.run / "raw" / CAMERA)
        self.stage = None
        self.transform = None
        self.message_count = 0
        self.nonfinite = {"rgb": 0, "geometry": 0}

    @property
    def writer(self):
        return self.stage if self.stage is not None else self.raw

    def refresh(self, final=False):
        self.events.refresh(final=final)
        if self.transform is not None or self.events.transform is None:
            return
        transform = self.events.transform
        self.raw.close()
        self.stage = DiskSet(self.run / "in_progress" / GRAVITY, transform)
        for kind in self.raw.streams:
            src = self.raw.directory / FILES[kind]
            header = pcd_info(src)
            for data in blocks(src, header):
                self.stage.cloud(kind, transform_records(data, transform))
        for stamp, p, q in tum_rows(self.raw.directory / FILES["poses"]):
            self.stage.pose(stamp, *transform.pose(p, q))
        if self.stage.counts() != self.raw.counts():
            raise ValueError("初始化前原始数据在转换时数量不符")
        self.stage.sync()
        write_json(self.run / "gravity_transform.json",
                   dict(identity=self.binding, initialization=self.events.evidence(), transform=transform.record()))
        self.transform = transform
        # A normal run retains just its aligned stream. If the switch fails, the
        # intact original prefix stays at raw/camera_init for recovery.
        remove_owned(self.raw.directory, FILES.values())
        self.raw.directory.parent.rmdir()

    def cloud(self, kind, native):
        if kind == "rgb" and not {"rgb", "rgba"}.intersection(native.dtype.names):
            raise ValueError("RGB 主题非空点云缺少原生颜色")
        xyz = extract_vectors(native)
        mask = np.isfinite(xyz).all(axis=1)
        self.nonfinite[kind] += len(native) - int(np.count_nonzero(mask))
        archived = transform_records(native, self.transform) if self.transform else native
        self.writer.cloud(kind, archived)
        self.message_count += 1
        if kind != "rgb" or self.transform is None or not mask.any():
            return None
        # The 10 cm voxel filter and display eviction apply only to RViz/Foxglove.
        # Every input record, including invalid XYZ and all attributes, stays on disk.
        display = archived if mask.all() else archived[mask]
        keys = np.floor(xyz[mask] / .1).astype(np.int64)
        _, ids = np.unique(keys, axis=0, return_index=True)
        return display, display[ids], keys[ids]

    def pose(self, stamp, p, q, parent, child):
        if parent != CAMERA or child != BODY:
            raise ValueError(f"意外的里程计父/子坐标：{parent}/{child}")
        if not np.isfinite(p).all():
            raise ValueError("里程计位置无效")
        if self.transform is not None:
            p, q = self.transform.pose(p, q)
        else:
            unit_quaternion(q)
        self.writer.pose(stamp, p, q)
        self.message_count += 1

    def status(self):
        self.writer.trajectory.flush()
        return dict(**self.writer.counts(), gravity_ready=self.transform is not None,
                    frame=self.writer.frame, messages=self.message_count, error=None)

    def complete(self, tf_proof):
        self.refresh(final=True)
        if live_binding(self.run) != self.binding:
            raise ValueError("算法或参数快照在会话内已改变")
        self.stage.close()
        if not tf_proof or not tf_proof.get("verified"):
            raise ValueError("实际 tf2 查询尚未验证，保留已写入数据")
        evidence = self.events.evidence()
        counts = self.stage.counts()
        if any(counts[COUNT_FIELDS[k]] <= self.nonfinite[k] for k in self.nonfinite):
            raise ValueError("原生点云缺少有限 XYZ 观测")
        manifest = make_manifest(self.stage.directory, self.transform, self.binding, evidence, counts)
        manifest["tf_check"] = tf_proof
        manifest["sampling"] = "every received native point; 0.1-m voxel filtering applies only to the live display"
        manifest["nonfinite_input_xyz"] = self.nonfinite
        write_json(self.stage.directory / "manifest.json", manifest)
        published = []
        names = [*FILES.values(), "manifest.json"]
        try:
            # The supervisor and the exclusive run/staging directories admit a
            # single writer. Same-volume renames also work on media without links.
            for name in names:  # Readers require the manifest, renamed last.
                if (self.out / name).exists():
                    raise FileExistsError(str(self.out / name))
                (self.stage.directory / name).rename(self.out / name)
                published.append(name)
        except BaseException:
            for name in reversed(published):
                (self.out / name).rename(self.stage.directory / name)
            raise
        result = dict(status="complete", error=None, output_frame=GRAVITY, gravity_applied=True,
                      segment_end_reason=self.events.ended, **counts)
        try:
            self.stage.directory.rmdir()
            self.stage.directory.parent.rmdir()
        except OSError as exc:
            result["staging_cleanup_warning"] = str(exc)
        return result

    def fail(self, exc):
        for store in (self.raw, self.stage):
            if store is not None:
                try:
                    store.close()
                except OSError:
                    pass
        paths = {store.frame: str(store.directory.relative_to(self.run)) for store in (self.raw, self.stage)
                 if store is not None and store.directory.exists()}
        return dict(status="incomplete", error=str(exc), committed=False, preserved=paths,
                    **self.writer.counts())


class SessionTF:
    def __init__(self, node):
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
        self.node = node
        self.broadcaster = StaticTransformBroadcaster(node)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.sent_stamp = None
        self.sent_at = None
        self.proof = None
        static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.static_sub = node.create_subscription(TFMessage, "/tf_static", self.check_static, static)
        self.dynamic_sub = node.create_subscription(TFMessage, "/tf", self.check_dynamic, dynamic)

    def check_static(self, msg):
        self.watch(msg, True)

    def check_dynamic(self, msg):
        self.watch(msg, False)

    def watch(self, msg, is_static):
        for t in msg.transforms:
            if t.child_frame_id not in (CAMERA, GRAVITY):
                continue
            stamp = t.header.stamp
            if (is_static and self.sent_stamp is not None and t.header.frame_id == GRAVITY
                    and t.child_frame_id == CAMERA and (stamp.sec, stamp.nanosec) == self.sent_stamp):
                continue
            raise ValueError(f"发现另一发布者占用重力 TF：{t.header.frame_id} -> {t.child_frame_id}")

    def poll(self, capture):
        if capture.transform is None or self.proof is not None:
            return
        import rclpy
        from geometry_msgs.msg import PointStamped, TransformStamped
        from tf2_geometry_msgs import do_transform_point
        tr = capture.transform
        if self.sent_stamp is None:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = GRAVITY, CAMERA
            tf.header.stamp = self.node.get_clock().now().to_msg()
            stamp = tf.header.stamp
            self.sent_stamp = stamp.sec, stamp.nanosec
            self.sent_at = time.monotonic()
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = map(float, tr.q)
            self.broadcaster.sendTransform(tf)
        at = rclpy.time.Time()
        if not self.buffer.can_transform(GRAVITY, CAMERA, at):
            if time.monotonic() - self.sent_at > 10:
                raise ValueError("未能通过实际 tf2 查询确认本会话的重力 TF")
            return
        actual = self.buffer.lookup_transform(GRAVITY, CAMERA, at)
        r, p = actual.transform.rotation, actual.transform.translation
        q = np.array([r.x, r.y, r.z, r.w])
        if (min(np.linalg.norm(q-tr.q), np.linalg.norm(q+tr.q)) > 1e-10
                or max(abs(p.x), abs(p.y), abs(p.z)) > 1e-12):
            raise ValueError("tf2 实际查询的旋转或平移与本次初始化不符")
        point = PointStamped()
        point.header.frame_id = CAMERA
        point.point.x, point.point.y, point.point.z = 1.25, -.75, 2.5
        query = do_transform_point(point, actual).point
        measured = np.array([query.x, query.y, query.z])
        expected = tr.vectors([1.25, -.75, 2.5])
        diff = float(np.linalg.norm(expected - measured))
        if not np.isfinite(diff) or diff > 1e-10:
            raise ValueError("tf2 样本点实际方向验证失败")
        self.proof = dict(verified=True, target=GRAVITY, source=CAMERA, quaternion_xyzw=q.tolist(),
                          sample_camera_init=[1.25, -.75, 2.5], sample_map_gravity=measured.tolist(),
                          expected_map_gravity=expected.tolist(), error_m=diff)
        write_json(capture.run / "gravity_tf_check.json", self.proof)


def main(run):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2
    capture = Capture(run)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fast_livo2_map_writer")
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    cache = collections.OrderedDict()
    last_rgb = None
    last_publish = 0.
    last_status = 0.
    owner = None
    try:
        owner = SessionTF(node)
        qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE)
        recent = node.create_publisher(PointCloud2, "/livo_view/accumulated_rgb", 1)
        relay = node.create_publisher(PointCloud2, "/livo_view/cloud_registered", 2)

        def cloud(msg, kind):
            nonlocal last_rgb
            if not msg.fields or not msg.width or not msg.height:
                return
            capture.refresh()
            display = capture.cloud(kind, ros_records(msg))
            if display is None:
                return
            full, selected, keys = display
            relay.publish(ros_cloud(full, GRAVITY, msg.header.stamp))
            last_rgb = selected.dtype, msg.header.stamp
            for i, key in enumerate(keys):
                key = tuple(key)
                cache[key] = selected[i].tobytes()
                cache.move_to_end(key)
                if len(cache) > 200000:
                    cache.popitem(last=False)

        def odom(msg):
            capture.refresh()
            p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
            capture.pose(f"{t.sec}.{t.nanosec:09d}", (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                         msg.header.frame_id, msg.child_frame_id)

        subscriptions = [
            node.create_subscription(PointCloud2, "/cloud_registered", lambda m: cloud(m, "rgb"), qos),
            node.create_subscription(PointCloud2, "/cloud_effected", lambda m: cloud(m, "geometry"), qos),
            node.create_subscription(Odometry, "/aft_mapped_to_init", odom, qos)]
        # Discover any existing TF owner before the supervisor starts this producer.
        until = time.monotonic() + 1.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        (capture.out / "ready").touch(exist_ok=False)
        while not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
            capture.refresh()
            owner.poll(capture)
            now = time.monotonic()
            if now - last_status >= 1:
                s = capture.status()
                write_json(capture.out / "status.json", dict(**s, display_voxels=len(cache),
                                                             ros_domain_id=node.context.get_domain_id(),
                                                             tf_verified=owner.proof is not None))
                last_status = now
            if last_rgb and now - last_publish >= 2:
                dtype, stamp = last_rgb
                recent.publish(ros_cloud(np.frombuffer(b"".join(cache.values()), dtype), GRAVITY, stamp))
                last_publish = now
        # The supervisor signals after the algorithm exits. Drain consecutive idle
        # time, allowing longer when the executor still delivers final messages.
        idle_since = time.monotonic()
        while time.monotonic() - idle_since < 1.0:
            previous = capture.message_count
            rclpy.spin_once(node, timeout_sec=.04)
            capture.refresh()
            owner.poll(capture)
            if previous != capture.message_count:
                idle_since = time.monotonic()
        if capture.transform and owner.proof is None:
            while owner.proof is None:
                rclpy.spin_once(node, timeout_sec=.05)
                owner.poll(capture)
        result = capture.complete(owner.proof)
    except Exception as exc:
        result = capture.fail(exc)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    write_json(capture.out / "result.json", result)
    write_json(capture.out / "status.json", dict(**result, gravity_ready=capture.transform is not None,
                                                 tf_verified=bool(getattr(owner, "proof", None))))
    print(result, flush=True)
    return int(result["status"] != "complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    try:
        sys.exit(main(parser.parse_args().run))
    except Exception as exc:
        print(str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
