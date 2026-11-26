"""Offline regression checks. The reference rotations and the binary PCD reader are independent."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from archive import Capture
from gravity import (BODY, CAMERA, GRAVITY, GravityTransform, legacy_log,
                     read_json, sha256, write_json)
from gravity_export import validate_map
from map_store import PCDStream, convert_pcd, pcd_info, ros_records


def ref_rotation(g):
    g = np.asarray(g, dtype=float)
    return Rotation.align_vectors([[0., 0., -1.]], [g / np.linalg.norm(g)])[0]


def independent_pcd(path):
    with path.open("rb") as f:
        fields = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD missing data")
            key, *vals = line.decode("ascii").split()
            if key == "#":
                continue
            fields[key] = vals
            if key == "DATA":
                break
        if fields["FIELDS"] != ["x", "y", "z", "rgb"] or fields["DATA"] != ["binary"]:
            raise ValueError("unexpected legacy test layout")
        count = int(fields["POINTS"][0])
        offset = f.tell()
    dtype = np.dtype([("xyz", "<f4", 3), ("rgb", "<u4")])
    if offset + count * dtype.itemsize != path.stat().st_size:
        raise ValueError("PCD byte count mismatch")
    return np.memmap(path, mode="r", dtype=dtype, shape=(count,), offset=offset), fields


def compare_real(raw_data, aligned):
    raw = raw_data / "mapping_session"
    inventory = json.loads((raw_data / "remote_manifest.json").read_text(encoding="utf-8"))
    listed = {r["archive_path"]: r["sha256"] for r in inventory["files"]}
    originals = ["algorithm.log", "resolved.yaml", "parameters_manifest.json",
                 "map/map_rgb.pcd", "map/map_geometry.pcd", "map/trajectory.tum"]
    hashes = {}
    for relative in originals:
        checksum = hashlib.sha256()
        with (raw / relative).open("rb") as f:
            for block in iter(lambda: f.read(1024*1024), b""):
                checksum.update(block)
        digest = checksum.hexdigest()
        assert digest == listed["mapping_session/" + relative]
        hashes[relative] = digest
    with (raw / "algorithm.log").open(encoding="utf-8", errors="replace") as f:
        rows = [s.split("IMU Initials: Gravity: ")[1].split()[:3] for s in f if "IMU Initials: Gravity: " in s]
    assert len(rows) == 1
    g = np.array(rows[0], dtype=float)
    reference = ref_rotation(g)
    report = dict(session=inventory["mapping_run"], gravity_camera_init_m_s2=g.tolist(),
                  reference="scipy.spatial.transform.Rotation.align_vectors; independent PCD/TUM readers",
                  original_sha256_matched=hashes, tilt_deg=float(np.degrees(reference.magnitude())),
                  gravity_after=reference.apply(g).tolist(), clouds={})
    for file in ("map_rgb.pcd", "map_geometry.pcd"):
        src, src_h = independent_pcd(raw / "map" / file)
        dst, dst_h = independent_pcd(aligned / file)
        assert len(src) == len(dst) == int(src_h["WIDTH"][0]) * int(src_h["HEIGHT"][0])
        max_err = 0.
        mismatched_color = 0
        f32_disagreements = 0
        for start in range(0, len(src), 65536):
            a, b = src[start:start+65536], dst[start:start+65536]
            expected = reference.apply(a["xyz"])
            max_err = max(max_err, float(np.max(np.abs(expected - b["xyz"]))))
            mismatched_color += int(np.count_nonzero(a["rgb"] != b["rgb"]))
            f32_disagreements += int(np.count_nonzero(expected.astype("<f4") != b["xyz"]))
        assert max_err < 1e-5 and not mismatched_color
        old_view = np.array(src_h["VIEWPOINT"], dtype=float)
        new_view = np.array(dst_h["VIEWPOINT"], dtype=float)
        assert np.allclose(new_view[:3], reference.apply(old_view[:3]), rtol=0, atol=1e-12)
        target_view = reference * Rotation.from_quat(old_view[[4, 5, 6, 3]])
        assert (target_view.inv() * Rotation.from_quat(new_view[[4, 5, 6, 3]])).magnitude() < 1e-12
        indices = [0, len(src)//2, len(src)-1]
        examples = [dict(index=i, camera_init=src["xyz"][i].tolist(), expected=reference.apply(src["xyz"][i]).tolist(),
                         saved_map_gravity=dst["xyz"][i].tolist(), packed_rgb=int(dst["rgb"][i])) for i in indices]
        edge_c = src["xyz"][indices[-1]].astype(float) - src["xyz"][0]
        edge_g = dst["xyz"][indices[-1]].astype(float) - dst["xyz"][0]
        slope_error = abs(float(-g @ edge_c / np.linalg.norm(g)) - float(edge_g[2]))
        assert slope_error < 1e-5
        report["clouds"][file] = dict(points=len(src), max_coordinate_error_m=max_err, mismatched_rgb=mismatched_color,
                                       unequal_float32_components=f32_disagreements, viewpoint_xyzw=new_view[[4,5,6,3]].tolist(),
                                       sample_vertical_separation_error_m=slope_error, samples=examples)
    def load_tum(file):
        rows = [line.split() for line in file.read_text(encoding="ascii").splitlines() if line and line[0] != "#"]
        return [r[0] for r in rows], np.array([r[1:] for r in rows], dtype=np.float64)
    t_src, poses = load_tum(raw / "map/trajectory.tum")
    t_dst, saved = load_tum(aligned / "trajectory.tum")
    assert t_src == t_dst and len(poses) == len(saved)
    p_expected = reference.apply(poses[:, :3])
    q_expected = reference * Rotation.from_quat(poses[:, 3:])
    q_saved = Rotation.from_quat(saved[:, 3:])
    p_err = float(np.max(np.linalg.norm(p_expected - saved[:, :3], axis=1)))
    q_err = float(np.max((q_expected.inv() * q_saved).magnitude()))
    unit_err = float(np.max(abs(np.linalg.norm(saved[:, 3:], axis=1)-1)))
    assert p_err < 1e-12 and q_err < 1e-12 and unit_err < 1e-12
    report["trajectory"] = dict(poses=len(poses), identical_timestamps=True, max_position_error_m=p_err,
                                max_orientation_error_rad=q_err, max_unit_quaternion_error=unit_err,
                                samples=[dict(row=i, timestamp=t_src[i], raw_pose=poses[i].tolist(),
                                              saved_pose=saved[i].tolist()) for i in [0, len(poses)//2, len(poses)-1]])
    m = validate_map(aligned)
    report["manifest_sha256"] = sha256(aligned / "manifest.json")
    report["output_sha256"] = {name: record["sha256"] for name, record in m["files"].items()}
    return report


def prepare_run(root):
    run = Path(root) / "fixture_session"
    run.mkdir()
    config = run / "resolved.yaml"
    config.write_text("/**:\n  ros__parameters:\n    uav.gravity_align_en: false\n", encoding="utf-8")
    write_json(run/"parameters_manifest.json", {"run": str(run), "resolved_sha256": sha256(config)})
    exe = Path(sys.executable).resolve()
    identity = dict(session_id=run.name, executable=str(exe), sha256=sha256(exe))
    write_json(run/"algorithm_identity.json", identity)
    return run


def event(run, gravity, kind="initialized"):
    import os
    identity = read_json(run/"algorithm_identity.json")
    m = read_json(run/"parameters_manifest.json")
    obj = dict(event=kind, segment=1, pid=os.getpid(), session_id=run.name, resolved_sha256=m["resolved_sha256"],
               algorithm_sha256=identity["sha256"], executable=identity["executable"])
    if kind == "initialized":
        obj.update(source="ImuProcess::Process2/stat.gravity", gravity_camera_init_m_s2=gravity)
    else:
        obj["reason"] = "reinitialization_requested"
    with (run/"gravity_events.jsonl").open("a", encoding="utf-8", newline="\n") as f:
        print(json.dumps(obj), file=f)


class GravityChecks(unittest.TestCase):
    def test_entire_ros_cloud_including_duplicate_xyz_and_packed_bits(self):
        names = ("x", "y", "z", "rgba")
        fields = [SimpleNamespace(name=n, datatype=6 if n=="rgba" else 7, offset=4*i, count=1)
                  for i,n in enumerate(names)]
        import struct
        repeated = [(1.,2.,3.,0xf87fedc0), (1.,2.,3.,0xffffffff), (1.01,2.,3.,0xffdddddd)]
        row = b"".join(struct.pack(">fffI", *v) + b"ABCD" for v in repeated) + b"row pad!"
        msg = SimpleNamespace(header=SimpleNamespace(frame_id=CAMERA), width=3, height=2,
                              point_step=20, row_step=len(row), fields=fields, data=row*2, is_bigendian=True)
        points = ros_records(msg)
        np.testing.assert_array_equal(points["rgba"], [v[3] for v in repeated]*2)
        with tempfile.TemporaryDirectory() as d:
            run = prepare_run(d)
            c = Capture(run)
            for kind in ("rgb", "geometry"):
                c.cloud(kind, points)
            c.pose("10.123456789", [0,1,2], [0,0,0,1], CAMERA, BODY)
            event(run, [1, 2, -9.52])
            c.refresh()
            for kind in ("rgb", "geometry"):
                c.cloud(kind, points)
            c.pose("11.123456789", [1,1,2], [0,0,0,1], CAMERA, BODY)
            # This unit checks the writer; the independent ROS test verifies the real TF.
            result = c.complete({"verified":True, "source":"unit_fixture"})
            self.assertEqual(result["status"], "complete")
            self.assertFalse((run/"raw").exists())
            m = validate_map(run/"map")
            self.assertEqual(m["counts"], dict(rgb_points=12, geometry_points=12, poses=2))
            h = pcd_info(run/"map/map_rgb.pcd")
            archived = np.fromfile(run/"map/map_rgb.pcd", dtype=h.dtype, offset=h.offset)
            np.testing.assert_array_equal(archived["rgba"], [v[3] for v in repeated]*4)
            self.assertEqual(h.frame, GRAVITY)

    def test_representative_and_near_antipodal(self):
        for v in ([0, 0, -1], [1e-11, -3e-13, -1], [1, 2, 3], [-1, 0, 0],
                  [0, 1, 0], [.1, -.4, -.9], [7e-13, -4e-11, 1], [0, 0, 1]):
            g = np.array(v)*9.81/np.linalg.norm(v)
            tr = GravityTransform(g)
            np.testing.assert_allclose(tr.R @ g, [0, 0, -9.81], atol=1e-13)
            np.testing.assert_allclose(tr.R.T @ tr.R, np.eye(3), atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(tr.R), 1, places=14)
            self.assertEqual(tr.q[2], 0.)
            r = Rotation.from_quat(tr.q)
            body = Rotation.from_euler("xyz", [1.2, -.52, .69])
            p, q = tr.pose([1, 2, -3], body.as_quat())
            np.testing.assert_allclose(p, r.apply([1, 2, -3]), atol=1e-14)
            self.assertLess(((r * body).inv() * Rotation.from_quat(q)).magnitude(), 1e-14)

    def test_pcd_extra_fields_viewpoint_and_double_rotation(self):
        dt = np.dtype([(n, "<f8") for n in ("x","y","z","normal_x","normal_y","normal_z","vp_x","vp_y","vp_z")]
                      + [("label","<u2"), ("rgb","<u4"), ("intensity","<f4"), ("descriptor","<i2",(3,))])
        a = np.zeros(4, dt)
        for i, n in enumerate(dt.names[:9]):
            a[n] = np.arange(4) + i
        a["rgb"] = [0xff00aa12, 0, 0xff33, 0xffffffff]
        a["label"], a["intensity"], a["descriptor"] = [3, 7, 0, 9], [5, .2, 0, 1], [[7, 9, 1]]*4
        g = [0, 9.81, 0]
        tr = GravityTransform(g)
        reference = ref_rotation(g)
        with tempfile.TemporaryDirectory() as d:
            src, dest = Path(d)/"a.pcd", Path(d)/"b.pcd"
            view = [1,2,3,*Rotation.from_rotvec([.4,.5,.6]).as_quat()[[3,0,1,2]]]
            s = PCDStream(src, dt, CAMERA, view)
            s.append(a)
            s.close(height=2)
            self.assertEqual(convert_pcd(src, dest, tr), 4)
            h = pcd_info(dest)
            self.assertEqual((h.width, h.height), (2,2))
            b = np.memmap(dest, dtype=dt, mode="r", offset=h.offset, shape=(4,))
            for group in (dt.names[:3], dt.names[3:6], dt.names[6:9]):
                np.testing.assert_allclose(np.column_stack([b[n] for n in group]),
                                           reference.apply(np.column_stack([a[n] for n in group])), atol=1e-13)
            for n in dt.names[9:]:
                self.assertEqual(a[n].tobytes(), b[n].tobytes())
            np.testing.assert_allclose(h.viewpoint[:3], reference.apply(view[:3]), atol=1e-13)
            rq = reference * Rotation.from_quat(np.array(view)[[4,5,6,3]])
            self.assertLess((rq.inv()*Rotation.from_quat(np.asarray(h.viewpoint)[[4,5,6,3]])).magnitude(), 1e-13)
            del b
            with self.assertRaises(ValueError):
                convert_pcd(dest, Path(d)/"twice.pcd", tr)

    def test_missing_evidence_and_reinit(self):
        with tempfile.TemporaryDirectory() as d:
            run = prepare_run(d)
            c = Capture(run)
            c.pose("123.000000001", [1,2,3], [0,0,0,1], CAMERA, BODY)
            with self.assertRaisesRegex(ValueError, "缺少"):
                c.complete({"verified":True})
            failed = c.fail("missing")
            self.assertEqual(failed["status"], "incomplete")
            self.assertTrue((run/"raw/camera_init/trajectory.tum").is_file())
            self.assertFalse((run/"map/manifest.json").exists())
        with tempfile.TemporaryDirectory() as d:
            run = prepare_run(d)
            c = Capture(run)
            event(run, [0, 9.81, 0])
            c.refresh()
            event(run, [0, -9.81, 0], "segment_closed")
            c.refresh(final=True)
            self.assertEqual(c.events.ended, "reinitialization_requested")
            event(run, [0, -9.81, 0])
            with self.assertRaisesRegex(ValueError, "重复初始化"):
                c.refresh(final=True)
            c.fail("segmentation check")

    def test_mismatched_and_modified_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            run = prepare_run(d)
            c = Capture(run)
            event(run, [0,-9.81,0])
            p = run/"gravity_events.jsonl"
            line = p.read_text()
            p.write_text(line.replace(run.name, "different_run"))
            with self.assertRaisesRegex(ValueError, "身份不符"):
                c.refresh()
            p.write_text(line)
            c.refresh()
            p.write_text(line.replace("-9.81", "9.81"))
            with self.assertRaisesRegex(ValueError, "替换"):
                c.refresh()
            c.fail("changed")

    def test_historical_ambiguity(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d)/"algorithm.log"
            line = "IMU Initials: Gravity: 0.0000 0.0000 -9.8100 1.0; details\n"
            for text in ("", line+line, line+"Reset ImuProcess\n", "Gravity Alignment Starts\n"+line):
                log.write_text(text)
                with self.assertRaises(ValueError):
                    legacy_log(log)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-data", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(GravityChecks))
    if not result.wasSuccessful():
        sys.exit(1)
    report = {"verified_at":datetime.now().astimezone().isoformat(timespec="seconds"),
              "unit_tests":result.testsRun, "result":"pass"}
    if args.real_data is not None:
        report["real_archive"] = compare_real(args.real_data, args.map)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        write_json(args.report, report)
    print(json.dumps({"tests":result.testsRun, "real_full_archive":bool(args.real_data)}, ensure_ascii=False))
