"""Export a verified historical mapping archive into a new, self-describing gravity map."""
import argparse
import math
import os
from pathlib import Path, PurePosixPath
import tempfile

import numpy as np

from gravity import (BODY, CAMERA, GRAVITY, MAP_SCHEMA, GravityTransform, check_config,
                     legacy_log, read_json, sha256, write_json)
from map_store import FILES, convert_pcd, convert_tum, pcd_info, tum_rows

COUNT_FIELDS = {"rgb": "rgb_points", "geometry": "geometry_points", "poses": "poses"}
# These audited historical writers store original camera_init coordinates and a
# neutral geometry color. An unlabeled PCD alone does not establish its frame.
LEGACY_WRITERS = {
    "archive.py": "3c3a9288cbf9629a413dac54b78c6214a75d7be052bd70467c6f744dbc14d4ce",
    "map_store.py": "badbce09e939dec428942b62f86ea946974aecaa189d64eac77887297a7aa9ef",
}


def verify_codecs(directory, counts, frame):
    for kind in ("rgb", "geometry"):
        header = pcd_info(directory / FILES[kind])
        if header.frame != frame or header.count != counts[COUNT_FIELDS[kind]]:
            raise ValueError(f"{kind} PCD 的坐标或总点数与清单不符")
    if sum(1 for _ in tum_rows(directory / FILES["poses"])) != counts["poses"]:
        raise ValueError("TUM 位姿数与清单不符")


def make_manifest(directory, transform, binding, evidence, counts):
    if any(counts[k] <= 0 for k in COUNT_FIELDS.values()):
        raise ValueError("RGB、几何点云或轨迹为空，完整地图无法提交")
    verify_codecs(directory, counts, GRAVITY)
    files = {name: {"sha256": sha256(directory / name), "frame": GRAVITY,
                    "gravity_applied": True} for name in FILES.values()}
    historical = evidence["kind"] == "legacy_complete_estimator_log"
    return dict(schema=MAP_SCHEMA, status="complete", input_frame=CAMERA, output_frame=GRAVITY,
                gravity_applied=True, transform=transform.record(), identity=binding,
                initialization=evidence, files=files, counts=counts, trajectory_body=BODY,
                official_outputs={"directory":"directory containing this manifest (map/ in a live session)",
                                  "rgb":FILES["rgb"], "geometry":FILES["geometry"], "trajectory":FILES["poses"]},
                trajectory_convention="T_GB = T_GC * T_CB; timestamp, tx ty tz qx qy qz qw",
                trajectory_timestamp=("historical /aft_mapped_to_init.header.stamp; original recorded decimal preserved"
                                      if historical else "/aft_mapped_to_init.header.stamp; exact integer sec/nanosec"),
                native_odometry_stamp_source="LIVO: estimator node clock at publication; ONLY_LIO: lidar update time",
                optional_native_pose_log={"path":"Log/result/<evo.seq_name>.txt", "enabled_by":"evo.pose_output_en",
                                          "frame":CAMERA, "gravity_applied":False, "primary":False,
                                          "time":"LidarMeasures.last_lio_update_time"},
                native_odometry_fields="TUM stores pose only. Native odometry remains in camera_init; twist is in its child frame.",
                ros_frames={"native_clouds_path_and_odometry": CAMERA, "native_odometry_child": BODY,
                            "live_rgb_views_at_acquisition": CAMERA if historical else GRAVITY, "saved_view": GRAVITY})


def validate_map(directory):
    directory = Path(directory).resolve()
    manifest = read_json(directory / "manifest.json")
    if (manifest.get("schema") != MAP_SCHEMA or manifest.get("status") != "complete"
            or manifest.get("gravity_applied") is not True or manifest.get("output_frame") != GRAVITY
            or manifest.get("input_frame") != CAMERA or manifest.get("trajectory_body") != BODY):
        raise ValueError("归档缺少已对齐、已提交的地图清单")
    tr = manifest["transform"]
    check = GravityTransform(tr["gravity_camera_init_m_s2"])
    if (tr["parent"] != GRAVITY or tr["child"] != CAMERA
            or not np.allclose(tr["R_GC"], check.R, rtol=0, atol=1e-13)
            or not np.allclose(tr["quaternion_xyzw"], check.q, rtol=0, atol=1e-13)
            or tr["translation_m"] != [0, 0, 0]):
        raise ValueError("地图变换与记录的重力不符")
    for name in FILES.values():
        info = manifest["files"][name]
        if (info.get("frame") != GRAVITY or info.get("gravity_applied") is not True
                or sha256(directory / name) != info["sha256"]):
            raise ValueError("归档坐标或 SHA256 不符：" + name)
    verify_codecs(directory, manifest["counts"], GRAVITY)
    return manifest


def verified_legacy(data):
    data = Path(data).resolve()
    inventory_path = data / "remote_manifest.json"
    inventory = read_json(inventory_path)
    origin = inventory["mapping_run"]
    run = data / "mapping_session"
    records = {row["archive_path"]: row for row in inventory["files"]}
    source_files = {}

    def checked(relative, expected_origin=None):
        rec = records[relative]
        path = data / relative
        if (expected_origin is not None and rec["source_path"] != expected_origin
                or path.stat().st_size != rec["size"] or sha256(path) != rec["sha256"]):
            raise ValueError("历史文件 SHA、归属或长度不符：" + relative)
        source_files[relative] = rec["sha256"]
        return path

    for name in ("algorithm.log", "resolved.yaml", "parameters_manifest.json", "session.json",
                 "map/result.json", *("map/" + n for n in FILES.values())):
        checked("mapping_session/" + name, origin + "/" + name)
    session = read_json(run / "session.json")
    counts = read_json(run / "map/result.json")
    if (session["run"] != origin or session["processes"]["archive"] != 0
            or counts.get("error") or counts != session["map"]):
        raise ValueError("历史会话、归档进程状态或统计不一致")
    config_sha = check_config(run, origin)
    executable = str(PurePosixPath(origin).parent.parent / "candidate/build/fastlivo_mapping")
    exe = checked("environment/candidate/build/fastlivo_mapping", executable)
    algorithm_sha = source_files["environment/candidate/build/fastlivo_mapping"]
    # Check the archived supervisor selected this ELF for this run, rather than
    # assuming every binary in the same tar file was the mapping algorithm.
    runtime = str(PurePosixPath(origin).parent.parent / "runtime/session.py")
    runner = checked("environment/runtime/session.py", runtime).read_text(encoding="utf-8")
    if ("{ROOT}/candidate/build/fastlivo_mapping --ros-args --params-file {resolved}" not in runner
            or str(PurePosixPath(origin).parent.parent) not in runner or not exe.is_file()):
        raise ValueError("历史会话入口和执行文件身份无法绑定")
    for name, expected_hash in LEGACY_WRITERS.items():
        key = "environment/runtime/" + name
        checked(key, str(PurePosixPath(runtime).with_name(name)))
        if source_files[key] != expected_hash:
            raise ValueError("尚未核实历史写入器保存的是原坐标，拒绝二次补偿：" + name)
    transform, number = legacy_log(run / "algorithm.log")
    display_key = "mapping_session/display/transform.json"
    if display_key in records:
        prior = read_json(checked(display_key, origin + "/display/transform.json"))
        if (prior["run"] != origin or prior["params"] != origin + "/resolved.yaml"
                or prior["log"] != origin + "/algorithm.log" or prior["child"] != CAMERA
                or prior["parent"] != GRAVITY or prior["translation"] != [0, 0, 0]
                or not np.allclose(prior["quaternion_xyzw"], transform.q, atol=1e-12, rtol=0)):
            raise ValueError("历史同会话显示记录不匹配")
    binding = dict(session_id=PurePosixPath(origin).name, source_run=origin,
                   executable=executable, algorithm_sha256=algorithm_sha, resolved_sha256=config_sha)
    # Historical RCLCPP printf used %.4f. Each component error <= 5e-5 m/s².
    angular_bound = math.asin(math.sqrt(3)*0.00005 / (np.linalg.norm(transform.gravity) - math.sqrt(3)*0.00005))
    evidence = dict(kind="legacy_complete_estimator_log", line=number, segment=1,
                    source="ImuProcess::Process2/stat.gravity", numeric_precision="%.4f",
                    gravity_direction_rounding_bound_deg=math.degrees(angular_bound),
                    inventory_sha256=sha256(inventory_path), checked_source_sha256=source_files,
                    historical_algorithm_exit=session["processes"]["algorithm"])
    return run, transform, binding, evidence, {k: counts[k] for k in COUNT_FIELDS.values()}


def export_legacy(data, destination):
    data, destination = Path(data).resolve(), Path(destination).resolve()
    if destination.exists() or destination.is_relative_to(data):
        raise ValueError("请选择历史输入目录以外、尚不存在的新输出目录")
    run, transform, binding, evidence, expected = verified_legacy(data)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".gravity-export-", dir=destination.parent) as tmp:
        stage = Path(tmp) / "map"
        stage.mkdir()
        counts = {}
        for kind, name in FILES.items():
            convert = convert_tum if kind == "poses" else convert_pcd
            counts[COUNT_FIELDS[kind]] = convert(run / "map" / name, stage / name, transform)
        if counts != expected:
            raise ValueError("转换后点和位姿数与原始归档不符")
        manifest = make_manifest(stage, transform, binding, evidence, counts)
        write_json(stage / "manifest.json", manifest)
        # Exclusive claim avoids replacing an empty user-created directory.
        # Readers require the last manifest. A failed export can be retried.
        destination.mkdir()
        moved = []
        try:
            for name in (*FILES.values(), "manifest.json"):
                os.replace(stage / name, destination / name)
                moved.append(name)
        except BaseException:
            for name in moved:
                (destination / name).unlink()
            destination.rmdir()
            raise
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy-archive", required=True, type=Path, help="包含 remote_manifest.json 的 data 目录")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = export_legacy(args.legacy_archive, args.output)
    print(f"{args.output}  {result['output_frame']}  {result['counts']}")
