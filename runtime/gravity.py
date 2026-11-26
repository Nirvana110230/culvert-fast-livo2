"""The single camera_init -> map_gravity transform used by TF, writers and export."""
import hashlib
import json
import math
import os
import re
import tempfile
from pathlib import Path

import numpy as np
import yaml

CAMERA = "camera_init"
GRAVITY = "map_gravity"
BODY = "aft_mapped"
MAP_SCHEMA = "fast_livo2.gravity_map.v1"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False,
                                     encoding="utf-8", prefix="." + path.name + ".") as f:
        tmp = Path(f.name)
        try:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        except BaseException:
            f.close()
            tmp.unlink(missing_ok=True)
            raise
    try:
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def unit_quaternion(values):
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("四元数 xyzw 无效")
    n = math.hypot(*q)
    if n < 1e-12 or abs(n - 1) > 0.01:
        raise ValueError("输入四元数不是有效的单位姿态")
    return q / n


def compose_quaternions(left, right):
    a, b, c, d = unit_quaternion(left)
    x, y, z, w = unit_quaternion(right)
    return unit_quaternion((d*x+a*w+b*z-c*y, d*y-a*z+b*w+c*x,
                            d*z+a*y-b*x+c*w, d*w-a*x-b*y-c*z))


def quaternion_matrix(q):
    x, y, z, w = unit_quaternion(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class GravityTransform:
    def __init__(self, gravity):
        self.gravity = np.asarray(gravity, dtype=np.float64)
        if self.gravity.shape != (3,) or not np.isfinite(self.gravity).all():
            raise ValueError("初始化重力向量必须含三个有限分量")
        x, y, z = self.gravity
        norm = math.hypot(x, y, z)
        if not 9.5 < norm < 10.1:
            raise ValueError("初始化重力大小超出物理范围")
        horizontal = math.hypot(x, y)
        if horizontal:
            half = 0.5 * math.atan2(horizontal, -z)
            self.q = np.array([(-y/horizontal)*math.sin(half),
                               (x/horizontal)*math.sin(half), 0.0, math.cos(half)])
        else:
            # The antipodal case has no preferred horizontal axis. Choose +X.
            self.q = np.array([1., 0., 0., 0.] if z > 0 else [0., 0., 0., 1.])
        self.R = quaternion_matrix(self.q)
        if (np.max(np.abs(self.R.T @ self.R - np.eye(3))) > 2e-14
                or abs(np.linalg.det(self.R) - 1) > 2e-14
                or np.linalg.norm(self.R @ self.gravity - [0, 0, -norm]) > 2e-13):
            raise ValueError("重力旋转矩阵验证失败")

    def vectors(self, xyz):
        return np.asarray(xyz, dtype=np.float64) @ self.R.T

    def pose(self, position, orientation):
        if not np.isfinite(position).all():
            raise ValueError("非有限姿态位置")
        return self.vectors(position), compose_quaternions(self.q, orientation)

    def viewpoint(self, values):
        tx, ty, tz, qw, qx, qy, qz = values
        p, q = self.pose([tx, ty, tz], [qx, qy, qz, qw])
        return [*p, q[3], *q[:3]]

    def record(self):
        return dict(parent=GRAVITY, child=CAMERA, direction="p_G = R_GC p_C + t_GC",
                    gravity_camera_init_m_s2=self.gravity.tolist(), R_GC=self.R.tolist(),
                    quaternion_xyzw=self.q.tolist(), translation_m=[0., 0., 0.],
                    yaw_reference="none", origin="camera_init")


def check_config(run, source_run):
    params = run / "resolved.yaml"
    manifest = read_json(run / "parameters_manifest.json")
    digest = sha256(params)
    if manifest["run"] != source_run or manifest["resolved_sha256"] != digest:
        raise ValueError("会话参数快照与来源或 SHA256 不符")
    data = yaml.safe_load(params.read_bytes())["/**"]["ros__parameters"]
    if data.get("uav.gravity_align_en", data.get("uav", {}).get("gravity_align_en")) is not False:
        raise ValueError("外部对齐要求实际配置明确关闭算法内部 uav.gravity_align_en")
    return digest


def live_binding(run):
    run = Path(run).resolve()
    alg = read_json(run / "algorithm_identity.json")
    exe = Path(alg["executable"]).resolve()
    if alg["session_id"] != run.name or not exe.is_file() or sha256(exe) != alg["sha256"]:
        raise ValueError("算法身份文件与本次会话或执行文件不符")
    return dict(session_id=run.name, source_run=str(run), executable=str(exe),
                algorithm_sha256=alg["sha256"], resolved_sha256=check_config(run, str(run)))


class GravityEvents:
    """Two durable same-process records at most. No scanning other runs or latest log."""

    def __init__(self, run, binding):
        self.path = Path(run) / "gravity_events.jsonl"
        self.binding = binding
        self.transform = None
        self.initial = None
        self.ended = None
        self.raw_prefix = b""

    def refresh(self, final=False):
        try:
            with self.path.open("rb") as f:
                raw = f.read(8193)
        except FileNotFoundError:
            if self.initial or final:
                raise ValueError("缺少本会话结构化 IMU 初始化记录")
            return
        if len(raw) > 8192 or not raw.startswith(self.raw_prefix):
            raise ValueError("本会话重力事件被替换、截断或超出预期")
        new = raw[len(self.raw_prefix):]
        if new and not new.endswith(b"\n"):
            if final:
                raise ValueError("本会话重力事件末行不完整")
            new = new[:new.rfind(b"\n")+1]
        for line in new.splitlines(keepends=True):
            event = json.loads(line)
            if (any(event.get(k) != self.binding[k] for k in
                    ("session_id", "executable", "algorithm_sha256", "resolved_sha256"))
                    or event.get("segment") != 1 or event.get("pid", 0) <= 0):
                raise ValueError("初始化来源、算法或配置与本次会话身份不符")
            if event["event"] == "initialized" and self.initial is None and self.ended is None:
                if event.get("source") != "ImuProcess::Process2/stat.gravity":
                    raise ValueError("初始化重力字段来源不明")
                self.transform = GravityTransform(event["gravity_camera_init_m_s2"])
                self.initial = event
            elif (event["event"] == "segment_closed" and self.initial and not self.ended
                  and event["pid"] == self.initial["pid"]
                  and event.get("reason") == "reinitialization_requested"):
                self.ended = event["reason"]
            else:
                raise ValueError("意外的重复初始化或分段顺序")
            self.raw_prefix += line
        if final and self.initial is None:
            raise ValueError("本会话尚未完成 IMU 初始化")

    def evidence(self):
        return dict(kind="estimator_state", event_file=self.path.name,
                    source=self.initial["source"], algorithm_pid=self.initial["pid"],
                    initial_event_sha256=hashlib.sha256(self.raw_prefix.splitlines(True)[0]).hexdigest(),
                    segment=1, segment_end_reason=self.ended, numeric_precision="double/max_digits10")


_OLD_INIT = re.compile(r"IMU Initials: Gravity:\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)\s+[^;\s]+;")


def legacy_log(path):
    gravity = None
    with Path(path).open("r", encoding="utf-8", errors="replace") as f:
        for number, line in enumerate(f, 1):
            if "Gravity Alignment Starts" in line or (gravity is not None and "Reset ImuProcess" in line):
                raise ValueError("历史算法已内部对齐或在初始化后重置，单一变换不适用")
            if "IMU Initials: Gravity:" in line:
                match = _OLD_INIT.search(line)
                if gravity is not None or not match:
                    raise ValueError("历史会话的初始化次数或格式有歧义")
                gravity = GravityTransform([float(v) for v in match.groups()])
                init_line = number
    if gravity is None:
        raise ValueError("历史完整日志缺少 IMU 初始化")
    return gravity, init_line
