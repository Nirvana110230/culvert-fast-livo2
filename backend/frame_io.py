"""Body-frame PCD and gravity-frame trajectory I/O for the offline sidecar."""
import csv
import json
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from gravity import GravityTransform


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf8")


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def rigid_inverse(pose):
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ pose[:3, 3]
    return result


def voxel(points, size):
    if not len(points):
        return points
    _, first = np.unique(np.floor(points / size).astype(np.int32), axis=0, return_index=True)
    return points[first]


@dataclass
class Frame:
    id: int
    stamp: int
    odom: np.ndarray
    cloud: Path
    color: Path | None


def session_gravity(index, event_path):
    events = [json.loads(line) for line in event_path.read_text().splitlines() if line.strip()]
    init = [r for r in events if r.get("event") == "initialized"]
    if len(init) != 1 or len({r.get("session_id") for r in events}) != 1:
        raise ValueError("one gravity initialization in one original session is required")
    meta = index.parent / "metadata.txt"
    if meta.is_file():
        tags = dict(line.split("=",1) for line in meta.read_text().splitlines() if "=" in line)
        expected = {"pose":"T_odom_imu","cloud_frame":"imu","odom_frame":"camera_init",
                    "FAST_LIVO2_SESSION_ID":init[0]["session_id"],
                    "FAST_LIVO2_CONFIG_SHA256":init[0]["resolved_sha256"],
                    "FAST_LIVO2_ALGORITHM_SHA256":init[0]["algorithm_sha256"]}
        if any(tags.get(k)!=v for k,v in expected.items()):
            raise ValueError("gravity event and keyframe source identity or frame convention mismatch")
    g = GravityTransform(init[0]["gravity_camera_init_m_s2"])
    G = np.eye(4)
    G[:3, :3] = g.R
    return G, {**g.record(), "session_id": init[0].get("session_id"), "event_file": str(event_path)}


def frame_from_row(index,row):
    p = np.eye(4)
    q = np.array([float(row[n]) for n in ("qx", "qy", "qz", "qw")])
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError("invalid body quaternion")
    p[:3, :3] = Rotation.from_quat(q).as_matrix()
    p[:3, 3] = [float(row[n]) for n in ("x", "y", "z")]
    if not np.isfinite(p).all():
        raise ValueError("nonfinite body pose")
    color = row.get("color_cloud", "").strip()
    return Frame(int(row["id"]),int(row["stamp_ns"]),p,index.parent/row["cloud"],index.parent/color if color else None)


def frames_and_gravity(index, event_path):
    G,record=session_gravity(index,event_path)
    frames = []
    ids = set()
    with index.open(newline="", encoding="utf-8-sig") as inp:
        for row in csv.DictReader(inp):
            stamp = int(row["stamp_ns"])
            ident = int(row["id"])
            if ident in ids or (frames and stamp <= frames[-1].stamp):
                raise ValueError("keyframe id must be unique and timestamps strictly increasing")
            ids.add(ident)
            frames.append(frame_from_row(index,row))
    if len(frames) < 3:
        raise ValueError("at least 3 frames required")
    return frames, G, record


def read_pcd(path):
    with path.open("rb") as f:
        header = {}
        while True:
            line = f.readline(8192)
            if not line or len(line) >= 8192:
                raise ValueError(f"invalid PCD header in {path}")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        if header["DATA"] != ["binary"]:
            raise ValueError(f"expected binary PCD in {path}")
        fields = header["FIELDS"]
        count = list(map(int, header.get("COUNT", [1] * len(fields))))
        sizes = {"F": "f", "I": "i", "U": "u"}
        dtype = [(name if name != "_" else f"_pad{idx}", "<" + sizes[k] + s, *((n,) if n != 1 else ()))
                 for idx, (name, k, s, n) in enumerate(zip(fields, header["TYPE"], header["SIZE"], count))]
        pts = int(header["POINTS"][0])
        data = np.fromfile(f, dtype=np.dtype(dtype), count=pts)
        if len(data) != pts or f.read(1):
            raise ValueError(f"PCD payload size mismatch in {path}")
    xyz = np.column_stack([data[n] for n in "xyz"])
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], data[finite]


def rgb_bits(data):
    for name in ("rgb", "rgba"):
        if name in data.dtype.names:
            return data[name].astype("<f4", copy=False).view("<u4") if data[name].dtype.kind == "f" else data[name].astype("<u4")
    if {"r", "g", "b"}.issubset(data.dtype.names):
        return (data["r"].astype("<u4") << 16) | (data["g"].astype("<u4") << 8) | data["b"].astype("<u4")
    raise ValueError("missing packed rgb/rgba or r,g,b fields in declared color cloud")


class MapWriter:
    def __init__(self, path, field):
        self.file = path.open("xb")
        self.field = field
        self.n = 0
        self.header(0)

    def header(self, count):
        self.file.seek(0)
        self.file.write((f"VERSION 0.7\nFIELDS x y z {self.field}\nSIZE 4 4 4 4\nTYPE F F F F\n"
                         f"COUNT 1 1 1 1\nWIDTH {count:20d}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                         f"POINTS {count:20d}\nDATA binary\n").encode("ascii"))

    def append(self, points, scalar):
        packed = np.empty((len(points), 4), dtype="<f4")
        packed[:, :3] = points
        packed[:, 3] = scalar
        packed.tofile(self.file)
        self.n += len(points)

    def close(self):
        self.header(self.n)
        self.file.close()


def rebuild(out, frames, before, after):
    geometry = [MapWriter(out / f"map_{s}.pcd", "intensity") for s in ("before", "after")]
    rgb = [MapWriter(out / f"map_rgb_{s}.pcd", "rgb") for s in ("before", "after")] if any(f.color for f in frames) else []
    ncolor = 0
    has_intensity = 0
    try:
        for frame, old, new in zip(frames, before, after):
            pts, data = read_pcd(frame.cloud)
            intensity = data["intensity"] if "intensity" in data.dtype.names else 0.
            has_intensity += int("intensity" in data.dtype.names)
            for writer, pose in zip(geometry, (old, new)):
                writer.append(transform(pts, pose), intensity)
            if frame.color:
                cp, data = read_pcd(frame.color)
                bits = rgb_bits(data).view("<f4")
                for writer, pose in zip(rgb, (old, new)):
                    writer.append(transform(cp, pose), bits)
                ncolor += 1
    finally:
        for writer in geometry + rgb:
            writer.close()
    return {"geometry_points_each": geometry[0].n, "intensity_frames": has_intensity, "color_frames": ncolor,
            "color_points_each": rgb[0].n if rgb else 0, "geometry_frames": len(frames)}


def write_tum(path, frames, poses):
    with path.open("w") as out:
        for frame, p in zip(frames, poses):
            values = [*p[:3, 3], *Rotation.from_matrix(p[:3, :3]).as_quat()]
            out.write(f"{frame.stamp // 10**9}.{frame.stamp % 10**9:09d} " + " ".join(f"{v:.10f}" for v in values) + "\n")


def dense_trajectory(path, out, frames, G, after, windows, origin):
    if not path.is_file():
        return {}
    stamps, xyz, quat = [], [], []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row["stage"] != "LIO":
                continue
            ns = int(row["stamp_ns"])
            if stamps and ns <= stamps[-1]:
                raise ValueError("non-monotonic dense LIO measurements")
            stamps.append(ns)
            xyz.append([float(row[k]) for k in "xyz"])
            quat.append([float(row[k]) for k in ("qx","qy","qz","qw")])
    if not stamps:
        return {}
    t=np.array([(n-origin)*1e-9 for n in stamps])
    key=np.array([(f.stamp-origin)*1e-9 for f in frames])
    correction=np.array([pose@rigid_inverse(frame.odom) for pose,frame in zip(after,frames)])
    clipped=np.clip(t,key[0],key[-1])
    rot=Slerp(key,Rotation.from_matrix(correction[:,:3,:3]))(clipped)
    trans=np.column_stack([np.interp(clipped,key,correction[:,i,3]) for i in range(3)])
    old_xyz=transform(np.array(xyz),G)
    new_xyz=rot.apply(np.array(xyz))+trans
    body=Rotation.from_quat(quat)
    old_q=(Rotation.from_matrix(G[:3,:3])*body).as_quat()
    new_q=(rot*body).as_quat()
    record={"count":len(t),"source_stage":"LIO","interpolation":"linear translation and SLERP on T_map_odom; nearest outside keyframe range",
            "extrapolated_by_constant_correction":int(np.count_nonzero(t!=clipped))}
    for name,coords,q in (("before",old_xyz,old_q),("after",new_xyz,new_q)):
        with (out/f"trajectory_dense_{name}.tum").open("x") as f:
            for stamp,v,o in zip(stamps,coords,q):
                f.write(f"{stamp//10**9}.{stamp%10**9:09d} "+" ".join(f"{a:.9f}" for a in (*v,*o))+"\n")
        if windows:
            subsets=[coords[(t>=lo)&(t<=hi)] for lo,hi in np.reshape(windows,(2,2))]
            med=[np.median(x,axis=0) for x in subsets]
            delta=med[1]-med[0]
            record[name]={"window_medians_xyz_m":[x.tolist() for x in med],"window_counts":[len(x) for x in subsets],
                          "delta_xyz_m":delta.tolist(),"separation_m":float(np.linalg.norm(delta)),
                          "horizontal_separation_m":float(np.linalg.norm(delta[:2])),
                          "window_stationarity_radius_p90_m":[float(np.quantile(np.linalg.norm(v-m,axis=1),.9)) for v,m in zip(subsets,med)]}
    return record
