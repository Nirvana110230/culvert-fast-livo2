"""PCD and TUM IO for the native ROS layouts. Points stream to disk in bounded blocks."""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from array import array
import os
from pathlib import Path

import numpy as np

from gravity import CAMERA, GRAVITY, unit_quaternion

XYZ = ("x", "y", "z")
NORMAL = ("normal_x", "normal_y", "normal_z")
POINT_VIEW = ("vp_x", "vp_y", "vp_z")
IDENTITY_VIEW = (0., 0., 0., 1., 0., 0., 0.)
HEADER_BYTES = 2048
CHUNK_POINTS = 32768
ROS_NUMBERS = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
PCD_KINDS = {"F": "f", "I": "i", "U": "u"}
FILES = {"rgb": "map_rgb.pcd", "geometry": "map_geometry.pcd", "poses": "trajectory.tum"}


def parts(dtype):
    for name in dtype.names:
        field = dtype.fields[name][0]
        base, shape = field.subdtype or (field, ())
        yield name, base, int(np.prod(shape)) if shape else 1


def validate_layout(dtype):
    names = dtype.names
    if not names or not all(n in names for n in XYZ):
        raise ValueError("点云缺少 XYZ")
    for group in (XYZ, NORMAL, POINT_VIEW):
        if any(n in names for n in group) and not all(n in names for n in group):
            raise ValueError("点云的几何分量不完整")
        if all(n in names for n in group):
            for n in group:
                f = dtype.fields[n][0]
                if f.subdtype or f.kind != "f" or f.itemsize not in (4, 8):
                    raise ValueError("几何分量必须为浮点标量")


def make_dtype(fields):
    entries = []
    for name, kind, count in fields:
        if count < 1 or not name or any(c.isspace() for c in name):
            raise ValueError("无效的点云字段")
        entries.append((name, kind) if count == 1 else (name, kind, (count,)))
    dtype = np.dtype(entries)
    validate_layout(dtype)
    return dtype


@dataclass
class Header:
    dtype: np.dtype
    width: int
    height: int
    viewpoint: tuple
    offset: int
    frame: str

    @property
    def count(self):
        return self.width * self.height


def read_header(stream):
    fields = {}
    frame = ""
    while stream.tell() < 65536:
        line = stream.readline(8192)
        if not line or not line.endswith(b"\n"):
            raise ValueError("PCD 文件头截断")
        tokens = line.decode("ascii").split()
        if tokens[:2] == ["#", "frame_id"]:
            frame = tokens[2]
        if not tokens or tokens[0].startswith("#"):
            continue
        key, *values = tokens
        if key in fields:
            raise ValueError("PCD 文件头字段重复")
        fields[key] = values
        if key == "DATA":
            break
    if fields.get("DATA") != ["binary"]:
        raise ValueError("仅支持本运行链的无压缩 binary PCD")
    names, sizes, types = (fields[k] for k in ("FIELDS", "SIZE", "TYPE"))
    counts = fields.get("COUNT", ["1"] * len(names))
    if not len(names) == len(sizes) == len(types) == len(counts):
        raise ValueError("PCD 字段数量不一致")
    dtype = make_dtype((n, "<" + PCD_KINDS[t] + s, int(c))
                       for n, s, t, c in zip(names, sizes, types, counts))
    width = int(fields["WIDTH"][0])
    height = int(fields["HEIGHT"][0])
    if width < 0 or height < 1 or int(fields["POINTS"][0]) != width * height:
        raise ValueError("PCD 点数与尺寸不符")
    viewpoint = tuple(map(float, fields.get("VIEWPOINT", IDENTITY_VIEW)))
    if len(viewpoint) != 7 or not np.isfinite(viewpoint).all():
        raise ValueError("PCD 采集视点无效")
    unit_quaternion([*viewpoint[4:], viewpoint[3]])
    return Header(dtype, width, height, viewpoint, stream.tell(), frame)


def pcd_info(path):
    with Path(path).open("rb") as f:
        header = read_header(f)
        if os.fstat(f.fileno()).st_size != header.offset + header.count * header.dtype.itemsize:
            raise ValueError("PCD 字节长度与已记录点数不符")
        return header


def blocks(path, header):
    with Path(path).open("rb") as f:
        f.seek(header.offset)
        left = header.count
        while left:
            n = min(left, CHUNK_POINTS)
            buf = f.read(n * header.dtype.itemsize)
            if len(buf) != n * header.dtype.itemsize:
                raise ValueError("PCD 数据截断")
            yield np.frombuffer(buf, header.dtype)
            left -= n


def header_bytes(dtype, count, viewpoint, frame, height=1):
    fields = list(parts(dtype))
    if count % height:
        raise ValueError("PCD 非整数行宽")
    lines = ["# .PCD v0.7", "# frame_id " + frame, "VERSION 0.7",
             "FIELDS " + " ".join(n for n, _, _ in fields),
             "SIZE " + " ".join(str(b.itemsize) for _, b, _ in fields),
             "TYPE " + " ".join(b.kind.upper() for _, b, _ in fields),
             "COUNT " + " ".join(str(c) for _, _, c in fields),
             "WIDTH " + str(count // height), "HEIGHT " + str(height),
             "VIEWPOINT " + " ".join(format(v, ".17g") for v in viewpoint), "POINTS " + str(count)]
    prefix = ("\n".join(lines) + "\n").encode("ascii")
    tail = b"DATA binary\n"
    pad = HEADER_BYTES - len(prefix) - len(tail)
    if pad < 2:
        raise ValueError("PCD 字段表过大")
    return prefix + b"#" + b" " * (pad - 2) + b"\n" + tail


class PCDStream:
    def __init__(self, path, dtype, frame, viewpoint=IDENTITY_VIEW):
        self.dtype = dtype
        self.viewpoint = viewpoint
        self.frame = frame
        self.count = 0
        self.file = Path(path).open("x+b")
        self._header()

    def _header(self, height=1):
        self.file.seek(0)
        self.file.write(header_bytes(self.dtype, self.count, self.viewpoint, self.frame, height))
        self.file.flush()

    def append(self, data):
        if data.dtype != self.dtype or data.ndim != 1:
            raise ValueError("同一会话点云布局发生变化")
        self.file.seek(0, 2)
        self.file.write(data.tobytes())
        self.count += len(data)
        self._header()

    def close(self, height=1):
        if self.file.closed:
            return
        try:
            self._header(height)
            os.fsync(self.file.fileno())
        finally:
            self.file.close()


def extract_vectors(records, names=XYZ):
    return np.column_stack([records[n] for n in names])


def transform_records(records, transform):
    result = records.copy()
    for names in (XYZ, NORMAL, POINT_VIEW):
        if all(n in records.dtype.names for n in names):
            # Translation is zero. Normals are rotated, never translated.
            with np.errstate(invalid="ignore"):
                xyz = transform.vectors(extract_vectors(records, names))
            for axis, name in enumerate(names):
                result[name] = xyz[:, axis]
    return result


def convert_pcd(source, dest, transform):
    meta = pcd_info(source)
    if meta.frame not in ("", CAMERA):
        raise ValueError("输入点云已重力对齐或原始坐标不明")
    writer = PCDStream(dest, meta.dtype, GRAVITY, transform.viewpoint(meta.viewpoint))
    try:
        for block in blocks(source, meta):
            writer.append(transform_records(block, transform))
    finally:
        writer.close(height=meta.height if writer.count == meta.count else 1)
    return writer.count


def ros_records(msg):
    """Remove ROS/PCL byte padding and preserve every named field, count and bit pattern."""
    if msg.header.frame_id != CAMERA:
        raise ValueError("原生点云坐标不是 camera_init")
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError("PointCloud2 步长无效")
    buf = memoryview(msg.data).cast("B")
    if len(buf) != msg.row_step * msg.height:
        raise ValueError("PointCloud2 数据长度不符")
    fields = [f for f in msg.fields if f.name != "_"]
    dtype = make_dtype((f.name, "<" + ROS_NUMBERS[f.datatype], int(f.count)) for f in fields)
    out = np.empty((msg.height, msg.width), dtype)
    endian = ">" if msg.is_bigendian else "<"
    for f in fields:
        scalar = np.dtype(endian + ROS_NUMBERS[f.datatype])
        if f.offset < 0 or f.offset + scalar.itemsize*f.count > msg.point_step:
            raise ValueError("PointCloud2 字段超出记录边界")
        shape = (msg.height, msg.width) if f.count == 1 else (msg.height, msg.width, f.count)
        stride = (msg.row_step, msg.point_step) if f.count == 1 else (msg.row_step, msg.point_step, scalar.itemsize)
        out[f.name] = np.ndarray(shape, scalar, buffer=buf, offset=f.offset, strides=stride)
    return out.reshape(-1)


def ros_cloud(records, frame, stamp):
    from sensor_msgs.msg import PointCloud2, PointField
    types = {np.dtype(v).str[-2:]: k for k, v in ROS_NUMBERS.items()}
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.height, msg.width = 1, len(records)
    msg.fields = [PointField(name=n, offset=records.dtype.fields[n][1],
                             datatype=types[b.str[-2:]], count=c)
                  for n, b, c in parts(records.dtype)]
    msg.point_step = records.dtype.itemsize
    msg.row_step = len(records) * msg.point_step
    msg.is_dense = all(np.isfinite(records[n]).all() for n in XYZ)
    payload = array("B")
    payload.frombytes(memoryview(np.ascontiguousarray(records)).cast("B"))
    msg.data = payload
    return msg


def tum_rows(path):
    with Path(path).open("r", encoding="ascii") as f:
        for i, line in enumerate(f, 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            parts_ = line.split()
            if len(parts_) != 8:
                raise ValueError(f"TUM 第 {i} 行列数无效")
            try:
                stamp = Decimal(parts_[0])
            except InvalidOperation as exc:
                raise ValueError(f"TUM 第 {i} 行时间无效") from exc
            pose = np.asarray(parts_[1:], dtype=np.float64)
            if not stamp.is_finite() or not np.isfinite(pose).all():
                raise ValueError(f"TUM 第 {i} 行有非有限值")
            unit_quaternion(pose[3:])
            yield parts_[0], pose[:3], pose[3:]


def write_pose(file, stamp, position, quaternion):
    file.write(stamp + " " + " ".join(format(v, ".17g") for v in (*position, *quaternion)) + "\n")


def convert_tum(source, dest, transform):
    count = 0
    with Path(dest).open("x", encoding="ascii", newline="\n") as out:
        for stamp, p, q in tum_rows(source):
            write_pose(out, stamp, *transform.pose(p, q))
            count += 1
        out.flush()
        os.fsync(out.fileno())
    return count
