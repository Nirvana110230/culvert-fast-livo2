"""Read back every output point and independently check the archived historical body transform."""
import argparse
import csv
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from frame_io import read_pcd, rgb_bits, save_json


def map_view(path):
    with path.open("rb") as f:
        header={}
        for _ in range(30):
            key,*values=f.readline().decode("ascii").split()
            header[key]=values
            if key=="DATA":
                break
        start=f.tell()
    count=int(header["POINTS"][0])
    assert header["DATA"]==["binary"] and header["SIZE"]==["4"]*4
    assert path.stat().st_size==start+16*count
    return np.memmap(path,dtype="<f4",mode="r",offset=start,shape=(count,4))


def load_tum(path):
    stamps=[]
    poses=[]
    with path.open() as f:
        for line in f:
            stamp,*x=line.split()
            s,ns=stamp.split(".")
            stamps.append(int(s)*10**9+int(ns))
            T=np.eye(4)
            T[:3,:3]=Rotation.from_quat(list(map(float,x[3:]))).as_matrix()
            T[:3,3]=list(map(float,x[:3]))
            poses.append(T)
    return stamps,np.array(poses)


def digest_file(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        while block:=f.read(1<<20):
            h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--index",type=Path,required=True)
    p.add_argument("--result",type=Path,required=True)
    a=p.parse_args()
    with a.index.open(newline="") as f:
        rows=list(csv.DictReader(f))
    stamps=[int(x["stamp_ns"]) for x in rows]
    maps={}
    for name in ("before","after"):
        ns,T=load_tum(a.result/f"trajectory_{name}.tum")
        assert ns==stamps
        for typ,key,prefix in (("geometry","cloud","map_"),("rgb","color_cloud","map_rgb_")):
            files=[r.get(key) for r in rows]
            if not any(files):
                continue
            path=a.result/f"{prefix}{name}.pcd"
            matrix=map_view(path)
            cursor=0
            maxerror=0.
            for k,rel in enumerate(files):
                if not rel:
                    continue
                cloud,data=read_pcd(a.index.parent/rel)
                observed=matrix[cursor:cursor+len(cloud)]
                expected=T[k,:3,:3]@cloud.T+T[k,:3,3,None]
                if len(cloud):
                    maxerror=max(maxerror,float(np.max(np.abs(observed[:,:3].T-expected))))
                if typ=="rgb":
                    assert np.array_equal(rgb_bits(data),observed[:,3].view("<u4"))
                elif "intensity" in data.dtype.names:
                    assert np.array_equal(data["intensity"].astype("<f4"),observed[:,3],equal_nan=True)
                cursor+=len(cloud)
            assert len(matrix)==cursor and maxerror<.00015
            maps[path.name]={"points_verified":cursor,"maximum_xyz_abs_error_m":maxerror,"sha256":digest_file(path)}
    ns,C=load_tum(a.result/"map_to_odom.tum")
    assert ns==stamps
    T=np.tile(np.eye(4),(len(rows),1,1))
    for k,r in enumerate(rows):
        T[k,:3,:3]=Rotation.from_quat([float(r[v]) for v in ("qx","qy","qz","qw")]).as_matrix()
        T[k,:3,3]=[float(r[v]) for v in "xyz"]
    _,opt=load_tum(a.result/"trajectory_after.tum")
    maximum=float(np.max(np.abs(C@T-opt)))
    assert maximum<.00001
    report={"passed":True,"frames":len(rows),"maps":maps,"T_map_odom_times_T_odom_body_max_abs":maximum}
    save_json(a.result/"verification.json",report)
    print(report)


if __name__=="__main__":
    main()
