"""Validate row-preserving A-to-B/C reconstruction using archived per-frame counts and poses."""
import argparse
from pathlib import Path
import sys,json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("index","maps","repo","out"):
        p.add_argument(key,type=Path)
    a=p.parse_args()
    sys.path.insert(0,str(a.repo/"tools"))
    from evaluate_z_geometry import binary_map
    frames=pd.read_csv(a.index,usecols=["id","stamp_ns","points","color_points"])
    entries=[(a.maps/"pgo","before"),(a.maps/"pgo","after"),(a.maps/"structure","after")]
    poses=[np.loadtxt(folder/f"trajectory_{v}.tum") for folder,v in entries]
    for tr in poses:
        assert len(tr)==len(frames) and np.max(abs(tr[:,0]-frames.stamp_ns.to_numpy()*1e-9))<1e-6
    rot=[Rotation.from_quat(pose[:,4:]).as_matrix() for pose in poses]
    records={}
    for name,col in [("map","points"),("map_rgb","color_points")]:
        maps=[binary_map(folder/f"{name}_{v}.pcd")[0] for folder,v in entries]
        ends=np.r_[0,np.cumsum(frames[col].to_numpy())]
        assert all(len(m)==ends[-1] for m in maps)
        for k,stage in [(1,"B"),(2,"C")]:
            residual=np.empty(ends[-1],dtype=np.float64)
            maxima=[]
            for i,(start,end) in enumerate(zip(ends[:-1],ends[1:])):
                local=(maps[0][start:end,:3]-poses[0][i,1:4]) @ rot[0][i]
                expected=local @ rot[k][i].T + poses[k][i,1:4]
                error=np.linalg.norm(expected-maps[k][start:end,:3],axis=1)*1e6
                residual[start:end]=error
                maxima.append(float(error.max(initial=0)))
            frames[f"{name}_{stage}_max_um"]=maxima
            records[f"{name}_{stage}"]=dict(n=len(residual),p50_um=float(np.median(residual)),p95_um=float(np.quantile(residual,.95)),
                max_um=float(residual.max()),rms_um=float(np.sqrt(np.mean(residual**2))))
    frames.to_csv(a.out/"frame_readback.csv",index=False,float_format="%.8g")
    (a.out/"readback.json").write_text(json.dumps(records,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
