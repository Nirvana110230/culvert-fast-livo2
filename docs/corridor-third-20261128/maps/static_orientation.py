"""Compare orientation at the two static windows using archived sensor time."""
from argparse import ArgumentParser
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    p=ArgumentParser(description=__doc__)
    p.add_argument("maps",type=Path);p.add_argument("out",type=Path)
    args=p.parse_args()
    origin=json.loads((args.maps/"imu.json").read_text())["origin_ns"]
    result={}
    for stage,folder,side in [("A","pgo","before"),("B","pgo","after"),("C","structure","after")]:
        rows=np.loadtxt(args.maps/f"{folder}/trajectory_dense_{side}.tum",dtype=str)
        ns=np.array([int(s.replace(".","")) for s in rows[:,0]],dtype=np.int64)
        times=(ns-origin)*1e-9
        poses=Rotation.from_quat(rows[:,4:].astype(float))
        first=poses[(times>=5)&(times<=25)].mean()
        last=poses[(times>=2059)&(times<=2079)].mean()
        delta=last*first.inv()
        result[stage]=dict(angle_deg=float(np.rad2deg(delta.magnitude())),
                           delta_fixed_xyz_deg=delta.as_euler("xyz",degrees=True).tolist())
    (args.out/"static_orientation.json").write_text(json.dumps(result,indent=2)+"\n")


if __name__=="__main__":
    main()
