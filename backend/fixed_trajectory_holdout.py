"""Compare trajectories using identical native point records in the frozen regions."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from frame_io import frames_and_gravity, save_json
from heldout import evaluate
from loop_edges import EdgeBuilder
from verify import load_tum


def main(args):
    plan=json.loads((args.reference/"plan.json").read_text())
    frames,G,_=frames_and_gravity(Path(plan["parameters"]["index"]),Path(plan["parameters"]["gravity"]))
    raw=np.array([G@f.odom for f in frames])
    stamps,poses=load_tum(args.trajectory)
    times=(np.asarray(stamps)-stamps[0])*1e-9
    pairs=plan["holdout_pairs_indices"]
    used=sorted({k for i,j in pairs for k in (i,j-1,j,j+1) if 0<=k<len(frames)})
    query=(np.array([frames[i].stamp for i in used])-stamps[0])*1e-9
    if min(query)<times[0] or max(query)>times[-1]:
        raise ValueError("frozen sensor times outside the candidate trajectory")
    right=np.searchsorted(times,query)
    if np.any((times[right]!=query)&((times[right]-times[np.maximum(right-1,0)])>.25)):
        raise ValueError("missing dense poses around a frozen sensor time")
    after=raw.copy()
    after[used,:3,:3]=Slerp(times,Rotation.from_matrix(poses[:,:3,:3]))(query).as_matrix()
    after[used,:3,3]=np.column_stack([np.interp(query,times,poses[:,k,3]) for k in range(3)])
    builder=EdgeBuilder(frames,raw,np.array(plan["holdout_centers_xy_m"]),set(plan["heldout_frame_indices"]))
    save_json(args.out,evaluate(pairs,builder,raw,after))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("reference","trajectory","out"):
        p.add_argument(name,type=Path)
    main(p.parse_args())
