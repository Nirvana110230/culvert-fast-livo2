"""Score the actual broadcast correction and the retrospective reconstruction separately."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from frame_io import frames_and_gravity, rigid_inverse, save_json
from verify import load_tum


def summarize(x,windows):
    means=[np.median(x[use],axis=0) for use in windows]
    delta=means[1]-means[0]
    return {"p0_windows_median_xyz_m":[v.tolist() for v in means],"p0_delta_xyz_m":delta.tolist(),
            "p0_3d_m":float(np.linalg.norm(delta)),"p0_xy_m":float(np.linalg.norm(delta[:2])),
            "z_range_m":float(np.ptp(x[:,2]))}


def main():
    p=argparse.ArgumentParser()
    for k in ("index","gravity","result","output"):
        p.add_argument("--"+k,type=Path,required=True)
    a=p.parse_args()
    fr,G,_=frames_and_gravity(a.index,a.gravity)
    online=json.loads((a.result/"online_summary.json").read_text())
    rows=list(csv.DictReader((a.result/"issued.csv").open()))
    ks=np.array([int(r["stamp_ns"]) for r in rows],dtype=np.int64)
    assert np.array_equal(ks,[f.stamp for f in fr])
    rotations=Rotation.from_quat([[float(q[n]) for n in ("qx","qy","qz","qw")] for q in rows])
    trans=np.array([[float(q[n]) for n in ("tx","ty","tz")] for q in rows])
    stamps,poses=load_tum(a.result/"trajectory_after.tum")
    assert np.array_equal(ks,stamps)
    corrected=np.array([P@rigid_inverse(f.odom) for P,f in zip(poses,fr)])
    states=[r for r in csv.DictReader((a.index.parent/"states.csv").open()) if r["stage"]=="LIO"]
    ns=np.array([int(r["stamp_ns"]) for r in states],dtype=np.int64)
    raw=np.array([[float(r[k]) for k in "xyz"] for r in states])
    t=(ns-ks[0])*1e-9
    key=(ks-ks[0])*1e-9
    use=[(t>=lo)&(t<=hi) for lo,hi in ((10,60),(1775,1825))]
    i=np.maximum(0,np.searchsorted(ks,ns,side="right")-1)
    live=rotations[i].apply(raw)+trans[i]
    clipped=np.clip(t,key[0],key[-1])
    retrospective=Slerp(key,Rotation.from_matrix(corrected[:,:3,:3]))(clipped).apply(raw)
    retrospective+=np.column_stack([np.interp(clipped,key,corrected[:,d,3]) for d in range(3)])
    actual_edges=json.loads((a.result/"edges.json").read_text())
    assert online["active_edges"]==[[e["target_id"],e["source_id"]] for e in actual_edges]
    stats={"caller":"second corridor taped P0; post-run measurement, not a constraint supplied to streaming detector",
           "keyframes":len(fr),"dense_lio_count":len(ns),"static_window_counts":[int(x.sum()) for x in use],
           "raw":summarize(raw@G[:3,:3].T,use),"issued_zero_order_hold":summarize(live,use),
           "retrospective":summarize(retrospective,use),"active_edges":online["active_edges"],
           "final_revisions":online["revisions"],"elapsed_s":online["elapsed_s"],"peak_rss_mib":online["peak_rss_MiB"],
           "rebuild":online["rebuild"],"query_seconds_p50_p95_max":np.quantile(online["query_wall_s"],[.5,.95,1]).tolist(),
           "frozen_holds_not_excluded_online":True}
    save_json(a.output,stats)


if __name__=="__main__":
    main()
