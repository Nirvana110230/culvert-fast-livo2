"""Refine a completed offline graph with native planes and sensor-time directions."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from frame_io import dense_trajectory, frames_and_gravity, rebuild, rigid_inverse, save_json, write_tum
from heldout import evaluate
from loop_edges import EdgeBuilder
from run import describe_track
from shared_planes import joint_graph, local_patches
from verify import load_tum


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base",type=Path,required=True,help="completed run.py graph")
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--imu",type=Path)
    parser.add_argument("--validate-directions",action="store_true",help="reserve every fifth velocity-balance observation")
    parser.add_argument("--directions-only",action="store_true",help="ablation: evaluate plane patches without adding them to the graph")
    parser.add_argument("--north-of",type=float,help="longest contiguous subgraph north of this map y")
    parser.add_argument("--index",type=Path,help="optional relocated keyframe index")
    parser.add_argument("--gravity",type=Path,help="matching relocated initialization events")
    args=parser.parse_args()
    started=time.monotonic()
    base,out=args.base,args.out
    plan=json.loads((base/"plan.json").read_text())
    index=args.index or Path(plan["parameters"]["index"])
    events=args.gravity or Path(plan["parameters"]["gravity"])
    frames,G,_=frames_and_gravity(index,events)
    if hashlib.sha256(index.read_bytes()).hexdigest()!=plan["index_sha256"]:
        raise ValueError("graph and native keyframe index differ")
    stamps,initial=load_tum(base/"trajectory_after.tum")
    if stamps!=[f.stamp for f in frames]:
        raise ValueError("graph and native keyframe stamps differ")
    out.mkdir(parents=True,exist_ok=False)
    raw=np.array([G@f.odom for f in frames])
    selected=np.arange(len(frames))
    if args.north_of is not None:
        ids=selected[initial[:,1,3]>=args.north_of]
        selected=max(np.split(ids,np.flatnonzero(np.diff(ids)>1)+1),key=len)
        if len(selected)<12:
            raise ValueError("insufficient subgraph")
    origin=plan["time_origin_ns"]
    times=(np.array(stamps)-origin)*1e-9
    builder=EdgeBuilder(frames,raw,np.array(plan["holdout_centers_xy_m"]),set(plan["heldout_frame_indices"]))
    groups=local_patches(builder,initial,selected)
    directions=[]
    if args.imu:
        data=json.loads(args.imu.read_text())
        with (index.parent/"states.csv").open() as stream:
            rows=[r for r in csv.DictReader(stream) if r["stage"]=="LIO"]
        if data["origin_ns"]!=int(rows[0]["stamp_ns"]):
            raise ValueError("IMU and trajectory sensor-time origins differ")
        state_t=np.array([(int(r["stamp_ns"])-origin)*1e-9 for r in rows])
        interp=Slerp(state_t,Rotation.from_quat([[float(r[q]) for q in ("qx","qy","qz","qw")] for r in rows]))
        for entry in data["directions"]:
            t=entry["t"]+(data["origin_ns"]-origin)*1e-9
            if times[selected[0]]<=t<=times[selected[-1]]:
                i=int(np.argmin(abs(times-t)))
                body=frames[i].odom[:3,:3].T@interp(t).apply(entry["body"])
                directions.append((i,body,entry["kind"]))
    factors=json.loads((base/"factors.json").read_text())
    reserved=[]
    if args.validate_directions:
        moving=[d for d in directions if d[2]=="velocity_balance"]
        reserved=moving[2::5]
        directions=[d for d in directions if d[2]=="stationary"]+[d for k,d in enumerate(moving) if k%5!=2]
    after,report,associations=joint_graph(frames,initial,factors,selected,groups,directions,not args.directions_only)
    if not args.directions_only:
        save_json(out/"shared_plane_associations.json",associations)
    if reserved:
        def errors(poses):
            degrees=[np.degrees(np.arccos(np.clip((poses[i-selected[0],:3,:3]@body)[2],-1,1)))
                     for i,body,_ in reserved]
            return np.quantile(degrees,[.5,.95]).tolist()
        report["reserved_direction_deg"] = dict(count=len(reserved),before_p50_p95=errors(initial[selected]),
                                               after_p50_p95=errors(after))
    report.update(base=str(base),imu=str(args.imu),first=int(selected[0]),last=int(selected[-1]),
                  initial_track=describe_track(initial[selected]),after=describe_track(after))
    write_tum(out/"trajectory_after.tum",[frames[i] for i in selected],after)
    report["optimization_and_association_s"]=time.monotonic()-started
    if args.north_of is None:
        write_tum(out/"trajectory_before.tum",frames,raw)
        corrections=[T@rigid_inverse(f.odom) for f,T in zip(frames,after)]
        write_tum(out/"map_to_odom.tum",frames,corrections)
        save_json(out/"map_to_odom.json",dict(parent="map_gravity",child="camera_init",last_T_map_odom=corrections[-1].tolist()))
        report["dense_lio"]=dense_trajectory(index.parent/"states.csv",out,frames,G,after,plan["parameters"]["p0_windows"],origin)
        save_json(out/"plan.json",plan | {"structure_base":str(base),"direction_observations":str(args.imu)})
        save_json(out/"holdout.json",evaluate(plan["holdout_pairs_indices"],builder,raw,after))
        t=time.monotonic()
        report["maps"]=rebuild(out,frames,raw,after)
        report["rebuild_s"]=time.monotonic()-t
    report.update(keyframes=len(selected),wall_clock_s=time.monotonic()-started,
                  peak_rss_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    save_json(out/"summary.json",report)
    print(json.dumps(report))


if __name__=="__main__":
    main()
