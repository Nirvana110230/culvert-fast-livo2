"""Measure native patch continuity at the documented round-two northern junction."""
import argparse
import json
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from frame_io import frames_and_gravity, transform, save_json
from loop_edges import EdgeBuilder
from shared_planes import local_patches
from verify import load_tum


def main(base, result, out):
    plan=json.loads((base/"plan.json").read_text())
    frames,G,_=frames_and_gravity(Path(plan["parameters"]["index"]),Path(plan["parameters"]["gravity"]))
    raw=np.array([G@f.odom for f in frames])
    _,old=load_tum(base/"trajectory_after.tum")
    _,new=load_tum(result/"trajectory_after.tum")
    builder=EdgeBuilder(frames,raw,np.array(plan["holdout_centers_xy_m"]),set(plan["heldout_frame_indices"]))
    groups=local_patches(builder,old,np.arange(len(old)))
    xy=old[:,:2,3]
    main=(abs(xy[:,0]-10)<2.5)&(xy[:,1]>70)&(xy[:,1]<82)
    u=np.array([41.,-38.])/np.hypot(41.,38.)
    delta=xy-[16,84]
    upper=(delta@u>0)&(delta@u<18)&(abs(delta@np.array([-u[1],u[0]]))<2.5)
    records=[]
    floor_links=defaultdict(set)
    values=defaultdict(lambda:[[],[]])
    for j,g in enumerate(groups):
        x,y,_=g["center"]
        if not (3<x<28 and 72<y<95):
            continue
        train=[(i,p) for k,(i,p) in enumerate(g["obs"]) if k%5!=2]
        held=[(i,p) for k,(i,p) in enumerate(g["obs"]) if k%5==2]
        ids=[i for i,_ in train]
        if g["kind"]=="floor":
            for i in ids:
                floor_links[("frame",i)].add(("plane",j))
                floor_links[("plane",j)].add(("frame",i))
        scores=[]
        for k,poses in enumerate((old,new)):
            fit=np.vstack([transform(p,poses[i]) for i,p in train])
            c=fit.mean(0)
            n=np.linalg.eigh((fit-c).T@(fit-c))[1][:,0]
            e=np.concatenate([(transform(p,poses[i])-c)@n for i,p in held])
            values[g["kind"]][k].extend(e)
            scores.append(float(np.sqrt(np.mean(e**2))))
        records.append(dict(id=j,kind=g["kind"],anchor=g["center"].tolist(),main_ids=[i for i in ids if main[i]],
            upper_ids=[i for i in ids if upper[i]],training_ids=ids,held_ids=[i for i,_ in held],rms_old_new_m=scores))
    start=[node for node in floor_links if node[0]=="frame" and main[node[1]]]
    parents=dict.fromkeys(start)
    queue=deque(start)
    path=[]
    while queue:
        node=queue.popleft()
        if node[0]=="frame" and upper[node[1]]:
            while node is not None:
                path.append(node)
                node=parents[node]
            break
        for nxt in sorted(floor_links[node]):
            if nxt not in parents:
                parents[nxt]=node
                queue.append(nxt)
    summary={kind:dict(held_points=len(vals[0]),rms_m=[float(np.sqrt(np.mean(np.square(v)))) for v in vals],
                      p95_abs_m=[float(np.quantile(np.abs(v),.95)) for v in vals]) for kind,vals in values.items()}
    save_json(out,dict(roi_map_xy=[3,28,72,95],support_error=summary,training_floor_chain=path[::-1],
                       direct=[r for r in records if r["main_ids"] and r["upper_ids"]],planes=records))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("base","result","out"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    main(a.base,a.result,a.out)
