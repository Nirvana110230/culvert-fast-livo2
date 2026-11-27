"""Inspect local reference surfaces and normal-consistent support of a frozen revisit."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

from frame_io import frames_and_gravity, rigid_inverse, transform, voxel, save_json
from heldout import find_planes,normal_at
from loop_edges import EdgeBuilder
from verify import load_tum


def main():
    p=argparse.ArgumentParser()
    for k in ("index","gravity","result","out"):
        p.add_argument("--"+k,type=Path,required=True)
    p.add_argument("--region",type=int,required=True,help="1-based frozen region")
    a=p.parse_args()
    a.out.mkdir(exist_ok=False,parents=True)
    frames,G,_=frames_and_gravity(a.index,a.gravity)
    before=np.array([G@f.odom for f in frames])
    _,after=load_tum(a.result/"trajectory_after.tum")
    i,j=json.loads((a.result/"plan.json").read_text())["holdout_pairs_indices"][a.region-1]
    origin=before[i,:3,3]
    builder=EdgeBuilder(frames,before,[],set())
    old=transform(builder.scan(i),before[i])-origin
    old=voxel(old[np.sum(old[:,:2]**2,axis=1)<16],.14)
    planes=find_planes(old)
    report={"indices":[i,j],"region":a.region,"surfaces":{}}
    samples={"target":old}
    for stage,poses in (("before",before),("after",after)):
        parts=[]
        for k in range(j-1,j+2):
            pts=builder.scan(k)
            raw=transform(pts,before[k])
            pts=pts[np.sum((raw[:,:2]-before[j,:2,3])**2,axis=1)<16]
            parts.append(transform(pts,before[i]@rigid_inverse(poses[i])@poses[k])-origin)
        src=voxel(np.vstack(parts),.14)
        samples[stage]=src
        normal,planar=normal_at(src)
        for label,cent,n,tangent,anchor in planes:
            if not label.startswith("wall"):
                continue
            mask=planar&(np.abs(normal@n)>np.cos(np.deg2rad(15)))&(np.abs((src-cent)@n)<.85)
            s=src[mask]
            row={"target_points":len(anchor),"normal_filtered":len(s)}
            report["surfaces"].setdefault(label,{"target_center":cent.tolist(),"target_normal":n.tolist()})[stage]=row
            if not len(s):
                continue
            old2d=(anchor-cent)@tangent
            d=cKDTree(old2d).query((s-cent)@tangent)[0]
            s=s[d<.28]
            row["same_footprint_points"]=len(s)
            if len(s)<12:
                continue
            dots=(s-cent)@n
            bins,cnt=np.unique(np.floor(dots/.1),return_counts=True)
            mode=(bins[np.argmax(cnt)]+.5)*.1
            s=s[np.abs(dots-mode)<.2]
            coverage=float(np.mean(cKDTree((s-cent)@tangent).query(old2d)[0]<.28))
            row.update(peak_points=len(s),footprint_coverage=coverage)
            if len(s)>=12:
                _,_,basis=np.linalg.svd(s-s.mean(0),full_matrices=False)
                row["normal"]=basis[-1].tolist()
                row["angle_deg"]=float(np.rad2deg(np.arccos(np.clip(abs(basis[-1]@n),0,1))))
            samples[stage+"_"+label]=s
    save_json(a.out/"corner_audit.json",report)
    np.savez_compressed(a.out/"corner_points.npz",**samples)


if __name__=="__main__":
    main()
