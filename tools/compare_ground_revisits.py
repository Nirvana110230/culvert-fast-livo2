"""Apply alternative measured trajectories to the same local B1 floor patches."""
import argparse
import csv
from decimal import Decimal
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import ConvexHull
from scipy.stats import spearmanr
from ground_support import field, quantiles, records


def samples(directory):
    tum=directory/"trajectory_dense_after.tum"
    if not tum.is_file():
        tum=directory/"trajectory_after.tum"
    if tum.is_file():
        rows = [line.split() for line in tum.read_text().splitlines()
                if line and not line.startswith("#")]
        t = [int(Decimal(r[0])*1_000_000_000) for r in rows]
        p = np.array([[float(v) for v in r[1:4]] for r in rows])
        q = Rotation.from_quat([[float(v) for v in r[4:8]] for r in rows])
    else:
        rows = [r for r in records(directory/"keyframes/states.csv") if r["stage"] == "LIO"]
        t = [int(r["stamp_ns"]) for r in rows]
        G = Rotation.from_matrix(json.loads((directory/"gravity_transform.json").read_text())["transform"]["R_GC"])
        p = G.apply(np.array([field(r, "xyz") for r in rows]))
        q = G * Rotation.from_quat([field(r,["qx","qy","qz","qw"]) for r in rows])
    if len(t)<2 or np.any(np.diff(t)<=0):
        raise ValueError(f"Invalid trajectory in {directory}")
    return np.array(t), p, q


def paired_surface_difference(a,b):
    # Intersect two supported polygons after transforming their real 3-D boundaries.
    shape=a[:,:2]
    for eq in ConvexHull(b[:,:2]).equations:
        signed=shape @ eq[:2]+eq[2]
        before=np.roll(signed,1)
        if np.all(signed <= 0):
            continue
        if np.all(signed > 0):
            return None
        clipped=[]
        for k,value in enumerate(signed):
            if (value <= 0)!=(before[k] <= 0):
                fraction=before[k]/(before[k]-value)
                clipped.append(shape[k-1]+fraction*(shape[k]-shape[k-1]))
            if value <= 0:
                clipped.append(shape[k])
        shape=np.array(clipped)
        if len(shape)<3:
            return None
    area=abs(shape[:,0] @ np.roll(shape[:,1],-1) - shape[:,1] @ np.roll(shape[:,0],-1))/2
    if area<0.02:
        return None
    xy=shape.mean(axis=0)
    def elevation(vertices):
        center=vertices.mean(axis=0)
        normal=np.linalg.svd(vertices-center,full_matrices=False)[2][-1]
        return center[2]-normal[:2]@(xy-center[:2])/normal[2]
    return elevation(a)-elevation(b)


def compare(base, frozen, cases, out):
    index = records(base/"keyframes/index.csv")
    stamps = np.array([int(r["stamp_ns"]) for r in index])
    pairs = records(frozen)
    ids = np.array([(int(r["i"]),int(r["j"])) for r in pairs],dtype=int).reshape(-1,2)
    points = np.array([[field(r,[f"{s}_{a}_imu" for a in "xyz"]) for s in ("i","j")] for r in pairs]).reshape(-1,2,3)
    with np.load(frozen.parent/"ground_support_polygons.npz") as archive:
        vertices,offsets=archive["vertices"],archive["offsets"]
    ref_signed = np.array([float(r["floor_difference_m"]) for r in pairs])
    ref_abs = np.abs(ref_signed)
    travel = np.array([float(r["travel_m"]) for r in pairs])
    angles = np.array([float(r["normal_delta_deg"]) for r in pairs])
    def association(data):
        if len(data)<2 or np.ptp(data)==0 or np.ptp(ref_abs)==0:
            return None
        return float(spearmanr(ref_abs,data).statistic)
    summary = dict(frozen_count=len(pairs), source_ids="B1 keyframe index row offsets",
                   spearman_baseline_abs_dz_vs_travel=association(travel),
                   spearman_baseline_abs_dz_vs_normal_angle=association(angles),
                   scope="Fixed local samples on odometry-proposed overlaps, no survey or physical identity labels. Frontend comparison reuses B1 deskewed samples.",
                   cases={})
    all_rows, supported, components = [], {}, {}
    for name,directory in cases.items():
        ts,pos,rotation = samples(directory)
        dt = (ts-ts[0])*1e-9
        lookup = (stamps-ts[0])*1e-9
        idx = np.searchsorted(ts,stamps)
        bracket = np.minimum(idx,len(ts)-1)
        previous = np.maximum(0,bracket-1)
        exact = ts[bracket] == stamps
        valid = ((stamps>=ts[0]) & (stamps<=ts[-1]) &
                 (exact | (ts[bracket]-ts[previous]<=250_000_000)))
        query=np.clip(lookup,dt[0],dt[-1])
        rp=Slerp(dt,rotation)(query).as_matrix()
        px=np.column_stack([np.interp(query,dt,pos[:,k]) for k in range(3)])
        coordinates=np.einsum("...ij,...j->...i",rp[ids],points)+px[ids]
        good=valid[ids].all(axis=1)
        delta=coordinates[:,0]-coordinates[:,1]
        relative_translation_z=px[ids[:,0],2]-px[ids[:,1],2]
        components[name]=(relative_translation_z,delta[:,2]-relative_translation_z)
        dz=delta[good,2]
        xy=np.linalg.norm(delta[good,:2],axis=1)
        errors=np.full(len(pairs),np.nan)
        polygons={}
        for k in np.flatnonzero(good):
            i,j=ids[k]
            for node in (i,j):
                if node not in polygons:
                    poly=vertices[offsets[node]:offsets[node+1]]
                    polygons[node]=poly @ rp[node].T + px[node]
            dz_overlap=paired_surface_difference(polygons[i],polygons[j])
            if dz_overlap is not None:
                errors[k]=dz_overlap
            all_rows.append(dict(case=name,i=int(ids[k,0]),j=int(ids[k,1]),
                                 local_plane_height_difference_m=delta[k,2],
                                 fixed_sample_xy_distance_m=np.linalg.norm(delta[k,:2]),
                                 overlap_height_difference_m=dz_overlap))
        supported[name]=errors
        summary["cases"][name]=dict(covered=int(good.sum()),abs_height_difference_m=quantiles(np.abs(dz)),
                                   signed_height_difference_m=quantiles(dz),xy_sample_distance_m=quantiles(xy),
                                   above_15cm=int(np.sum(abs(dz)>.15)),above_25cm=int(np.sum(abs(dz)>.25)),
                                   abs_height_over_50m_travel_m=quantiles(abs(delta[good & (travel>=50),2])),
                                   median_abs_change_from_baseline_m=float(np.median(abs(dz)-ref_abs[good])) if len(dz) else None,
                                   max_abs_signed_disagreement_with_frozen_baseline_m=float(np.max(abs(dz-ref_signed[good]))) if len(dz) else None)
        valid_surface=errors[np.isfinite(errors)]
        summary["cases"][name]["supported_overlap_abs_height_m"]=quantiles(np.abs(valid_surface))
    common=np.logical_and.reduce([np.isfinite(errors) for errors in supported.values()])
    summary["common_supported_count"]=int(common.sum())
    for name,errors in supported.items():
        summary["cases"][name]["common_overlap_abs_height_m"]=quantiles(np.abs(errors[common]))
        summary["cases"][name]["common_overlap_above_15cm"]=int(np.sum(np.abs(errors[common])>.15))
        if "b1" in supported:
            shared=np.isfinite(errors)&np.isfinite(supported["b1"])
            summary["cases"][name]["paired_b1_abs_height_m"]=quantiles(np.abs(supported["b1"][shared]))
            summary["cases"][name]["paired_case_abs_height_m"]=quantiles(np.abs(errors[shared]))
            for k,label in enumerate(("pose_translation","local_point_attitude")):
                diff=components[name][k]-components["b1"][k]
                summary["cases"][name][f"paired_abs_change_at_fixed_sample_from_{label}_m"]=quantiles(abs(diff[shared]))
    out.mkdir(parents=True,exist_ok=True)
    with (out/"ground_revisit_cases.csv").open("w",newline="",encoding="utf8") as f:
        w=csv.DictWriter(f,fieldnames=["case","i","j","local_plane_height_difference_m",
                                       "fixed_sample_xy_distance_m","overlap_height_difference_m"])
        w.writeheader()
        w.writerows(all_rows)
    (out/"ground_revisit_comparison.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n",encoding="utf8")
    return summary


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base",type=Path)
    parser.add_argument("frozen_pairs",type=Path)
    parser.add_argument("out",type=Path)
    parser.add_argument("--case",action="append",required=True,metavar="NAME=DIRECTORY")
    args=parser.parse_args()
    cases={name:Path(p) for name,p in (c.split("=",1) for c in args.case)}
    compare(args.base,args.frozen_pairs,cases,args.out)
