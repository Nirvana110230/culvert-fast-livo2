"""Follow identical native point records through two historical map rebuilds."""
import argparse
import json
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from evaluate_z_geometry import binary_map, read_spatial_sample, horizontal, plane_residual


def main(args):
    old,_=binary_map(args.old_native)
    new,_=binary_map(args.new_native)
    if old.shape!=new.shape or not np.array_equal(old[:,3],new[:,3]):
        raise ValueError("native geometry record ordering differs")
    sample,_=read_spatial_sample(args.old_5cm)
    trajectory=np.loadtxt(args.old_trajectory)[:,1:4]
    tree=cKDTree(old[:,:3])
    def pair(q):
        d,ids=tree.query(q,k=2,workers=4)
        if d[:,0].max()>1e-6:
            raise ValueError("5 cm reference is not a subset of the previous native geometry")
        ambiguous=(d[:,1]<=1e-7)&(np.linalg.norm(new[ids[:,1],:3]-new[ids[:,0],:3],axis=1)>.0001)
        return q[~ambiguous],new[ids[~ambiguous,0],:3],int(ambiguous.sum())
    with zipfile.ZipFile(args.reference_zip) as archive:
        fixed=pd.read_csv(archive.open("analysis/geometry/matched_floor_grid.csv"))[["i","j"]]
        segments=json.load(archive.open("analysis/geometry/long_segments_matched_stats.json"))["definitions"]
        reference=pd.read_csv(archive.open("analysis/geometry/long_segments_profiles.csv"))
    floor,_=horizontal(sample,trajectory,20,2.5)
    a,b,amb=pair(floor)
    df=pd.DataFrame(np.column_stack((a,b)),columns=["old_x","old_y","old_z","new_x","new_y","new_z"])
    df["i"]=np.floor(df.old_x).astype(int)
    df["j"]=np.floor(df.old_y).astype(int)
    cells=df.groupby(["i","j"])
    med=cells.median()[cells.size()>=4].reset_index().merge(fixed,on=["i","j"])
    res=[plane_residual(med[k+"_x"],med[k+"_y"],med[k+"_z"]) for k in ("old","new")]
    result=dict(fixed_cells=len(fixed),supported_cells=len(med),native_floor_samples=len(a),ambiguous_floor_excluded=amb,
                floor_rms_m=[np.sqrt(np.mean(e**2)) for e in res],floor_correlation=np.corrcoef(res)[0,1],profiles={})
    samples=horizontal(sample,trajectory,8,3.)
    for typ,points in zip(("floor","ceiling"),samples):
        a,b,amb=pair(points)
        for seg,definition in segments.items():
            start=np.array(definition["start_xy"])
            delta=np.array(definition["end_xy"])-start
            u=delta/np.linalg.norm(delta)
            along=(a[:,:2]-start)@u
            cross=(a[:,:2]-start)@np.array([-u[1],u[0]])
            keep=(along>0)&(along<np.linalg.norm(delta))&(abs(cross)<1.8)
            df=pd.DataFrame({"s":along[keep],"old":a[keep,2],"new":b[keep,2]})
            bins=df.groupby(np.floor(df.s).astype(int).rename("sbin"))
            med=bins.median()[bins.size()>=8].reset_index()
            ref=reference[(reference.surface==typ)&(reference.segment==seg)]
            common=set(ref[ref.series=="before"].sbin)&set(ref[ref.series=="after"].sbin)
            med=med[med.sbin.isin(common)]
            row=dict(fixed_bins=len(common),covered=len(med),samples=int(keep.sum()))
            for side in ("old","new"):
                coef=np.polyfit(med.s,med[side],1)
                row[side]=dict(slope_deg=float(np.degrees(np.arctan(coef[0]))),
                               rms_m=float(np.sqrt(np.mean((med[side]-np.polyval(coef,med.s))**2))))
            result["profiles"].setdefault(seg,{})[typ]=row
    args.out.write_text(json.dumps(result,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("old_native","new_native","old_5cm","old_trajectory","reference_zip","out"):
        p.add_argument(name,type=Path)
    main(p.parse_args())
