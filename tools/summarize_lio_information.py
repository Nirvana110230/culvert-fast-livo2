"""Validate and summarize sampled pre-prior LIO normal matrices and their measurement weights."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def dist(x):
    x = np.asarray(x, dtype=float)
    if not len(x):
        return {"n": 0}
    return dict(zip(("min","p05","p50","p95","max"),map(float,np.quantile(x,[0,.05,.5,.95,1]))))


def summarize(folder):
    matrices=folder/"lio_information_6d.csv"
    weights=folder/"measurement_weight_sources.csv"
    with matrices.open(newline="") as f:
        samples=list(csv.DictReader(f))
    with weights.open(newline="") as f:
        source=list(csv.DictReader(f))
    valid=[r for r in samples if r["valid"]=="1"]
    if not valid: raise ValueError("no valid sampled normal matrices")
    if len(valid)!=len({r["cut_s"] for r in valid}): raise ValueError("duplicate information sample time")
    mat=np.zeros((len(valid),6,6))
    for i in range(6):
        for j in range(i,6):
            mat[:,i,j]=mat[:,j,i]=[float(r[f"h{i}{j}"]) for r in valid]
    data=lambda key: np.array([float(r[key]) for r in valid])
    eig=np.column_stack([data(f"lambda{i}") for i in range(6)])
    weak=np.column_stack([data("weak_"+k) for k in ("rx","ry","rz","tx","ty","tz")])
    up=np.column_stack([data("up_"+k) for k in "xyz"])
    recomputed=np.linalg.eigvalsh(mat)
    native_scale=np.maximum(eig[:,-1],1)
    residual=np.einsum("nij,nj->ni",mat,weak)-eig[:,0,None]*weak
    all_rows={r["cut_s"]:r for r in source if r["class"]=="all"}
    if set(all_rows)!={r["cut_s"] for r in samples}: raise ValueError("weight sample timestamps differ")
    total_v=sum(float(r["vertical_translation_information"]) for r in all_rows.values())
    total_w=sum(float(r["weight_sum"]) for r in all_rows.values())
    arm_by_cut={r["cut_s"]:float(r["rms_arm_m"]) for r in valid}
    def scaled_yaw(records):
        return sum(float(r["yaw_rotation_information"])/arm_by_cut[r["cut_s"]]**2
                   for r in records if r["cut_s"] in arm_by_cut)
    total_yaw=scaled_yaw(all_rows.values())
    classes={}
    for category in sorted({r["class"] for r in source}):
        sub=[r for r in source if r["class"]==category]
        n=sum(int(r["count"]) for r in sub)
        sums={k:sum(float(r[k]) for r in sub) for k in
              ("weight_sum","sigma_plane_sum_m2","sigma_point_sum_m2","inverse_weight_sum_m2",
               "fixed_variance_sum_m2","vertical_translation_information","yaw_rotation_information")}
        classes[category]={"count":n,"weight_fraction":sums["weight_sum"]/total_w,
            "vertical_translation_information_fraction":sums["vertical_translation_information"]/total_v if total_v else None,
            "gravity_axis_scaled_rotation_information_fraction":scaled_yaw(sub)/total_yaw if total_yaw else None,
            "arithmetic_mean_R_inv_per_m2":sums["weight_sum"]/n if n else None,
            "mean_fixed_variance_m2":sums["fixed_variance_sum_m2"]/n if n else None,
            "mean_plane_variance_m2":sums["sigma_plane_sum_m2"]/n if n else None,
            "mean_projected_point_variance_m2":sums["sigma_point_sum_m2"]/n if n else None,
            "component_fraction_of_summed_variance":{k:sums[k]/sums["inverse_weight_sum_m2"] if n else None
                for k in ("fixed_variance_sum_m2","sigma_plane_sum_m2","sigma_point_sum_m2")}}
    def row_source(k):
        return np.array([float(all_rows[r["cut_s"]][k]) for r in valid])
    vdir=np.einsum("ni,nij,nj->n",up,mat[:,3:,3:],up)
    ydir=np.einsum("ni,nij,nj->n",up,mat[:,:3,:3],up)
    scale=data("rms_arm_m"); weight=data("weight_sum")
    relative=lambda a,b: float(np.max(np.abs(a-b)/np.maximum(np.maximum(np.abs(a),np.abs(b)),1)))
    checks={"recomputed_eigenvalue_relative_to_largest_max_error":float(np.max(np.abs(recomputed-eig)/native_scale[:,None])),
            "weak_eigenvector_relative_residual_max":float(np.max(np.linalg.norm(residual,axis=1)/native_scale)),
            "weak_unit_norm_max_error":float(np.max(np.abs(np.linalg.norm(weak,axis=1)-1))),
            "vertical_projection_max_relative_error":relative(vdir,row_source("vertical_translation_information")),
            "yaw_projection_max_relative_error":relative(ydir,row_source("yaw_rotation_information")/scale**2),
            "inverse_weight_decomposition_max_relative_error":max(abs(float(r["inverse_weight_sum_m2"])-float(r["fixed_variance_sum_m2"])
                -float(r["sigma_plane_sum_m2"])-float(r["sigma_point_sum_m2"]))/max(1,abs(float(r["inverse_weight_sum_m2"]))) for r in source)}
    if max(checks.values())>1e-7: raise ValueError(f"independent matrix / covariance verification failed: {checks}")
    if sum(c["count"] for name,c in classes.items() if name!="all")!=classes["all"]["count"]:
        raise ValueError("normal categories do not conserve matches")
    tnorm=np.linalg.norm(weak[:,3:],axis=1)
    if np.any(eig[:,-1]<=0): raise ValueError("valid information sample has no positive eigenvalue")
    ratio=eig[:,0]/eig[:,-1]
    vert=np.einsum("ni,ni->n",up,weak[:,3:])
    worst=[{"cut_s":float(valid[i]["cut_s"]),"lambda_min_max_ratio":float(ratio[i]),
            "weak_scaled_rotation_world_xyz":weak[i,:3].tolist(),"weak_translation_world_xyz":weak[i,3:].tolist(),
            "weak_translation_dot_up":float(vert[i]),"second_to_first_eigenvalue_ratio":float(eig[i,1]/eig[i,0]) if eig[i,0]>0 else None}
           for i in np.argsort(ratio)[:5]]
    summary={"matrix_sha256":hashlib.sha256(matrices.read_bytes()).hexdigest(),"weight_sha256":hashlib.sha256(weights.read_bytes()).hexdigest(),
        "samples":len(samples),"valid_samples":len(valid),"sample_dt_s":dist(np.diff(data("cut_s"))),
        "definition":"Final accepted point-to-plane matches, before state prior. World coordinates [RMS_arm*rotation,translation]. Per-update unweighted RMS of matched IMU-frame lever arms. Eigenvalues ascending; eigendirection sign arbitrary. No trajectory-error ground truth.",
        "classes":classes,"rms_arm_m":dist(scale),"lambda_min_over_max":dist(ratio),
        "weak_rotation_norm":dist(np.linalg.norm(weak[:,:3],axis=1)),"weak_translation_norm":dist(tnorm),
        "weak_abs_translation_projection_up":dist(np.abs(vert)),
        "weak_abs_scaled_rotation_projection_up":dist(np.abs(np.einsum("ni,ni->n",up,weak[:,:3]))),
        "weak_abs_translation_cosine_up_for_norm_gt_0_1":dist(np.abs(vert[tnorm>.1])/tnorm[tnorm>.1]),
        "vertical_translation_rayleigh_per_sum_weight":dist(vdir/weight),
        "yaw_scaled_rotation_rayleigh_per_sum_weight":dist(ydir/weight),
        "five_smallest_min_over_max":worst,"independent_recomputation":checks}
    for filename,key in (("algorithm_identity.json","runtime_identity"),("offline.json","run_configuration")):
        for file in (folder/filename,folder.parent/filename):
            if file.is_file():
                summary[key]=json.loads(file.read_text(encoding="utf-8"))
                break
    return summary


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("directory",type=Path); p.add_argument("--out",type=Path)
    args=p.parse_args()
    result=json.dumps(summarize(args.directory),indent=2,allow_nan=False)
    if args.out: args.out.write_text(result+"\n",encoding="utf-8")
    else: print(result)
