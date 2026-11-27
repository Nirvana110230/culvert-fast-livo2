"""Offline keyframe graph optimization with reproducible constraints and held-out measurements."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time

import gtsam
import numpy as np

from frame_io import dense_trajectory, frames_and_gravity, rebuild, rigid_inverse, save_json, write_tum
from heldout import evaluate
from loop_edges import EdgeBuilder, p0_pair, rotation_deg, screen, select_pairs
from noise import calibrate, loop_covariance, motion_scale
from preview import preview


def optimize(poses, times, model, loops, loop_scale=1.):
    graph=gtsam.NonlinearFactorGraph()
    values=gtsam.Values()
    factors=[]
    for k, matrix in enumerate(poses):
        p=gtsam.Pose3(matrix)
        values.insert(k,p)
        if k==0:
            noise=gtsam.noiseModel.Isotropic.Sigma(6,1e-5)
            graph.add(gtsam.PriorFactorPose3(k,p,noise))
            continue
        delta=rigid_inverse(poses[k-1])@matrix
        sigmas=np.array(model["base_sigmas"])*motion_scale(delta,times[k]-times[k-1])
        noise=gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        graph.add(gtsam.BetweenFactorPose3(k-1,k,gtsam.Pose3(delta),noise))
        factors.append({"kind":"odom","target_index":k-1,"source_index":k,"T_target_source":delta.tolist(),
                        "sigmas_rot_rad_trans_m":sigmas.tolist()})
    for e in loops:
        cov=loop_covariance(e,model)*loop_scale**2
        noise=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.8),
                                              gtsam.noiseModel.Gaussian.Covariance(cov))
        graph.add(gtsam.BetweenFactorPose3(e["target_index"],e["source_index"],gtsam.Pose3(e["T_target_source"]),noise))
        factors.append({"kind":e["kind"],"target_index":e["target_index"],"source_index":e["source_index"],
                        "T_target_source":e["T_target_source"],"covariance_source_tangent":cov.tolist(),
                        "sigmas_rot_rad_trans_m":np.sqrt(np.diag(cov)).tolist(),"huber_k":1.8})
    params=gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(80)
    opt=gtsam.LevenbergMarquardtOptimizer(graph,values,params)
    result=opt.optimize()
    after=np.array([result.atPose3(k).matrix() for k in range(len(poses))])
    return after, factors, {"solver":"GTSAM Pose3 Levenberg-Marquardt","initial_error":float(graph.error(values)),
        "final_error":float(graph.error(result)), "iterations":int(opt.iterations()),"factor_count":graph.size(),
        "gauge":"first pose prior sigma 1e-5", "loop_loss":"Huber 1.8 on whitened 6D residual block"}


def describe_track(T):
    d=np.diff(T[:,:3,3],axis=0)
    lengths=np.linalg.norm(d,axis=1)
    delta=T[-1,:3,3]-T[0,:3,3]
    return {"length_m":float(sum(lengths)),"first_last_xyz_m":delta.tolist(),
            "first_last_norm_m":float(np.linalg.norm(delta)), "largest_step_m":float(max(lengths)),
            "step_p95_m":float(np.quantile(lengths,.95)),"z_range_m":float(np.ptp(T[:,2,3]))}


def whitened(e,r):
    v=gtsam.Pose3.Logmap(gtsam.Pose3(r))
    return float(np.linalg.norm(np.linalg.solve(np.linalg.cholesky(e["covariance_source_tangent"]),v)))


def solve_screened(poses,times,model,loops):
    active=list(loops)
    passes=[]
    while True:
        after,factors,graph=optimize(poses,times,model,active)
        discard=[]
        for e in active:
            i,j=e["target_index"],e["source_index"]
            residue=rigid_inverse(np.array(e["T_target_source"]))@rigid_inverse(after[i])@after[j]
            norm=whitened(e,residue)
            e.setdefault("graph_screen_whitened_norms",[]).append(norm)
            if norm>7.2:
                discard.append(e)
        passes.append({"input_loops":len(active),"graph_initial_error":graph["initial_error"],
                       "graph_final_error":graph["final_error"],"inconsistent":[[e["target_id"],e["source_id"]] for e in discard]})
        if not discard:
            graph["consistency_passes"]=passes
            return after,factors,graph,active
        for e in discard:
            e["accepted"]=False
            e["reasons"].append("postfit_graph_inconsistent")
        active=[e for e in active if e["accepted"]]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--index",type=Path,required=True)
    p.add_argument("--gravity",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True,help="new private output directory")
    p.add_argument("--p0-windows",type=float,nargs=4,metavar=("A0","A1","B0","B1"),
                   help="operator-identified tape windows in seconds relative to first keyframe (defaults to none)")
    p.add_argument("--time-origin-ns",type=int,help="first valid odometry stamp if different from first keyframe")
    p.add_argument("--frozen",type=Path,help="reuse a previous run's frozen sensor-time pairs and exclude their geometry")
    p.add_argument("--max-queries",type=int,default=45)
    p.add_argument("--candidate-radius",type=float,default=5)
    p.add_argument("--min-seconds",type=float,default=75)
    p.add_argument("--mode",choices=("tape","automatic","combined"),default="combined")
    args=p.parse_args()
    start=time.monotonic()
    out=args.out
    out.mkdir(parents=True,exist_ok=False)
    frames,G,gravity=frames_and_gravity(args.index,args.gravity)
    before=np.array([G@f.odom for f in frames])
    origin=args.time_origin_ns if args.time_origin_ns is not None else frames[0].stamp
    times=np.array([(f.stamp-origin)*1e-9 for f in frames])
    tape,windows=p0_pair(times,before,args.p0_windows)
    fixed_pairs=None
    if args.frozen:
        reference=json.loads(args.frozen.read_text())
        stamps=np.array([f.stamp for f in frames],dtype=np.int64)
        fixed_pairs=[]
        for pair in reference["pairs"]:
            candidate=[]
            for side in ("target","source"):
                difference=np.abs(stamps-pair[side+"_stamp_ns"])
                k=int(np.argmin(difference))
                if difference[k]*1e-9>reference["maximum_time_difference_s"]:
                    raise ValueError(f"frozen {side} time absent in region {pair['region']}")
                candidate.append(k)
            fixed_pairs.append(tuple(candidate))
    hold,centers,excluded,auto,total=select_pairs(before,times,tape,args.max_queries,args.candidate_radius,args.min_seconds,fixed_pairs)
    plan={"schema":1,"parameters":vars(args) | {"index":str(args.index),"out":str(out),"gravity":str(args.gravity),
                                                "frozen":str(args.frozen) if args.frozen else None},
          "time_origin_ns":origin,"gravity_transform":gravity,
          "index_sha256":hashlib.sha256(args.index.read_bytes()).hexdigest(),"tape_windows":windows,
          "holdout_pairs_indices":hold,"holdout_centers_xy_m":centers.tolist(),"holdout_radius_m":4.,
          "heldout_frame_indices":sorted(excluded),"raw_proposals":total,"selected_automatic_pairs":auto}
    save_json(out/"plan.json",plan)
    builder=EdgeBuilder(frames,before,centers,excluded)
    model=calibrate(builder,before,times)
    save_json(out/"noise_model.json",model)
    pairs=( [(tape,"tape")] if tape and args.mode!="automatic" else [])
    if args.mode!="tape":
        pairs.extend((ij,"automatic") for ij in auto)
    edges=[]
    with (out/"registration_progress.jsonl").open("x") as log:
        for (i,j),kind in pairs:
            t=time.monotonic()
            row=builder.estimate(i,j,kind)
            row["elapsed_s"]=time.monotonic()-t
            edges.append(row)
            log.write(json.dumps(row,allow_nan=False)+"\n")
            log.flush()
    loops=screen(edges,before,times)
    registration_s=time.monotonic()-start
    after,factors,graph,loops=solve_screened(before,times,model,loops)
    for e in edges:
        if "T_target_source" in e:
            i,j=e["target_index"],e["source_index"]
            for name,poses in (("before",before),("after",after)):
                residual=rigid_inverse(np.array(e["T_target_source"]))@rigid_inverse(poses[i])@poses[j]
                e[name+"_factor_residual_m"]=float(np.linalg.norm(residual[:3,3]))
                e[name+"_factor_residual_deg"]=rotation_deg(residual)
            if e["accepted"]:
                norm=whitened(e,residual)
                e["final_whitened_norm"]=norm
                e["final_huber_weight"]=min(1.,1.8/max(1e-12,norm))
    save_json(out/"edges.json",edges)
    save_json(out/"factors.json",factors)
    write_tum(out/"trajectory_before.tum",frames,before)
    write_tum(out/"trajectory_after.tum",frames,after)
    write_tum(out/"map_to_odom.tum",frames,[T@rigid_inverse(f.odom) for f,T in zip(frames,after)])
    dense=dense_trajectory(args.index.parent/"states.csv",out,frames,G,after,args.p0_windows,origin)
    correction={"parent":"map_gravity","child":"camera_init","T_definition":"T_map_body_opt @ inverse(T_odom_body_raw)",
                "interpolation":"keyframe history only","quaternion_order":"xyzw","timestamp_unit":"seconds",
                "last_T_map_odom":(after[-1]@rigid_inverse(frames[-1].odom)).tolist()}
    save_json(out/"map_to_odom.json",correction)
    with (out/"poses.csv").open("x",newline="") as f:
        w=csv.writer(f); w.writerow(["id","stamp_ns","t_s",*[f"{n}_{a}_m" for n in ("before","after") for a in "xyz"]])
        for k,frame in enumerate(frames):
            w.writerow([frame.id,frame.stamp,times[k],*before[k,:3,3],*after[k,:3,3]])
    validation=evaluate(hold,builder,before,after)
    save_json(out/"holdout.json",validation)
    from surface_quality import RULES
    save_json(out/"holdout_validated.json",{"same_poses_and_loops":True,"rules":RULES,"regions":validation})
    sensitivity=[]
    for scale in (.5,1.,2.):
        alternate=after if scale==1. else optimize(before,times,model,loops,scale)[0]
        drift=np.linalg.norm(alternate[:,:3,3]-after[:,:3,3],axis=1)
        comparison={"loop_sigma_multiplier":scale,"fixed_loop_count":len(loops),"track":describe_track(alternate),
                    "median_deviation_from_nominal_m":float(np.median(drift)),"max_deviation_from_nominal_m":float(max(drift))}
        if tape:
            i,j=tape
            comparison["tape_pair_separation_m"]=float(np.linalg.norm(alternate[j,:3,3]-alternate[i,:3,3]))
        checks=validation if scale==1. else evaluate(hold,builder,before,alternate)
        comparison["heldout_source_medians_m"]=[c["point_to_point"].get("after",{}).get("source_median_m") for c in checks]
        sensitivity.append(comparison)
    save_json(out/"weight_sensitivity.json",sensitivity)
    t=time.monotonic()
    maps=rebuild(out,frames,before,after)
    map_s=time.monotonic()-t
    bounds=preview(out,before,after)
    summary={"keyframes":len(frames),"duration_s":float(times[-1]-times[0]),"tape_windows":windows,
             "accepted":[{"kind":e["kind"],"target_id":e["target_id"],"source_id":e["source_id"]} for e in loops],
             "considered":len(edges),"heldout_revisits":len(hold),"holdout_points_removed_from_fits_with_repetition":builder.masked_count,
             "before":describe_track(before),"after":describe_track(after),"dense_lio":dense,"graph":graph,"maps":maps,
             "bounds":bounds,"registration_phase_s":registration_s,"rebuild_s":map_s,
             "wall_clock_s":time.monotonic()-start,"peak_rss_MiB":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
             "bytes_prior_to_summary":sum(f.stat().st_size for f in out.iterdir() if f.is_file())}
    if tape:
        i,j=tape
        for name,poses in [("before",before),("after",after)]:
            d=poses[j,:3,3]-poses[i,:3,3]
            summary[name].update(tape_window_delta_xyz_m=d.tolist(),tape_window_separation_m=float(np.linalg.norm(d)))
    if not np.isfinite(after).all():
        raise RuntimeError("nonfinite graph solution")
    save_json(out/"summary.json",summary)
    print(json.dumps(summary,ensure_ascii=False))


if __name__ == "__main__":
    main()
