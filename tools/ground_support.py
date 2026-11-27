"""Measure local floor support and revisit consistency on completed estimator keyframes."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull
from scipy.stats import spearmanr

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))
from frame_io import frames_and_gravity, read_pcd


def records(path):
    with path.open(encoding="utf8", newline="") as stream:
        return list(csv.DictReader(stream))


def field(row, names):
    return np.array([float(row[k]) for k in names])


def quantiles(data):
    data = np.asarray(data)
    if not len(data):
        return {"n": 0}
    return {"n": len(data), **dict(zip(("min","p05","p50","p95","max"),
                                        np.percentile(data,[0,5,50,95,100]).tolist()))}


def normal_evidence(plane_rows,rows,pairs,up):
    strong=[r for r in plane_rows if r["class"]=="horizontal_below" and int(r["count"])>=20]
    tensors=np.zeros((len(strong),3,3))
    for name,(i,j) in zip(("xx","xy","xz","yy","yz","zz"),((0,0),(0,1),(0,2),(1,1),(1,2),(2,2))):
        tensors[:,i,j]=tensors[:,j,i]=[float(r["n"+name+"_sum"])/int(r["count"]) for r in strong]
    values,vectors=np.linalg.eigh(tensors)
    theta=np.degrees(np.arccos(np.clip(abs(vectors[:,:,2] @ up),0.,1.)))
    count=[]
    fraction=[]
    dz=[]
    for pair in pairs:
        endpoints=[rows[k] for k in (pair["i"],pair["j"])]
        count.append(min(r["voxel_floor_matches"] for r in endpoints))
        fraction.append(min(r["voxel_floor_matches"]/r["voxel_matches"] for r in endpoints))
        dz.append(abs(pair["floor_difference_m"]))
    def association(x):
        return float(spearmanr(dz,x).statistic) if len(x)>1 and np.ptp(x)>0 and np.ptp(dz)>0 else None
    return {"eligible_windows":len(strong),
            "principal_normal_tensor_fraction_p05_p50_p95":np.percentile(values[:,2],[5,50,95]).tolist() if strong else [],
            "principal_angle_to_gravity_deg_p05_p50_p95":np.percentile(theta,[5,50,95]).tolist() if strong else [],
            "revisit_abs_dz_vs_min_floor_count_spearman":association(count),
            "revisit_abs_dz_vs_min_floor_fraction_spearman":association(fraction),
            "meaning":"Normal alignment only. Per-plane identifiers, cross-scan covariance and noise independence were not measured."}


def fit_floor(points, rng):
    """Gravity-frame points relative to IMU. Other returns include tables and non-floor ledges."""
    xy = points[:, :2]
    eligible = points[(np.linalg.norm(xy, axis=1) <= 2.5) &
                      (points[:, 2] < -0.2) & (points[:, 2] > -2.5)]
    if len(eligible) < 50:
        return None
    # Use disjoint fit/validation returns. All returns inside the measured ROI enter one split.
    train, hold = eligible[::2], eligible[1::2]
    best_n, best = 0, None
    for _ in range(60):
        a,b,c = train[rng.choice(len(train), 3, replace=False)]
        normal = np.cross(b-a,c-a)
        length = np.linalg.norm(normal)
        if length < 1e-7:
            continue
        normal /= length
        if abs(normal[2]) < math.cos(math.radians(25)):
            continue
        offset = a @ normal
        matches = np.abs(train @ normal - offset) <= 0.035
        count = np.count_nonzero(matches)
        if count > best_n:
            best_n, best = count, matches
    if best_n < 16:
        return None
    selected = train[best]
    center = selected.mean(axis=0)
    _, _, vt = np.linalg.svd(selected-center, full_matrices=False)
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    offset = center @ normal
    residuals = np.abs(hold @ normal-offset)
    verified = residuals <= 0.05
    support = hold[verified]
    if len(support) < 15 or normal[2] < math.cos(math.radians(25)):
        return None
    # Reject narrow wall bases. The scanner sees much of the floor ahead of the cart.
    # Preserve the polygon of actual held-out returns for overlap tests at revisits.
    spread = np.linalg.eigvalsh(np.cov(support[:, :2].T))
    if spread[0] < 0.04:
        return None
    hull = ConvexHull(support[:,:2])
    height = -offset / normal[2]
    if not 0.2 <= height <= 2.5:
        return None
    return (normal, height, len(support), np.median(residuals[verified]), support[:,:2].mean(axis=0),
            hull.equations, support[hull.vertices,:2])


def analyze(directory, out):
    completed = json.loads((directory/"complete.json").read_text())
    frames, gravity, identity = frames_and_gravity(directory/"index.csv",directory.parent/"gravity_events.jsonl")
    states = [r for r in records(directory/"states.csv") if r["stage"] == "LIO"]
    if completed["frames"] != len(frames) or not states:
        raise ValueError("Incomplete keyframe session")
    basis = gravity[:3,:3]
    # Audit emits the same estimator double as the keyframe stamp, formatted with 17 digits.
    plane_rows = records(directory/"plane_constraints.csv")
    def counts(label):
        return {round(float(r["cut_s"])*1e9):int(r["count"]) for r in plane_rows if r["class"]==label}
    floor_counts, all_counts = counts("horizontal_below"), counts("all")
    weak_windows = sorted(t for t,n in floor_counts.items() if n < 20 and n < .02*all_counts[t])
    rng = np.random.default_rng(27112026)
    stamps, pos, normals, heights, centers, polygons, body_polygons = [], [], [], [], [], [], []
    rows = []
    for frame in frames:
        stamp = frame.stamp*1e-9
        p = basis @ frame.odom[:3,3]
        measured, _ = read_pcd(frame.cloud)
        result = fit_floor(measured @ (basis @ frame.odom[:3,:3]).T, rng)
        floor_count, total_count = floor_counts[frame.stamp], all_counts[frame.stamp]
        entry = dict(id=frame.id, stamp_s=stamp, gx=p[0], gy=p[1], gz=p[2], has_plane=int(result is not None),
                     voxel_floor_matches=floor_count,voxel_matches=total_count,
                     weak_support=int(floor_count < 20 and floor_count < .02*total_count))
        if result is not None:
            n,h,count,res,center,polygon,boundary = result
            boundary_z = -h - boundary @ n[:2]/n[2]
            body_polygons.append(np.column_stack([boundary,boundary_z]) @ (basis @ frame.odom[:3,:3]))
            entry.update(normal_x=n[0],normal_y=n[1],normal_z=n[2], height_m=h, holdout_support=count,
                         patch_x=p[0]+center[0],patch_y=p[1]+center[1],
                         holdout_median_abs_inlier_residual_m=res, slope_deg=np.degrees(np.arccos(np.clip(n[2],-1.,1.))))
        else:
            n,h = np.full(3,np.nan), np.nan
            center,polygon = np.full(2,np.nan),None
            body_polygons.append(np.empty((0,3)))
        centers.append(p[:2]+center)
        polygons.append(polygon)
        stamps.append(stamp); pos.append(p); normals.append(n); heights.append(h); rows.append(entry)
    stamps, pos, normals, heights, centers = map(np.asarray,(stamps,pos,normals,heights,centers))
    travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1))]
    def floor_z(k,xy):
        return pos[k,2]-heights[k] - normals[k,:2] @ (xy-pos[k,:2])/normals[k,2]
    def covers(k,xy):
        eq=polygons[k]
        return eq is not None and np.max(eq[:,:2] @ (xy-pos[k,:2])+eq[:,2]) <= 0.001
    steps = []
    for i in range(1,len(frames)):
        j = i-1
        xy = centers[i]-centers[j]
        span = np.linalg.norm(xy)
        if not (np.isfinite(heights[[i,j]]).all() and 0.25 <= span <= 1.5 and
                stamps[i]-stamps[j] <= 5.2 and normals[j] @ normals[i] > math.cos(math.radians(10))):
            continue
        normal_slope = -0.5 * (normals[i,:2]/normals[i,2]+normals[j,:2]/normals[j,2])
        estimated_step = floor_z(i,centers[i])-floor_z(j,centers[j])
        expected_step = normal_slope @ xy
        steps.append(dict(i=i,j=j,xy_m=span,measured_floor_step_m=estimated_step,
                          slope_predicted_step_m=expected_step,difference_m=estimated_step-expected_step))
    repeat = []
    for i in range(1,len(frames)):
        if not np.isfinite(heights[i]):
            continue
        xy_dist = np.linalg.norm(centers[:i]-centers[i],axis=1)
        valid = (np.isfinite(heights[:i]) & (xy_dist <= 0.6) & (stamps[i]-stamps[:i] >= 20) &
                 (travel[i]-travel[:i] >= 10) & (normals[:i] @ normals[i] > math.cos(math.radians(10))))
        ids = [j for j in np.flatnonzero(valid) if covers(i,(centers[i]+centers[j])/2) and covers(j,(centers[i]+centers[j])/2)]
        if not len(ids):
            continue
        j = ids[np.argmin(xy_dist[ids])]
        xy_mid = (centers[i]+centers[j])/2
        revisit = dict(i=i,j=int(j),xy_m=xy_dist[j],travel_m=travel[i]-travel[j],
                       dt_s=stamps[i]-stamps[j],floor_difference_m=floor_z(i,xy_mid)-floor_z(j,xy_mid),
                       normal_delta_deg=np.degrees(np.arccos(np.clip(normals[i] @ normals[j],-1.,1.))))
        # Freeze both interpolated points INSIDE the measured supports in their local IMU coordinates.
        # Other trajectories can move these same samples without selecting more convenient matches.
        for label,k in (("i",i),("j",j)):
            p=frames[k].odom
            local=(basis @ p[:3,:3]).T @ (np.r_[xy_mid,floor_z(k,xy_mid)] - pos[k])
            for axis,value in zip("xyz",local):
                revisit[f"{label}_{axis}_imu"]=value
        repeat.append(revisit)
    weak_at_keyframe=[i for i,r in enumerate(rows) if r["weak_support"]]
    weak_overlaps=[]
    for i in weak_at_keyframe:
        if not rows[i]["has_plane"]:
            continue
        near = (stamps[i]-stamps[:i] <= 2.) & (np.linalg.norm(centers[i]-centers[:i],axis=1) <= 1.0)
        for j in np.flatnonzero(near & np.isfinite(heights[:i]))[::-1]:
            midpoint = (centers[i]+centers[j])/2
            if not (covers(i,midpoint) and covers(j,midpoint)):
                continue
            delta = floor_z(i,midpoint)-floor_z(j,midpoint)
            angle = np.degrees(np.arccos(np.clip(normals[i] @ normals[j],-1.,1.)))
            robust_noise = 3. * 1.4826 * np.hypot(rows[i]["holdout_median_abs_inlier_residual_m"],
                                                   rows[j]["holdout_median_abs_inlier_residual_m"])
            if angle > 6 or abs(delta) > 0.08:
                gate = "discontinuity"
            elif abs(delta) <= max(0.02,robust_noise):
                gate = "within_pair_noise"
            else:
                gate = "candidate"
            weak_overlaps.append(dict(i=i,j=int(j),dt_s=stamps[i]-stamps[j],angle_deg=angle,
                                      innovation_m=delta,pair_3sigma_m=robust_noise,gate=gate))
            break
    out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/"ground_support_polygons.npz",vertices=np.vstack(body_polygons),
                        offsets=np.cumsum([0]+[len(vertices) for vertices in body_polygons]))
    for filename, data, fields in [
        ("ground_keyframes.csv",rows,["id","stamp_s","gx","gy","gz","has_plane","normal_x","normal_y","normal_z",
                                      "height_m","holdout_support","holdout_median_abs_inlier_residual_m","slope_deg",
                                     "voxel_floor_matches","voxel_matches","weak_support","patch_x","patch_y"]),
        ("ground_steps.csv",steps,["i","j","xy_m","measured_floor_step_m","slope_predicted_step_m","difference_m"]),
        ("ground_candidate_windows.csv",weak_overlaps,["i","j","dt_s","angle_deg","innovation_m","pair_3sigma_m","gate"]),
        ("ground_revisits.csv",repeat,["i","j","xy_m","travel_m","dt_s","floor_difference_m","normal_delta_deg",
                                       "i_x_imu","i_y_imu","i_z_imu","j_x_imu","j_y_imu","j_z_imu"])]:
        with (out/filename).open("w",encoding="utf8",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=fields)
            writer.writeheader()
            writer.writerows(data)
    supported=[r for r in rows if r["has_plane"]]
    weak_ids=set(weak_at_keyframe)
    s_pos=np.array([basis @ field(r,"xyz") for r in states])
    summary = {"session_id":identity["session_id"],"frame":"map_gravity",
               "frames":len(rows),"supported":len(supported),"holdout_support":quantiles([r["holdout_support"] for r in supported]),
               "floor_match_count":quantiles(list(floor_counts.values())),
               "weak_floor_windows":len(weak_windows),"lio_windows":len(all_counts),"weak_keyframes":len(weak_at_keyframe),
               "weak_keyframes_with_current_floor":sum(rows[i]["has_plane"] for i in weak_at_keyframe),
               "weak_keyframes_with_current_and_recent_floor":len(weak_overlaps),
               "weak_material_innovations":sum(r["gate"]=="candidate" for r in weak_overlaps),
               "sensor_height_above_extended_plane_m":quantiles([r["height_m"] for r in supported]),
               "local_slope_deg":quantiles([r["slope_deg"] for r in supported]),
               "holdout_median_inlier_residual_m":quantiles([r["holdout_median_abs_inlier_residual_m"] for r in supported]),
               "gravity_axis_odom_height_m":quantiles(s_pos[:,2]),"start_end_odom_height_difference_m":s_pos[-1,2]-s_pos[0,2],
               "short_step_abs_slope_disagreement_m":quantiles([abs(r["difference_m"]) for r in steps]),
               "revisit_signed_local_floor_m":quantiles([r["floor_difference_m"] for r in repeat]),
               "revisit_abs_local_floor_m":quantiles([abs(r["floor_difference_m"]) for r in repeat]),
               "revisit_above_15cm":sum(bool(abs(r["floor_difference_m"])>.15) for r in repeat),
               "revisit_above_15cm_with_weak_endpoint":sum(bool(abs(r["floor_difference_m"])>.15 and
                                                           (r["i"] in weak_ids or r["j"] in weak_ids)) for r in repeat),
               "revisit_abs_50m_travel_m":quantiles([abs(r["floor_difference_m"]) for r in repeat if r["travel_m"]>=50]),
               "scope":"Lower local near-horizontal surfaces and candidate XY revisits in odometry; no surveyed floor level or verified revisit identity."}
    (out/"ground_summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n",encoding="utf8")
    evidence=normal_evidence(plane_rows,rows,repeat,basis[2])
    (out/"floor_normal_observability.json").write_text(json.dumps(evidence,indent=2,allow_nan=False)+"\n",encoding="utf8")
    return summary


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("keyframes",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    print(json.dumps(analyze(args.keyframes,args.output),indent=2))
