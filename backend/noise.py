"""Estimate per-axis scale from this run's adjacent GICP/odometry disagreement."""
import numpy as np
import gtsam
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from frame_io import rigid_inverse, transform


def motion_scale(T,dt):
    # Frontend emits on 1 m, 15 degrees or 5 s. Variances scale with the
    # largest consumed fraction, retaining a floor for very short updates.
    return float(np.sqrt(max(.2,np.linalg.norm(T[:3,3]),Rotation.from_matrix(T[:3,:3]).magnitude()/np.deg2rad(15),dt/5)))


def calibrate(builder,poses,times):
    eligible=[k for k in range(1,len(poses)) if k not in builder.excluded and k-1 not in builder.excluded]
    chosen=[eligible[i] for i in np.linspace(0,len(eligible)-1,min(48,len(eligible)),dtype=int)] if eligible else []
    rows=[]
    for k in chosen:
        a=builder.samples(k-1,[k-1])
        b=builder.samples(k,[k])
        z=rigid_inverse(poses[k-1])@poses[k]
        row={"indices":[k-1,k],"used":False}
        rows.append(row)
        if min(len(a),len(b))<80:
            continue
        sol=builder.fit(a,b,z)
        if not sol.converged:
            continue
        current=transform(b,sol.T_target_source)
        d=cKDTree(a).query(current)[0]
        v=cKDTree(current).query(a)[0]
        overlap=min(float(np.mean(d<.32)),float(np.mean(v<.32)))
        row["overlap"]=overlap
        if overlap<.45:
            continue
        delta=gtsam.Pose3.Logmap(gtsam.Pose3(rigid_inverse(z)@sol.T_target_source))
        scale=motion_scale(z,times[k]-times[k-1])
        kept=current[d<.32]
        lever=float(np.sqrt(np.mean(np.sum((kept-kept.mean(0))**2,axis=1))))
        cells=len(np.unique(np.floor(kept).astype(int),axis=0))
        row.update(used=True,twist_rad_m=delta.tolist(),motion_scale=scale,normalized_twist=(delta/scale).tolist(),
                   inlier_rms_lever_m=lever,occupied_1m_cells=cells)
    used=np.array([r["normalized_twist"] for r in rows if r["used"]])
    # Treat at most 64 spatially separate 1 m cells as independent. The
    # cohort's measured lever arm converts residual translation to angle.
    cells=float(np.clip(np.median([r["occupied_1m_cells"] for r in rows if r["used"]]),1,64)) if len(used) else 16.
    lever=max(.25,float(np.median([r["inlier_rms_lever_m"] for r in rows if r["used"]]))) if len(used) else 4.
    floor_t=.18/np.sqrt(cells)
    floor_r=np.arctan(floor_t/lever)
    floor=np.array([floor_r]*3+[floor_t]*3)
    if len(used)>=8:
        median=np.median(used,axis=0)
        mad=1.4826*np.median(np.abs(used-median),axis=0)
        q68=np.quantile(np.abs(used),.68,axis=0)
        scale=np.hypot(np.maximum(mad,q68),floor)
    else:
        median=mad=q68=np.zeros(6)
        scale=2*floor
    return {"order":["rx_rad","ry_rad","rz_rad","tx_m","ty_m","tz_m"], "eligible":len(eligible),
            "tested":len(chosen),"used":len(used),"samples":rows,"median":median.tolist(),"mad_sigma":mad.tolist(),
            "q68_abs":q68.tolist(),"registration_scale_floor":floor.tolist(),"effective_1m_cells":cells,
            "median_rms_lever_m":lever,"base_sigmas":scale.tolist(),
            "fallback":len(used)<8,"meaning":"within-bag adjacent scan/odometry consistency, not absolute sensor covariance"}


def loop_covariance(e,model):
    repeat=np.abs(gtsam.Pose3.Logmap(gtsam.Pose3(e["T_primary_inverse_neighbor"])))/np.sqrt(2.)
    # Nearest-point residual divided by the square root of the number of
    # occupied octants provides a conservative dimensional reference.
    t=e["rms_inlier_m"]/np.sqrt(max(1,e["occupied_centered_octants"]))
    r=np.arctan(t/max(.25,e["matched_lever_radius_m"]))
    scales=np.maximum.reduce([np.array(model["base_sigmas"]),repeat,np.array([r]*3+[t]*3)])
    eig,Q=np.linalg.eigh(e["normalized_point_plane_moment_source"])
    inflation=np.clip(np.median(eig)/np.maximum(eig,1e-7),1.,16.)
    cov=np.diag(scales)@(Q@np.diag(inflation)@Q.T)@np.diag(scales)
    e["repeatability_single_pair_rad_m"]=repeat.tolist()
    e["directional_variance_inflation"]=inflation.tolist()
    e["covariance_source_tangent"]=cov.tolist()
    e["sigmas_rot_rad_trans_m"]=np.sqrt(np.diag(cov)).tolist()
    return cov
