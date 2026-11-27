"""Compare gravity-frame trajectories and same-stamp visual corrections on Baseline 02."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr


def vectors(table, columns):
    return table[columns.split()].to_numpy(dtype=float)


def trajectory_metrics(t, p):
    xyz0 = p - p[0]
    sampled = np.stack([np.interp(np.arange(0, t[-1], 1.), t, a) for a in p.T], axis=1)
    plane = np.c_[xyz0[:, :2], np.ones(len(p))]
    trend = plane @ np.linalg.lstsq(plane, xyz0[:, 2], rcond=None)[0]
    delta = p[(t >= 1775) & (t <= 1825)].mean(axis=0) - p[(t >= 10) & (t <= 60)].mean(axis=0)
    return dict(count=len(t), span_s=float(t[-1]), length_1s_m=np.linalg.norm(np.diff(sampled, axis=0), axis=1).sum(),
                z_span_m=np.ptp(p[:, 2]), z_after_plane_rms_m=np.sqrt(np.mean((xyz0[:, 2]-trend)**2)),
                p0_delta_m=delta.tolist(), p0_xy_m=np.linalg.norm(delta[:2]), p0_3d_m=np.linalg.norm(delta))


def case(run, appearance):
    states = pd.read_csv(run / "keyframes/states.csv", dtype={"stamp_ns": "int64"})
    lio = states[states.stage.str.startswith("LIO")].set_index("stamp_ns")
    time = lio.index.to_numpy() * 1e-9
    t = time-time[0]
    gravity = json.loads((run / "gravity_transform.json").read_text())["transform"]
    rotation = Rotation.from_matrix(gravity["R_GC"])
    p = rotation.apply(vectors(lio, "x y z"))
    attitude = rotation * Rotation.from_quat(vectors(lio, "qx qy qz qw"))
    # World-referenced tilt relative to the initial sensor pose (includes no base_link lever arm).
    relative = attitude * attitude[0].inv()
    rpy = relative.as_euler("xyz", degrees=True)
    report = trajectory_metrics(t, p)
    report.update(tilt_p05_p50_p95_deg=np.percentile(rpy[:, :2], [5, 50, 95], axis=0).tolist(),
                  bias_g_first_last=vectors(lio.iloc[[0, -1]], "bgx bgy bgz").tolist(),
                  bias_a_first_last=vectors(lio.iloc[[0, -1]], "bax bay baz").tolist(),
                  max_gravity_vector_change_m_s2=np.linalg.norm(vectors(lio, "gx gy gz")-
                      vectors(lio.iloc[[0]], "gx gy gz"), axis=1).max())
    # Non-overlapping one-second displacements. Project a horizontal plane at
    # the starting attitude into the later attitude, then along the actual XY step.
    ids = np.searchsorted(t, np.arange(0, t[-1], 1.0))
    delta = np.diff(p[ids], axis=0)
    plane_normal = relative[ids[:-1]].apply([0., 0., 1.])
    moving = (np.linalg.norm(delta[:, :2], axis=1) >= .20) & (plane_normal[:, 2] > .5)
    if moving.sum() >= 2:
        normal, step = plane_normal[moving], delta[moving]
        projected_z = -np.sum(normal[:, :2]*step[:, :2], axis=1)/normal[:, 2]
        yaw_step = (attitude[ids[1:]] * attitude[ids[:-1]].inv()).as_rotvec()[:, 2]
        turning = np.abs(yaw_step[moving]) >= np.deg2rad(15.)
        rho = float(spearmanr(projected_z, step[:, 2]).statistic) if np.ptp(projected_z) and np.ptp(step[:, 2]) else None
        motion = {"steps": int(moving.sum()), "min_xy_m": .20, "rho_attitude_predicted_vs_estimated_dz": rho}
        for label, mask in (("turn", turning), ("straight_or_gentle", ~turning)):
            if mask.any():
                motion[label] = {"steps": int(mask.sum()), "p95_abs_dz_mm": float(1000*np.quantile(abs(step[mask, 2]), .95))}
        report["moving_one_second_steps"] = motion
    vio = states[states.stage.str.startswith("VIO")].set_index("stamp_ns")
    common = lio.index.intersection(vio.index)
    if len(common):
        va, la = vio.loc[common], lio.loc[common]
        dp = rotation.apply(vectors(va, "x y z")-vectors(la, "x y z"))
        angle = (Rotation.from_quat(vectors(va, "qx qy qz qw")) *
                 Rotation.from_quat(vectors(la, "qx qy qz qw")).inv()).magnitude()
        def visual(mask):
            return dict(frames=int(mask.sum()), updated=int((va.stage[mask] == "VIO").sum()),
                        p95_delta_position_mm=1000*np.percentile(np.linalg.norm(dp[mask],axis=1),95),
                        p95_abs_delta_z_mm=1000*np.percentile(abs(dp[mask,2]),95),
                        p95_angle_deg=np.degrees(np.percentile(angle[mask],95)))
        report["visual"] = {"all": visual(np.ones(len(common),dtype=bool))}
        # Quantiles refer to the same bag across all front ends. Replay time_offset is applied once.
        if appearance is not None:
            import yaml
            params = yaml.safe_load((run / "resolved.yaml").read_text())["/**"]["ros__parameters"]
            offset = params["time_offset.img_time_offset"] + params["time_offset.exposure_time_init"]
            samples = appearance.sort_values("image_header_ns").drop_duplicates("image_header_ns")
            target = pd.DataFrame({"image_header_ns":common.to_numpy()-int(round(offset*1e9))})
            matched = pd.merge_asof(target.sort_values("image_header_ns"), samples, on="image_header_ns",
                                    direction="nearest", tolerance=2000)
            for col in ["mean_luma","mean_abs_gradient_1px"]:
                lo, hi = samples[col].quantile([.20,.80])
                for label, mask in [("low20", matched[col]<=lo), ("high20", matched[col]>=hi)]:
                    if mask.any():
                        report["visual"][col+"_"+label] = visual(mask.to_numpy())
            report["visual"]["image_matched"] = int(matched.mean_luma.notna().sum())
        # Distinguish bends using angular velocity around the gravity axis, across ~1 s.
        d = np.searchsorted(time, time-1).clip(0, len(time)-1)
        yaw_rate = np.abs((attitude * attitude[d].inv()).as_rotvec()[:,2]) / np.maximum(time-time[d],.01)
        turns = yaw_rate[lio.index.get_indexer(common)] > np.deg2rad(15)
        if turns.any():
            report["visual"]["turn_gt15deg_s"] = visual(turns)
    process = json.loads((run / "offline.json").read_text())
    report["offline_elapsed_s"] = process["elapsed_s"]
    report["peak_rss_mib"] = process["peak_rss_kib"]/1024
    report["inputs"] = process["counts"]
    return t, p, rpy, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--appearance", type=Path)
    parser.add_argument("--field-tum", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    appearance = pd.read_csv(args.appearance) if args.appearance else None
    fig, axes = plt.subplots(3,2,figsize=(12,10),layout="constrained")
    xy,z,pitch,roll,yaw,displacement = axes.flat
    reports = {}
    if args.field_tum:
        f = np.loadtxt(args.field_tum)
        tf = f[:,0]-f[0,0]
        reports["field_B1"] = trajectory_metrics(tf, f[:,1:4])
        xy.plot(f[:,1],f[:,2],color=".58",alpha=.7,label="Field B1",lw=1)
        z.plot(tf,f[:,3],color=".55",alpha=.8,label="Field B1",lw=1)
    for run in args.runs:
        t,p,rpy,reports[run.name] = case(run,appearance)
        xy.plot(*p[:,:2].T,label=run.name,lw=.9,alpha=.82)
        z.plot(t,p[:,2],label=run.name,lw=.8)
        roll.plot(t,rpy[:,0],lw=.65,label=run.name)
        pitch.plot(t,rpy[:,1],lw=.65,label=run.name)
        yaw.plot(t,np.rad2deg(np.unwrap(np.deg2rad(rpy[:,2]))),lw=.65,label=run.name)
        displacement.plot(t,np.linalg.norm(p[:,:2]-p[0,:2],axis=1),lw=.65,label=run.name)
    xy.set(xlabel="X (m)",ylabel="Y (m)",aspect="equal")
    z.set(xlabel="Elapsed time (s)",ylabel="Estimated gravity-frame Z (m)")
    roll.set(xlabel="Elapsed time (s)",ylabel="Roll relative to initial pose (deg)")
    pitch.set(xlabel="Elapsed time (s)",ylabel="Pitch relative to initial pose (deg)")
    yaw.set(xlabel="Elapsed time (s)",ylabel="Unwrapped yaw relative to initial pose (deg)")
    displacement.set(xlabel="Elapsed time (s)",ylabel="XY distance from start (m)")
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(args.output/"frontend_comparison.png",dpi=165)
    plt.close(fig)
    (args.output/"frontend_metrics.json").write_text(json.dumps(reports,indent=2,allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
