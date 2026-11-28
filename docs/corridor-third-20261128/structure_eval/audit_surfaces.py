"""Recompute surface and direction residuals on archived poses, without solving a graph.

On the board set PYTHONPATH to <release>/backend:<release>/backend/pydeps and
OPENBLAS_NUM_THREADS=1. Pass the full_z directory. JSON is written to stdout.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from frame_io import frames_and_gravity, transform
from loop_edges import EdgeBuilder
from shared_planes import local_patches
from verify import load_tum


def audit(root):
    pgo, structure = root / "pgo", root / "structure"
    plan = json.loads((pgo / "plan.json").read_text())
    native = Path(plan["parameters"]["index"])
    frames, gravity, _ = frames_and_gravity(native, Path(plan["parameters"]["gravity"]))
    stamps, b = load_tum(pgo / "trajectory_after.tum")
    stamps_c, c = load_tum(structure / "trajectory_after.tum")
    assert stamps == stamps_c == [f.stamp for f in frames]
    a = np.array([gravity @ f.odom for f in frames])
    trajectories = {"A": a, "B": b, "C": c}
    build = EdgeBuilder(frames, a, np.array(plan["holdout_centers_xy_m"]), set(plan["heldout_frame_indices"]))
    groups = local_patches(build, b, np.arange(len(frames)))
    archived = json.loads((structure / "shared_plane_associations.json").read_text())
    assert len(groups) == len(archived)
    blocks, planes = [], []
    for j, (g, ar) in enumerate(zip(groups, archived)):
        obs = g["obs"]
        ids = [i for i, _ in obs]
        assert g["kind"] == ar["surface"]
        assert ids[2::5] == ar["validation_ids"] and [i for k,i in enumerate(ids) if k%5!=2] == ar["training_ids"]
        assert np.allclose(g["center"], ar["anchor"], atol=1e-7, rtol=0)
        direction = b[np.minimum(np.array(ids)+2,len(b)-1),:2,3] - b[np.maximum(np.array(ids)-2,0),:2,3]
        direction = direction[np.linalg.norm(direction,axis=1)>.6]
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        cross = len(direction)>1 and np.min(np.abs(direction @ direction.T)) < np.cos(np.deg2rad(30))
        planes.append(dict(id=j,surface=g["kind"],x=g["center"][0],y=g["center"][1],z=g["center"][2],
            span_s=(stamps[max(ids)]-stamps[min(ids)])*1e-9,cross_heading=bool(cross),
            offset_m=ar["optimized_offset_at_anchor_m"],normal_change_deg=float(np.degrees(np.arccos(
                np.clip(np.dot(ar["initial_normal"],ar["optimized_normal"]),-1,1))))))
        for stage, poses in trajectories.items():
            xyz = np.array([transform(pts,poses[i]) for i,pts in obs])
            mask = np.arange(len(obs)) % 5 != 2
            train = xyz[mask].reshape(-1,3)
            center = train.mean(axis=0)
            n = np.linalg.eigh((train-center).T @ (train-center))[1][:,0]
            if n @ g["normal"] < 0:
                n = -n
            residual = (xyz-center) @ n * 1000
            for k, i in enumerate(ids):
                blocks.append([j,g["kind"],i,stage,"train" if mask[k] else "held",*residual[k].tolist()])
    imu = json.loads((root / "imu.json").read_text())
    origin = imu["origin_ns"]
    with (native.parent / "states.csv").open() as f:
        states = [r for r in csv.DictReader(f) if r["stage"] == "LIO"]
    state_t = np.array([(int(r["stamp_ns"])-origin)*1e-9 for r in states])
    interp = Slerp(state_t,Rotation.from_quat([[float(r[k]) for k in ("qx","qy","qz","qw")] for r in states]))
    times=(np.array(stamps)-origin)*1e-9
    directions=[]
    for d in imu["directions"]:
        t=d["t"]
        if not times[0]<=t<=times[-1]:
            continue
        i=int(np.argmin(abs(times-t)))
        body=frames[i].odom[:3,:3].T @ interp(t).apply(d["body"])
        row=dict(t=t,frame=i,kind=d["kind"],force_norm=d.get("force_norm"),delta_v_norm=(
            float(np.linalg.norm(d["delta_v"])) if "delta_v" in d else None),x=b[i,0,3],y=b[i,1,3])
        for stage,poses in trajectories.items():
            up=poses[i,:3,:3] @ body
            row[stage+"_deg"]=float(np.degrees(np.arccos(np.clip(up[2]/np.linalg.norm(up),-1,1))))
        directions.append(row)
    return dict(planes=planes, directions=directions, block_columns=["plane","surface","frame","stage","split","e1_mm","e2_mm","e3_mm"],blocks=blocks)


if __name__ == "__main__":
    print(json.dumps(audit(Path(sys.argv[1])),allow_nan=False))
