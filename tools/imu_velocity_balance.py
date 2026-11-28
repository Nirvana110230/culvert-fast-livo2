"""Sensor-time specific force, gyro bias and finite-window velocity balance."""
import argparse
import csv
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def imu_samples(bag):
    rows = []
    fields_checked = False
    for shard in sorted(bag.glob("*.db3"), key=lambda p: int(p.stem.split("_")[-1])):
        with sqlite3.connect(shard.resolve().as_uri()+"?mode=ro&immutable=1", uri=True) as db:
            topic = db.execute("SELECT id FROM topics WHERE name='/livox/imu' AND type='sensor_msgs/msg/Imu'").fetchone()
            for data, in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY id", topic):
                sec, ns, length = struct.unpack_from("<iII", data, 4)
                start = 4 + ((12 + length + 7)//8)*8
                gyro = struct.unpack_from("<3d", data, start+13*8)
                acc = struct.unpack_from("<3d", data, start+25*8)
                if not fields_checked:
                    m = deserialize_message(data, Imu)
                    for vector, decoded in ((gyro, m.angular_velocity), (acc, m.linear_acceleration)):
                        assert vector == (decoded.x, decoded.y, decoded.z)
                    assert (m.header.stamp.sec, m.header.stamp.nanosec) == (sec, ns)
                    fields_checked = True
                rows.append((sec*10**9+ns, *gyro, *acc))
    stamps = np.fromiter((r[0] for r in rows), np.int64)
    if np.any(np.diff(stamps)<=0):
        raise ValueError("non-monotonic IMU sensor time")
    return stamps, np.array([r[1:] for r in rows])


def angle(x, y):
    return float(np.rad2deg(np.arctan2(np.linalg.norm(np.cross(x,y)), np.dot(x,y))))


def analyze(args):
    root = args.session
    with (root/"keyframes/states.csv").open() as f:
        states = [r for r in csv.DictReader(f) if r["stage"]=="LIO"]
    def columns(names):
        return np.array([[float(r[n]) for n in names] for r in states])
    origin = int(states[0]["stamp_ns"])
    times = np.array([(int(r["stamp_ns"])-origin)*1e-9 for r in states])
    p = columns("xyz")
    rot = Rotation.from_quat(columns(("qx","qy","qz","qw")))
    g, ba, bg = (columns([prefix+a for a in "xyz"]) for prefix in ("g","ba","bg"))
    if args.init_norm is None:
        init_path=root/"diagnostics/imu_init_samples.csv"
        if not init_path.is_file():
            init_path=root/"keyframes/imu_init_samples.csv"
        with init_path.open() as f:
            init = np.array([[float(r["a"+k]) for k in "xyz"] for r in csv.DictReader(f)])
        init_norm = np.linalg.norm(init.mean(0))
    else:
        init_norm = args.init_norm
    scale = 9.81/init_norm
    stamps, raw = imu_samples(args.bag)
    t = (stamps-origin)*1e-9
    report = dict(origin_ns=origin, scale_m_s2_per_raw_unit=scale, imu_messages=len(t),
                  init_norm_source="algorithm log" if args.init_norm is not None else "initialization samples",
                  windows=[], directions=[])
    G = np.array(json.loads((root/"gravity_transform.json").read_text())["transform"]["R_GC"])
    interp = Slerp(times,rot)
    for lo, hi in zip(args.static[::2], args.static[1::2]):
        keep=(t>=lo)&(t<hi)
        sample=raw[keep]
        si=(times>=lo)&(times<hi)
        mean=sample.mean(0)
        grid=times[si]
        window_force=np.empty((len(grid),3))
        # Mean over all 200 Hz measurements in each 0.1 s LIO neighbourhood.
        for k, stamp in enumerate(grid):
            window_force[k]=scale*raw[np.searchsorted(t,stamp-.05):np.searchsorted(t,stamp+.05),3:].mean(0)
        residual=rot[si].apply(window_force-ba[si])+g[si]
        centered=p[si]-np.median(p[si],axis=0)
        obs=scale*mean[3:]-ba[si].mean(0)
        mid=float(grid[len(grid)//2])
        norm=lambda a:np.linalg.norm(a,axis=1)
        report["windows"].append(dict(interval=[lo,hi], samples=len(sample), states=int(si.sum()),
            raw_mean=mean.tolist(), raw_std=sample.std(0).tolist(),
            raw_slope_per_s=np.polyfit(t[keep]-mid,sample,1)[0].tolist(),
            gyro_minus_bg=(mean[:3]-bg[si].mean(0)).tolist(),bg=bg[si].mean(0).tolist(),ba=ba[si].mean(0).tolist(),
            gravity_odom=g[si].mean(0).tolist(), specific_force_residual_mean_odom=residual.mean(0).tolist(),
            mean_residual_norm_m_s2=float(np.linalg.norm(residual.mean(0))),
            position_radius_p95_m=float(np.quantile(norm(centered),.95)),
            attitude_radius_p95_deg=float(np.degrees(np.quantile((interp(mid).inv()*rot[si]).magnitude(),.95))),
            direction_vs_fixed_up_deg=angle(G@interp(mid).apply(obs),[0,0,1])))
        window=report["windows"][-1]
        window["accepted_stationary"]=bool(window["position_radius_p95_m"]<.025 and
            window["attitude_radius_p95_deg"]<.35 and max(window["raw_std"][:3])<.004 and
            np.linalg.norm(mean[:3])<.04 and max(window["raw_std"][3:])<.015 and
            abs(np.linalg.norm(mean[3:])*scale-9.81)<.2 and len(sample)>=.9*200*(hi-lo))
        if window["accepted_stationary"]:
            report["directions"].append(dict(t=mid,body=(obs/np.linalg.norm(obs)).tolist(),kind="stationary"))
    # In moving windows integral R*f dt + g*dt = v_end-v_start.
    # Estimate endpoint velocities independently of IMU velocities from a local
    # quadratic fit to LIO positions. Keep only well-conditioned, uninterrupted windows.
    def velocity(stamp):
        use=np.abs(times-stamp)<.65
        return np.polynomial.polynomial.polyfit(times[use]-stamp,p[use],2)[1]
    for lo in np.arange(2., times[-1]-9., 8.):
        hi=lo+8.
        if any(lo<b and hi>a for a,b in zip(args.static[::2],args.static[1::2])):
            continue
        i,j=np.searchsorted(t,[lo,hi])
        tt=np.r_[lo,t[i:j],hi]
        if np.max(np.diff(tt))>.025 or len(tt)<1400:
            continue
        mid=(lo+hi)/2
        force=np.column_stack([np.interp(tt,t,scale*raw[:,k+3])-np.interp(tt,times,ba[:,k]) for k in range(3)])
        impulse=np.trapz(interp(tt).apply(force),tt,axis=0)
        up=(impulse-(velocity(hi)-velocity(lo)))/(hi-lo)
        if abs(np.linalg.norm(up)-9.81)>.2:
            continue
        body=interp(mid).inv().apply(up)
        report["directions"].append(dict(t=mid,body=(body/np.linalg.norm(body)).tolist(),kind="velocity_balance",
            force_norm=float(np.linalg.norm(up)),delta_v=(velocity(hi)-velocity(lo)).tolist()))
    args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("bag",type=Path)
    a.add_argument("session",type=Path)
    a.add_argument("out",type=Path)
    a.add_argument("--static",type=float,nargs="*",default=[])
    a.add_argument("--init-norm",type=float,help="recorded frontend initialization norm for older sessions")
    args=a.parse_args()
    if len(args.static)%2:
        a.error("pass pairs of sensor-time offsets")
    analyze(args)
