"""Map-gravity attitude, velocity and elevation of the front end; non-overlapping 1 s windows."""
import json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation,Slerp
from scipy.stats import spearmanr
from clock_analysis import read,q

def analyze(raw,out):
    tf=json.loads((raw/"gravity_transform.json").read_text(encoding="utf8"))["transform"]
    f=read(raw,"keyframes/states.csv")
    f=f[f.stage=="LIO"]
    origin=int(f.stamp_ns.iloc[0]); t=(f.stamp_ns.to_numpy()-origin)*1e-9
    rg=Rotation.from_matrix(np.array(tf["R_GC"])); rt=np.array(tf["translation_m"]); assert np.max(abs(rg.as_matrix()@rg.as_matrix().T-np.eye(3)))<1e-9
    co=f[list("xyz")].to_numpy()
    xyz=rg.apply(co)+rt; vel=rg.apply(f[["vx","vy","vz"]].to_numpy())
    rot=rg*Rotation.from_quat(f[["qx","qy","qz","qw"]].to_numpy())
    euler=rot.as_euler("xyz",degrees=True); yaw=np.rad2deg(np.unwrap(np.deg2rad(euler[:,2])))
    init=(t>=5)&(t<25)
    base=np.median(euler[init],axis=0)
    # Preserve full-resolution extrema; cumulative extrema separately use exact 1 s bins.
    n=int(np.floor(t[-1])); grid=np.arange(n+1,dtype=float); mid=grid[:-1]+.5
    sample=np.array([np.interp(grid,t,xyz[:,i]) for i in range(3)]).T
    v=np.array([np.interp(mid,t,vel[:,i]) for i in range(3)]).T
    e=Slerp(t,rot)(mid).as_euler("xyz",degrees=True)
    delta=np.diff(sample,axis=0); dz=delta[:,2]; dh=np.linalg.norm(delta[:,:2],axis=1)
    speed=np.linalg.norm(v,axis=1)
    yawrate=np.diff(np.interp(grid,t,yaw))
    ep=e[:,1]-base[1]; er=e[:,0]-base[0]
    move=dh>=.10
    slope=np.rad2deg(np.arctan2(dz,dh))
    ds=pd.DataFrame(dict(t_mid_s=mid,roll_map_deg=e[:,0],pitch_map_deg=e[:,1],yaw_map_wrapped_deg=e[:,2],
        yaw_map_unwrapped_deg=np.interp(mid,t,yaw),delta_roll_from_start_deg=er,delta_pitch_from_start_deg=ep,
        yaw_rate_deg_s=yawrate,speed_state_m_s=speed,vertical_velocity_state_m_s=v[:,2],horizontal_displacement_m=dh,
        delta_z_m=dz,z_rel_start_m=sample[1:,2]-sample[0,2],path_elevation_deg=slope,move=move,
        cumulative_up_m=np.cumsum(np.maximum(dz,0)),cumulative_down_m=np.cumsum(np.maximum(-dz,0))))
    ds.to_csv(out/"attitude_elevation_1s.csv",index=False,float_format="%.10g")
    cor={}
    for nm,a,b in (("pitch_centered__delta_z",ep,dz),("roll_centered__delta_z",er,dz),("pitch_centered__trajectory_inclination",ep,slope),
                   ("roll_centered__trajectory_inclination",er,slope),("abs_yaw_rate__abs_delta_z",abs(yawrate),abs(dz)),
                   ("speed__abs_delta_z",speed,abs(dz)),("state_vz__delta_z",v[:,2],dz)):
        cor[nm]={"pearson_r":float(np.corrcoef(a[move],b[move])[0,1]),"spearman_rho":float(spearmanr(a[move],b[move]).statistic)}
    gt=xyz[:,2]
    vk=np.linalg.norm(vel,axis=1); g=np.asarray(tf["gravity_camera_init_m_s2"])
    result={"origin_ns":origin,"R_GC":rg.as_matrix().tolist(),"mapped_initial_gravity":rg.apply(g).tolist(),
        "coordinate_convention":"extrinsic xyz roll pitch yaw; map_gravity<-body=R_GC @ R_Cbody; yaw has no external reference",
        "duration_s":float(t[-1]),"one_s_windows":n,"motion_by_horizontal_step_0p1_m":int(move.sum()),
        "map_gravity_full":{k:q(a) for k,a in (("roll_deg",euler[:,0]),("pitch_deg",euler[:,1]),("yaw_wrapped_deg",euler[:,2]),
            ("yaw_unwrapped_deg",yaw),("speed_norm",vk),("vertical_velocity",vel[:,2]),("relative_z_m",gt-gt[0]))},
        "roll_pitch_yaw_initial_window_p50":base,"roll_pitch_yaw_first":euler[0],"roll_pitch_yaw_last":euler[-1],
        "odom_z_range":float(np.ptp(co[:,2])),"odom_z_net":float(co[-1,2]-co[0,2]),"map_z_range":float(np.ptp(gt)),
        "map_z_min_time_s":float(t[gt.argmin()]),"map_z_max_time_s":float(t[gt.argmax()]),"map_z_net":float(gt[-1]-gt[0]),
        "cumulative_z_up_down_native_m":[float(np.maximum(np.diff(gt),0).sum()),float(np.maximum(-np.diff(gt),0).sum())],
        "cumulative_z_up_down_1s_m":[float(ds.cumulative_up_m.iloc[-1]),float(ds.cumulative_down_m.iloc[-1])],
        "one_s_net_m":float(dz.sum()),"moving_full_stats":{k:q(ds.loc[move,k]) for k in ("roll_map_deg","pitch_map_deg","yaw_rate_deg_s","speed_state_m_s","vertical_velocity_state_m_s","delta_z_m","path_elevation_deg")},
        "moving_correlations":cor}
    return result
