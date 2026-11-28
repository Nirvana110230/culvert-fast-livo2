"""Analyze estimator states, final matcher residuals, static gyro gates and sampled resources."""
from collections import Counter
import re
import sys
import json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from clock_analysis import csvout,q,read

def cols(f,prefix): return f[[prefix+k for k in "xyz"]].to_numpy()
def norm(a): return np.linalg.norm(a,axis=-1)
def angle(q1,q2): return 2*np.arccos(np.clip(np.abs(np.sum(q1*q2,axis=-1)),0,1))
def stats(f,names): return {k:q(f[k]) for k in names}
def read_json(p): return json.loads(p.read_text(encoding="utf-8"))
def series_bins(out,f,t,fields,name,seconds=60):
    frame=f[fields].copy(); frame["bin"]=np.floor(np.maximum(t,0)/seconds).astype(int)
    rows=[]
    for bn,g in frame.groupby("bin"):
        row={"start_s":int(bn*seconds),"end_open_s":int((bn+1)*seconds),"samples":len(g)}
        for col in fields:
            vals=g[col].dropna()
            if len(vals):
                for k,fun in [("mean",np.mean),("p50",np.median),("p95",lambda x:np.quantile(x,.95)),("max",np.max)]:
                    row[col+"_"+k]=fun(vals)
        rows.append(row)
    csvout(rows,out,name)
    return rows

def static_updates(out,raw,desk,vi,imu,origin):
    text='\n'.join((raw/n).read_text(encoding="utf8",errors="replace") for n in ("algorithm.log.1","algorithm.log"))
    actual=[(float(a),int(b)) for a,b in re.findall(r'static gyro update t=(\d+\.\d+) n=(\d+)',text)]
    epochs=desk.prop_start_s.to_numpy()
    imu_ns=imu.header_ns.to_numpy()
    imu_secs=(imu_ns//10**9)+(imu_ns%10**9)*1e-9
    assert np.min(np.diff(imu_secs))>0
    gyro=imu[["wx","wy","wz"]].to_numpy(); accel=imu[["ax","ay","az"]].to_numpy()
    qs=vi[["qx","qy","qz","qw"]].to_numpy()
    # First propagated LIO starts at the end of initialization. VIO posterior at
    # epoch i is the state inspected before the next forward propagation.
    pos=np.vstack(([0,0,0],cols(vi,"")[:-1])); quat=np.vstack(([0,0,0,1],qs[:-1]))
    spd=np.r_[0,norm(cols(vi,"v"))[:-1]]; bg=np.vstack(([0,0,0],cols(vi,"bg")[:-1]))
    ba=np.vstack(([0,0,0],cols(vi,"ba")[:-1]))
    assert len(epochs)==len(pos) and np.max(abs(epochs[1:]-vi.stamp_ns.to_numpy()[:-1]*1e-9))<1e-6
    indices=np.searchsorted(imu_secs,epochs,side="right")
    start=0; outrows=[]
    for i,stamp in enumerate(epochs):
        moved=(norm(pos[i]-pos[start])>.025 or angle(quat[start],quat[i])>.006 or spd[i]>.08 or
               (i>0 and stamp-imu_secs[indices[i]-1]>.1))
        if moved: start=i
        span=stamp-epochs[start]
        if span<4: continue
        lo,hi=indices[start],indices[i]; sample=gyro[lo:hi]
        if len(sample)>=700 and sample.std(axis=0).max()<.004 and norm(sample.mean(axis=0))<.04:
            mu=sample.mean(axis=0); innovation=mu-bg[i]
            row={"t_s":(stamp-origin*1e-9),"start_s":epochs[start]-origin*1e-9,"n":len(sample),"window_s":span,
                 "max_gyro_axis_std_rad_s":sample.std(axis=0).max(),"pre_speed_m_s":spd[i],
                 "endpoint_translation_m":norm(pos[i]-pos[start]),"endpoint_rotation_rad":angle(quat[start],quat[i]),
                 "log_match":any(abs(stamp-t)<1e-6 and n==len(sample) for t,n in actual),
                 "innovation_norm_rad_s":norm(innovation),"next_VIO_bg_delta_norm_rad_s":norm(cols(vi.iloc[i:i+1],"bg")[0]-bg[i])}
            for k,v in (("omega",mu),("pre_bg",bg[i]),("pre_ba",ba[i]),("omega_minus_bg",innovation),
                        ("omega_std",sample.std(axis=0)),("raw_acc",accel[lo:hi].mean(axis=0)),
                        ("next_VIO_bg",cols(vi.iloc[i:i+1],"bg")[0])):
                row.update({k+"_"+a:float(v[j]) for j,a in enumerate("xyz")})
            outrows.append(row)
        start=i
    corr=sum(r["log_match"] for r in outrows)
    assert corr==len(actual), (corr,len(actual))
    csvout(outrows,out,"static_updates.csv")
    return {"guard_reconstructions":len(outrows),"confirmed_by_rotated_log":len(actual),"confirmed_all":corr==len(actual),
        "t_s":[round(r["t_s"],4) for r in outrows],"sample_counts":dict(Counter(r["n"] for r in outrows)),
        "confirmed_t_s":[round(r["t_s"],4) for r in outrows if r["log_match"]],
        "innovation_norm_rad_s":q([r["innovation_norm_rad_s"] for r in outrows]),"records":outrows}

def analyze(raw,out,bag,tools_dir,imu_report,c2):
    sys.path.insert(0,str(tools_dir))
    from summarize_frontend_timing import summarize
    d=raw/"diagnostics"; diag=summarize(d)
    li0=read(raw,"keyframes/states.csv"); stages=li0.stage.value_counts().to_dict()
    li=li0[li0.stage.str.startswith("LIO")].reset_index(drop=True)
    vi=li0[li0.stage.str.startswith("VIO")].reset_index(drop=True)
    assert np.array_equal(li.stamp_ns,vi.stamp_ns)
    origin=int(li.stamp_ns.iloc[0]); t=(li.stamp_ns-origin).to_numpy()*1e-9
    xyz=cols(li,""); qs=li[["qx","qy","qz","qw"]].to_numpy()
    vq=vi[["qx","qy","qz","qw"]].to_numpy()
    delta=cols(vi,"")-xyz
    dv=norm(delta); da=np.rad2deg(angle(qs,vq)); dg=norm(cols(vi,"bg")-cols(li,"bg"))
    plane=read(d,"plane_constraints.csv"); planes=plane[plane["class"]=="all"].reset_index(drop=True)
    assert max(abs(planes.cut_s-li.stamp_ns*1e-9))<1e-6
    speed=norm(cols(li,"v")); g=cols(li,"g"); up=-g[0]/norm(g[0])
    heights=xyz@up
    # Match the prior report's trajectory-height definition in its gravity frame below.
    residual=planes.abs_sum_m/planes["count"]
    lio=pd.DataFrame(dict(t_s=t,lidar_matches=planes["count"],mean_abs_plane_m=residual,window_p95_plane_m=planes.p95_abs_m,lidar_iterations=planes.iterations,
                          vio_position_step_m=dv,vio_attitude_step_deg=da,vio_gyro_bias_step_rad_s=dg,speed_m_s=speed,gravity_norm=norm(g),
                          bg_norm_rad_s=norm(cols(li,"bg")),ba_norm_m_s2=norm(cols(li,"ba")),height_m=heights))
    series_bins(out,lio,t,list(lio.columns[1:]),"estimator_60s.csv")
    worst=[]
    for field in ["vio_position_step_m","vio_attitude_step_deg","mean_abs_plane_m","speed_m_s"]:
        for idx in lio[field].nlargest(5).index:
            worst.append({"rank_by":field,**lio.iloc[idx].to_dict()})
    csvout(worst,out,"estimator_extremes.csv")
    info=read(d,"lio_information_6d.csv"); info["condition_number"]=info.lambda5/info.lambda0
    weight=read(d,"measurement_weight_sources.csv")
    wr={}
    for label,f in weight.groupby("class"):
        wr[label]={"n":len(f),"points":int(f["count"].sum()),"mean_effective_variance_m2":f.inverse_weight_sum_m2.sum()/f["count"].sum(),
            "mean_plane_variance_m2":f.sigma_plane_sum_m2.sum()/f["count"].sum(),"mean_point_variance_m2":f.sigma_point_sum_m2.sum()/f["count"].sum(),
            "total_vertical_translation_information":f.vertical_translation_information.sum(),"total_yaw_rotation_information":f.yaw_rotation_information.sum()}
    desk=read(d,"deskew_timing.csv")
    su=static_updates(out,raw,desk,vi,bag["U"],origin)
    proc=read(raw,"Log/processing.csv")
    rss=pd.read_json(raw/"resources.jsonl",lines=True)
    for label in sorted({k for r in rss.process_rss_bytes for k in r}):
        rss["rss_"+label+"_mib"]=[r.get(label,np.nan)/2**20 for r in rss.process_rss_bytes]
    rss["available_gib"]=rss.mem_available_bytes/2**30
    rss["free_disk_gib"]=rss.disk_free_bytes/2**30
    m=bag["M"]; im=bag["I"]
    set_image_times=im.header_ns.to_numpy()+100_000_000
    # Route observations from event timestamps in wall time. These align approximately to a measured camera acquisition on this host.
    markers={}
    for line in (raw/"events.txt").read_text().splitlines():
        k,val=line.split(None,1); ns=pd.Timestamp(val.replace(",",".")).value
        i=int(np.argmin(abs(m.host_after_get_ns.to_numpy()-ns)))
        markers[k]={"nearest_camera_frame":i,"nearest_host_get_minus_marker_s":(m.host_after_get_ns.iloc[i]-ns)/1e9,
            "nearest_sensor_cut_s":(set_image_times[i]-origin)/1e9,"marker_utc8":str(pd.Timestamp(val.replace(",",".")))}
    # Sampled current-cycle timings, not cumulative averages over all LIO and VIO calls.
    proc["sensor_t_s"]=proc.state_s-origin*1e-9
    proc["image_pending_vs_vio"]=proc.image_rx-proc.vio_frames
    proc_cols=["processing_ms","cycle_ms","lidar_queue","imu_queue","image_queue","image_pending_vs_vio"]
    series_bins(out,proc,proc.sensor_t_s,proc_cols,"processing_60s.csv")
    lr=read(raw,"Log/visual_lru.csv")
    lr["sensor_t_s"]=np.interp(lr.monotonic_s,proc.wall_s,proc.sensor_t_s)
    for k in ("image_bytes","rss_bytes","protected_bytes"): lr[k.replace("_bytes","_mib")]=lr[k]/2**20
    lcols=["used_points","points","features","images","image_mib","rss_mib","protected_mib","maintenance_ms"]
    series_bins(out,lr,lr.sensor_t_s,lcols,"visual_cache_60s.csv")
    rcols=[k for k in rss if k.startswith("rss_")]+["available_gib","free_disk_gib"]
    series_bins(out,rss,rss.elapsed_s,rcols,"resources_60s.csv")
    csvout(rss[["elapsed_s",*rcols]].to_dict("records"),out,"resources_samples.csv")
    c=read_json(raw/"Log/input_counters.json"); ci=imu_report
    baseline=xyz[(t>=5)&(t<25)].mean(axis=0); radius=norm(xyz-baseline)
    cross={str(x):float(t[np.flatnonzero(radius>x)[0]]) for x in (.05,.1,.5,1)}
    stable_speed={f"{lo}-{hi}": {"LIO":q(speed[(t>=lo)&(t<hi)]),"VIO":q(norm(cols(vi,"v"))[(t>=lo)&(t<hi)])}
                  for lo,hi in ((5,25),(2059,2079))}
    angle_g=np.rad2deg(np.arccos(np.clip((g@g[0])/(norm(g)*norm(g[0])),-1,1)))
    # Recompute the raw IMU moments in both selected intervals to verify CDR extraction.
    u=bag["U"]; ut=(u.header_ns-origin)*1e-9
    for win in ci["windows"]:
        lo,hi=win["interval"]; arr=u[["wx","wy","wz","ax","ay","az"]][(ut>=lo)&(ut<hi)]
        assert len(arr)==win["samples"] and np.max(abs(arr.mean().to_numpy()-win["raw_mean"]))<1e-12
    r_last=lr.iloc[-1]; motion=read_json(raw/"session_subset.json")
    stage_counts={"before_move":sum(row["t_s"]<markers["MOVE_START"]["nearest_sensor_cut_s"] for row in su["records"]),
        "after_2000s":sum(row["t_s"]>=2000 for row in su["records"])}
    def zstats(x):
        return {"first":x.iloc[0],"last":x.iloc[-1],**q(x)}
    zero_s=float(t[-1]); stats_by_phase=[]
    phases=[("start_static",5,25),("middle",100,2000),("end_static",2059,2079)]
    for name,lo,hi in phases:
        f=lio[(t>=lo)&(t<hi)]; e=li[(t>=lo)&(t<hi)]
        row={"phase":name,"start_s":lo,"end_s":hi,"n":len(f)}
        for col in ("speed_m_s","lidar_matches","mean_abs_plane_m","vio_position_step_m","vio_attitude_step_deg","bg_norm_rad_s","ba_norm_m_s2"):
            row[col+"_p50"]=float(f[col].median()); row[col+"_p95"]=float(f[col].quantile(.95))
        row["g_norm_mean"]=float(np.mean(norm(cols(e,"g")))); stats_by_phase.append(row)
    csvout(stats_by_phase,out,"phase_estimator.csv")
    lr_total={k:int(r_last[k]) for k in ("points_evicted_total","features_evicted_total","pruned_observations_total","pressure_points_total","images_created","images_released",
        "images_last_ref_freed","bytes_last_ref_freed","images_detached_external_ref","external_image_refs","system_shortfall_bytes")}
    compare={"previous_count":c2["offline"]["counts"],"previous_elapsed_s":c2["offline"]["elapsed_s"],"previous_peak_rss_kib":c2["offline"]["peak_rss_kib"],
             "previous_static_count":len(c2["su"]),"previous_static_bounds":[c2["su"][0]["relative_s"],c2["su"][-1]["relative_s"]],
             "previous_cases":c2["cases"],"previous_imu_windows":c2["imu"]["windows"]}
    rs={k:zstats(rss[k].dropna()) for k in rcols}
    active=rss.dropna(subset=["rss_algorithm_mib"])
    rprogress={}
    for lo,hi in ((0,120),(120,600),(600,1200),(1200,1800),(1800,2105)):
        a=active[(active.elapsed_s>=lo)&(active.elapsed_s<hi)]
        if len(a)>2:
            rprogress[str(lo)+"-"+str(hi)]={"samples":len(a),"rss_first_mib":float(a.rss_algorithm_mib.iloc[0]),"rss_last_mib":float(a.rss_algorithm_mib.iloc[-1]),
                "rss_slope_mib_min":float(np.polyfit(a.elapsed_s/60,a.rss_algorithm_mib,1)[0]),"median_mib":float(a.rss_algorithm_mib.median())}
    # This independent full-run gauge uses z along initialization gravity and an orientation-invariant best-fit horizontal plane.
    basis=Rotation.align_vectors(np.array([[0.,0.,1.]]),up[None,:])[0]; zxyz=basis.apply(xyz)
    coeff=np.linalg.lstsq(np.c_[zxyz[:,:2],np.ones(len(zxyz))],zxyz[:,2],rcond=None)[0]
    plane_rms=float(np.sqrt(np.mean((zxyz[:,2]-np.c_[zxyz[:,:2],np.ones(len(zxyz))]@coeff)**2)))
    return {"start_departure_s":cross,"static_state_speeds":stable_speed,"gravity_angle_vs_start_deg":q(angle_g),"origin_ns":origin,"duration_s":zero_s,"counters":c,"stages":stages,"timing":diag,"static":su,"static_phase_counts":stage_counts,
        "imu_windows":ci["windows"],"markers":markers,"sampled_processing":stats(proc,proc_cols),"processing_over100ms":int((proc.processing_ms>100).sum()),
        "cycle_over100ms":int((proc.cycle_ms>100).sum()),"processing_rows":len(proc),"visual_cache":stats(lr,lcols),"last_cache_counters":lr_total,
        "cache_first_last":{k:[lr[k].iloc[0],lr[k].iloc[-1]] for k in ("frame","images","image_bytes","rss_bytes","used_points","protected_bytes")},
        "visual_cache_rows":len(lr),"resource_summary":rs,"resource_rows":len(rss),"rss_trend":rprogress,"phases":stats_by_phase,
        "lio_info":stats(info,["points","valid","lambda0","lambda5","condition_number","world_tup_information","weak_translation_up","rms_arm_m"]),
        "measurement_weight":wr,"state_metrics":stats(lio,list(lio.columns[1:])),"init_gravity_track_range_m":np.ptp(zxyz[:,2]),
        "init_gravity_track_plane_rms_m":plane_rms,"bg_first_last":cols(li,"bg")[[0,-1]].tolist(),"ba_first_last":cols(li,"ba")[[0,-1]].tolist(),
        "g_first_last":g[[0,-1]].tolist(),"previous":compare,"session":motion}
