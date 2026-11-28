"""Offline summaries of the seven streams. Read clocks.csv.gz made by bag_extract.py."""
import gzip
import io
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd
import yaml
from bag_extract import NAMES, SCHEMA

def load_clocks(raw):
    buffers={k:io.StringIO() for k in SCHEMA}
    with gzip.open(raw/"clocks.csv.gz","rt",encoding="utf8") as f:
        for line in f:
            buffers[line[0]].write(line[2:])
    out={}
    for k,buf in buffers.items():
        buf.seek(0)
        out[k]=pd.read_csv(buf,names=["shard","rowid","bag_ns",*SCHEMA[k]],float_precision="round_trip")
        buf.close()
    return out

def q(vals):
    a=np.asarray(vals,dtype=float)
    a=a[np.isfinite(a)]
    return dict(zip(["n","min","p05","p50","p95","p99","max","mean"],
        [len(a),*np.quantile(a,[0,.05,.5,.95,.99,1]).tolist(),a.mean()])) if len(a) else {"n":0}

def csvout(rows,out,name):
    pd.DataFrame(rows).to_csv(out/name,index=False,float_format="%.10g")

def read(raw,path): return pd.read_csv(raw/path,float_precision="round_trip")
def mix(raw,pattern):
    return pd.concat([read(raw,f.relative_to(raw)) for f in raw.glob(pattern)],ignore_index=True)
def native(obj):
    if isinstance(obj,np.ndarray): return obj.tolist()
    if isinstance(obj,np.generic): return obj.item()
    raise TypeError(type(obj))
def write_json(path,data):
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False,default=native)+"\n",encoding="utf8")

def analyze(raw,out,t,tools_dir):
    sys.path.insert(0,str(tools_dir))
    from inspect_bag_clocks import inspect_shared_reads
    streams=[]; gaps=[]; sh=[]; perfile={}
    metadata=yaml.safe_load((raw/"metadata.yaml").read_text())["rosbag2_bagfile_information"]
    meta={r["topic_metadata"]["name"]:r["message_count"] for r in metadata["topics_with_message_count"]}
    init=int(metadata["starting_time"]["nanoseconds_since_epoch"])
    periods={"I":100,"C":100,"M":100,"L":100,"U":5}
    for name,k in NAMES.items():
        f=t[k]
        src= "header_ns" if k in "ICLU" else {"M":"host_monotonic_after_get_ns","E":"host_monotonic_ns","T":"monotonic_before_ns"}[k]
        for clock,col in (("record","bag_ns"),("source",src)):
            a=f[col].to_numpy()
            d=np.diff(a)*1e-6
            r={"topic":name,"clock":clock,"field":col,"messages":len(f),"first_ns":int(a[0]),"last_ns":int(a[-1]),"span_s":(a[-1]-a[0])/1e9,"rate_hz":(len(a)-1)*1e9/(a[-1]-a[0]),**q(d),"duplicates":int(sum(d==0)),"backsteps":int(sum(d<0))}
            if k in periods:
                target=periods[k]
                for mul in (.5,1.5,2):
                    r[("below_" if mul<1 else "above_")+str(target*mul)+"ms"]=int(sum(d<target*mul if mul<1 else d>target*mul))
                for i in np.flatnonzero((d<target/2)|(d>target*1.5)):
                    idx=i+1
                    gaps.append({"topic":name,"clock":clock,"shard":int(f.shard.iloc[idx]),"rowid":int(f.rowid.iloc[idx]),"previous_ns":int(a[i]),"current_ns":int(a[idx]),"bag_elapsed_s":(f.bag_ns.iloc[idx]-init)*1e-9,"dt_ms":d[i]})
            streams.append(r)
        assert meta[name]==len(f)
        for shard,n in f.shard.value_counts().items():
            perfile.setdefault(int(shard),{})[name]=int(n)
    for info in metadata["files"]:
        ident=int(Path(info["path"]).stem.rsplit("_",1)[1])
        counts=perfile[ident]; n=sum(counts.values())
        assert n==info["message_count"]
        sh.append({"shard":ident,**counts,"count":n,"metadata_count":info["message_count"]})
    csvout(sh,out,"bag_shards.csv"); csvout(streams,out,"topic_clocks.csv"); pd.DataFrame(gaps).to_csv(out/"interval_outliers.csv.gz",index=False,float_format="%.10g",compression={"method":"gzip","mtime":0})
    m=t["M"]; im=t["I"]; l=t["L"]; e=t["E"]; c=t["T"]
    assert np.array_equal(m.frame_num,np.arange(len(m)))
    assert len(m)==len(im)==len(t["C"])
    pair=t["C"].header_ns.to_numpy()==im.header_ns.to_numpy()
    traces=mix(raw,"sensors/camera/timeshare_reads.csv*")
    tg=im.header_ns.to_numpy()[traces.frame_num]
    assert np.all(tg==traces.image_header_ns)
    anomal=np.flatnonzero((np.diff(im.header_ns)<50e6)|(np.diff(im.header_ns)>150e6))+1
    direct=inspect_shared_reads(raw/"sensors/camera",list(map(int,anomal)))
    direct.pop("examples_around_actual_abnormal_headers",None)
    delay=(m.publish_done_ns-m.host_after_get_ns)/1e6
    # Native tick frequency inferred from latch observations; centering prevents loss from subtracting epochs.
    tick=c.device_timestamp_value.to_numpy()
    mid=(c.monotonic_before_ns.to_numpy()-c.monotonic_before_ns.iloc[0])+(c.monotonic_after_ns-c.monotonic_before_ns).to_numpy()*.5
    rough_slope=np.median(np.diff(mid)/np.diff(tick))
    rough_intercept=np.median(mid-rough_slope*(tick-tick[0]))
    prelim=(mid-(rough_slope*(tick-tick[0])+rough_intercept))*1e-6
    good=np.abs(prelim-np.median(prelim))<100
    slope,intercept=np.polyfit((tick-tick[0])[good],mid[good],1)
    latch_resid=(mid-(slope*(tick-tick[0])+intercept))*1e-6
    latch_odd=[{"ordinal":int(i),"shard":int(c.shard.iloc[i]),"rowid":int(c.rowid.iloc[i]),"ticks":int(tick[i]),"affine_resid_ms":float(latch_resid[i])} for i in np.flatnonzero(~good)]
    events={}
    ea={k:v.sort_values("event_device_timestamp_raw").event_device_timestamp_raw.to_numpy() for k,v in e.groupby("event_name")}
    for n,seq in ea.items():
        d=np.diff(seq)*slope/1e6
        events[n]={"n":len(seq),"dt_ms":q(d),"gt150ms":int(sum(d>150)),"lt50ms":int(sum(d<50))}
    seq=m.device_timestamp_raw.to_numpy()
    rows={}
    for n in ("Line0RisingEdge","FrameStart","ExposureStart","ExposureEnd","Line0FallingEdge"):
        idx=np.searchsorted(ea[n],seq)
        left=np.clip(idx-1,0,len(ea[n])-1); right=np.clip(idx,0,len(ea[n])-1)
        ind=np.where(abs(ea[n][left]-seq)<abs(ea[n][right]-seq),left,right)
        rows[n]=ea[n][ind]
    # Anchor to actual exposure start, select the first exposure end after each start.
    starts=ea["ExposureStart"]; ends=ea["ExposureEnd"]
    idx=np.searchsorted(ends,starts)
    valid=idx<len(ends)
    pairs=(ends[idx[valid]]-starts[valid])*slope/1e6
    widths=pairs[(pairs>0)&(pairs<100)]
    offsets={k:q((v-seq)*slope/1e6) for k,v in rows.items() if k not in ("ExposureEnd","Line0FallingEdge")}
    dget=np.diff(m.host_monotonic_after_get_ns)*1e-6; ddev=np.diff(seq)*slope/1e6
    ex=[]
    for i in anomal:
        ex.append(dict(frame=i,header_dt_ms=(im.header_ns.iloc[i]-im.header_ns.iloc[i-1])/1e6,bag_dt_ms=(im.bag_ns.iloc[i]-im.bag_ns.iloc[i-1])/1e6,
            sdk_get_dt_ms=dget[i-1],device_dt_ms=ddev[i-1],publish_ms=delay.iloc[i],prev_publish_ms=delay.iloc[i-1],trace_available=bool((traces.frame_num==i).any())))
    csvout(ex,out,"camera_anomalies.csv")
    timewin=read(raw,"diagnostics/timing_windows.csv"); cuts=timewin.end_s.to_numpy()
    h=im.header_ns.to_numpy()
    # Reconstruct integer nanoseconds to within 1 us of double precision epoch.
    candidate=(h//10**9)+(h%10**9)*1e-9+.1
    matched=np.searchsorted(cuts,candidate-1e-6)
    mask=np.abs(cuts[np.clip(matched,0,len(cuts)-1)]-candidate)<1e-6
    unique=np.unique(matched[mask]); assert len(unique)==len(cuts)
    fm=mix(raw,"sensors/camera/frames.csv*")
    # Rotated frame CSV contains 18 data fields despite a retained 19-field header; source emits host without SDK-host field.
    conv=fm.lost_packets if fm.conversion_ms.isna().all() else fm.conversion_ms
    dr=json.loads((raw/"sensors/camera/driver_summary.json").read_text())
    cnt=json.loads((raw/"Log/input_counters.json").read_text())
    first_valid=int(np.flatnonzero(mask)[0]); last_valid=int(np.flatnonzero(mask)[-1])
    frame_rec_offset=q((im.bag_ns.to_numpy()-m.host_after_get_ns.to_numpy())*1e-6)
    event_gaps={}
    for n,g in e.groupby("event_name"):
        a=g.event_device_timestamp_raw.to_numpy()
        event_gaps[n]=int(sum(np.diff(a)<0))
    start_info={"first_accepted_frame":first_valid,"last_accepted_frame":last_valid,"before":first_valid,"after":len(im)-1-last_valid,
        "unmatched_in_middle_frames":np.flatnonzero(~mask[first_valid:last_valid+1]).tolist(),"n_images_matching_accepted_cuts":int(mask.sum()),
        "distinct_accepted_cuts":len(unique),"new_cut_s":.1,"mapper_image_callbacks":cnt["image_rx"]}
    start_info["unmatched_in_middle_frames"]=[first_valid+i for i in start_info["unmatched_in_middle_frames"]]
    # Host epoch is a bag arrival observation. Its offset with a sensor header is summarized, never called transport latency.
    offsets_host={k:q((t[k].bag_ns-t[k].header_ns)*1e-9) for k in ("I","C","L","U")}
    # Relative sensor-clock consistency alone cannot verify absolute exposure alignment.
    it=t["U"].header_ns.to_numpy(); ip=np.searchsorted(it,h+100_000_000)
    ip=np.clip(ip,1,len(it)-1); nearest=np.where(abs(it[ip]-h-100_000_000)<abs(it[ip-1]-h-100_000_000),it[ip],it[ip-1])
    ticks_relative=(seq-seq[0])*slope*1e-6
    rel_head=(h-h[0])*1e-6
    image_vs_camera_ms=q(rel_head-ticks_relative)
    box={}
    for label,rf,cname in [("lidar",l,"scan_s"),("imu",t["U"],"stamp_s")]:
        if label=="lidar":
            traced=read(raw,"diagnostics/preprocess_timing.csv")[cname].to_numpy()
        else:
            traced=read(raw,"diagnostics/imu_init_samples.csv")[cname].to_numpy()
        hs=rf.header_ns.to_numpy()
        loc=np.searchsorted(hs,np.rint(traced*1e9).astype(np.int64)-800)
        dif=abs(hs[loc]*1e-9-traced)
        box[label]={"bag_ord_first_trace":int(loc[0]),"bag_ord_last_trace":int(loc[-1]),"matching_trace_times_within_1us":int(sum(dif<=1e-6)),"n_trace":len(traced)}
    u={"nearest_imu_minus_corrected_image_ms":q((nearest-h-100_000_000)*1e-6),"relative_image_header_minus_device_clock_ms":image_vs_camera_ms,"stream_membership":box,"frame_num_bounds":[int(m.frame_num.min()),int(m.frame_num.max())],"lost_packets":int(m.lost_packets.sum()),"events":events,
        "frame_fields_distinct":{k:int(m[k].nunique()) for k in ("trigger_index","frame_counter","exposure_frame_us","gain_frame")},
        "driver":{k:v for k,v in dr.items() if k in ("frames","timeouts","events_written","events_queue_dropped","exit_code","settings_restored","mode","calibrated","error_present")},
        "usb":json.loads((raw/"sensors/camera/usb_stats.json").read_text()),"frame_trace_count":len(traces),"frame_trace_min":int(traces.frame_num.min()),
        "metadata_image_count_match":True,"camera_info_same_stamp_ordinal":int(pair.sum()),
        "header_matches_exact_lidar":int(im.header_ns.isin(l.header_ns).sum()),"shared_trace":direct,
        "sdk_get_to_publish_ms":q(delay),"get_to_publish_gt100":int(sum(delay>100)),"image_record_minus_sdk_get_ms":frame_rec_offset,
        "frame_device_dt_ms":q(ddev),"frame_get_mono_dt_ms":q(dget),"latch_ns_per_tick":slope,"latch_inferred_mhz":1000/slope,
        "latch_span_ms":q((c.monotonic_after_ns-c.monotonic_before_ns)*1e-6),"latch_failed":int(sum((c.latch_rc!=0)|(c.read_rc!=0))),
        "latch_affine_residual_ms":q(latch_resid[good]),"latch_excluded_100ms":latch_odd,"exposure_duration_ms":q(widths),"exposure_start_count":len(starts),
        "nearby_events_minus_device_frame_ms":offsets,"event_source_backsteps_original_order":event_gaps,"frame_csv_rows":len(fm),
        "frame_csv_min_frame":int(fm.frame_num.min()),"convert_ms_retained":q(conv),"image_anomalies":ex,
        "bag_sensor_epoch_offset_s":offsets_host,"coverage":start_info}
    return {"total":int(sum(meta.values())),"shards":len(sh),"duration_s":metadata["duration"]["nanoseconds"]/1e9,
        "bag_start_ns":init,"topic_counts":meta,"streams":streams,"camera":u}
