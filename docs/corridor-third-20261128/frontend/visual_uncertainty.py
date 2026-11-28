"""Statistics of local visual support, paired filter posterior and end geometry."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from clock_analysis import read,q,write_json

def main(raw,second,source,out):
    st=read(raw,"keyframes/states.csv"); li=st[st.stage=="LIO"].reset_index(drop=True); vi=st[st.stage.str.startswith("VIO")].reset_index(drop=True)
    assert np.array_equal(li.stamp_ns,vi.stamp_ns)
    t=(li.stamp_ns-li.stamp_ns.iloc[0]).to_numpy()*1e-9
    zones={"all":np.ones(len(t),bool),"start_5_25":(t>=5)&(t<25),"end_2059_2079":(t>=2059)&(t<2079)}
    names=["var_rx","var_ry","var_rz","var_x","var_y","var_z"]
    var=[]
    for part,mask in zones.items():
        for dim in names:
            a=li.loc[mask,dim].to_numpy(); b=vi.loc[mask,dim].to_numpy()
            assert min(a.min(),b.min())>=0
            sigma=180/np.pi if dim.startswith("var_r") else 1000
            for stage,arr in (("LIO",a),("VIO",b)):
                var.append({"window":part,"dim":dim,"stage":stage,"n":len(arr),"p50_variance":np.median(arr),"p95_variance":np.quantile(arr,.95),
                    "p50_sigma_deg_or_mm":np.median(np.sqrt(arr))*sigma,"p95_sigma_deg_or_mm":np.quantile(np.sqrt(arr),.95)*sigma,
                    "first_variance":arr[0],"last_variance":arr[-1],"p50_paired_VIO_over_LIO_var":np.median(b/a),"p95_paired_VIO_over_LIO_var":np.quantile(b/a,.95),
                    "paired_smaller":int(np.sum(b<a)),"paired_equal":int(np.sum(b==a)),"paired_larger":int(np.sum(b>a))})
    pd.DataFrame(var).to_csv(out/"posterior_variance.csv",index=False,float_format="%.12g")
    lc=read(raw,"Log/visual_lru.csv")
    ass=(lc.frame.to_numpy()-1).astype(int); ltime=t[ass]
    lru={}
    for part,mask in {"all":np.ones(len(lc),bool),"start_5_25":(ltime>=5)&(ltime<25),"end_2059_2079":(ltime>=2059)&(ltime<2079)}.items():
        f=lc[mask];lru[part]={"n":len(f),**{k:q(f[k]) for k in ("used_points","voxels","points","features")},
            "used_over_maintained":q(f.used_points/f.points),"zero":int(sum(f.used_points==0))}
    logs='\n'.join((raw/n).read_text(encoding="utf8",errors="replace") for n in ("algorithm.log.1","algorithm.log"))
    trace={int(a):int(b) for a,b in re.findall(r"\[VIO\] frame=(\d+) points=(\d+)",logs)}
    same={int(r.frame):int(r.used_points) for r in lc.itertuples()}
    overlap=trace.keys()&same.keys(); assert all(trace[i]==same[i] for i in overlap)
    def positions(root):
        f=read(root,"keyframes/states.csv"); f=f[f.stage=="LIO"].reset_index(drop=True)
        tf=json.loads((root/"gravity_transform.json").read_text(encoding="utf8"))["transform"]
        xyz=f[list("xyz")].to_numpy()@np.asarray(tf["R_GC"]).T+np.asarray(tf["translation_m"])
        return f,xyz
    def plane(xyz):
        X=np.c_[xyz[:,:2],np.ones(len(xyz))]
        fit=np.linalg.lstsq(X,xyz[:,2],rcond=None)[0]; dz=xyz[:,2]-X@fit
        return {"n":len(xyz),"map_z_range_m":np.ptp(xyz[:,2]),"fit_z_ax_by_c":fit,"plane_residual_rms_m":np.sqrt(np.mean(dz*dz))}
    _,px=positions(raw);s2,p2=positions(second)
    rot=Rotation.from_quat(li[["qx","qy","qz","qw"]].to_numpy())
    geometry={}; centers={}
    plane_df=read(raw/"diagnostics","plane_constraints.csv"); L=plane_df[plane_df["class"]=="all"]
    assert np.max(abs(L.cut_s.to_numpy()-li.stamp_ns.to_numpy()*1e-9))<1e-6
    for part,mask in zones.items():
        if part=="all": continue
        pts=px[mask]; center=np.median(pts,axis=0);centers[part]=center
        stamp=li.stamp_ns.to_numpy()[mask]; maps={}
        pl_time=plane_df.cut_s.to_numpy()
        for k,g in plane_df[(pl_time>=stamp[0]*1e-9-1e-6)&(pl_time<=stamp[-1]*1e-9+1e-6)].groupby("class"):
            maps[k]={"count":int(g["count"].sum()),"mae_mm":1000*g.abs_sum_m.sum()/g["count"].sum(),"rmse_mm":1000*np.sqrt(g.sq_sum_m.sum()/g["count"].sum())}
        geometry[part]={"n":len(pts),"xyz_median_m":center,"z_span_m":np.ptp(pts[:,2]),"z_std_m":np.std(pts[:,2]),
            "plane_matches_median":np.median(L["count"].to_numpy()[mask]),"plane_fit":maps}
    dif=centers["end_2059_2079"]-centers["start_5_25"]
    geometry["static_centroid_delta_map_m"]=dif
    lru2=read(second,"Log/visual_lru.csv"); res2=read(second,"resources.csv")
    second_archive=json.loads((second/"offline.json").read_text(encoding="utf8"))
    peak=float(res2.rss_kib.max())
    summary={"local_map":lru,"console_frame_entries":len(trace),"console_lru_overlap":len(overlap),"distinct_measured_frames":len(trace.keys()|same.keys()),
        "console_bounds":sorted(trace)[::max(1,len(trace)-1)],"third_geom":plane(px),"second_geom":plane(p2),"end_geometry":geometry,
        "second_resources":{"resource_samples":len(res2),"resource_csv_peak_rss_mib":peak/1024,"offline_report_peak_rss_mib":second_archive["peak_rss_kib"]/1024,"peak_pss_mib":res2.pss_kib.max()/1024,
            "elapsed_s":second_archive["elapsed_s"],"cache_samples":len(lru2),"cache_peak_mib":lru2.image_bytes.max()/2**20,
            "visual_rss_peak_mib":lru2.rss_bytes.max()/2**20,"used_points":q(lru2.used_points),"last_image_count":int(lru2.images.iloc[-1])},
        "third_last_cache_frame":int(lc.frame.iloc[-1]),
        "source_sha256":{n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in ("src/vio.cpp","src/LIVMapper.cpp","src/keyframe_export.cpp","include/common_lib.h")},
        "second_input_sha256":{n:hashlib.sha256((second/n).read_bytes()).hexdigest() for n in ("Log/visual_lru.csv","Log/processing.csv","resources.csv","gravity_transform.json","offline.json","keyframes/states.csv")}}
    write_json(out/"visual_uncertainty.json",summary)
if __name__=="__main__":
    p=argparse.ArgumentParser()
    for k in ("raw","second","source","out"):p.add_argument("--"+k,required=True,type=Path)
    a=p.parse_args();main(a.raw,a.second,a.source,a.out)
