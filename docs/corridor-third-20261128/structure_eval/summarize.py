"""Summarize per-plane, per-block and direction checks from audit_surfaces.py."""
import argparse
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def distribution(v):
    v=np.asarray(v,dtype=float)
    return dict(n=len(v),min=float(v.min()),p50=float(np.median(v)),p90=float(np.quantile(v,.9)),
                p95=float(np.quantile(v,.95)),max=float(v.max()),rms=float(np.sqrt(np.mean(v**2))))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("observations",type=Path);p.add_argument("map_root",type=Path);p.add_argument("out",type=Path)
    args=p.parse_args()
    raw=json.loads(args.observations.read_text())
    out=args.out
    plane=pd.DataFrame(raw["planes"])
    block=pd.DataFrame(raw["blocks"],columns=raw["block_columns"])
    direction=pd.DataFrame(raw["directions"])
    summary=json.loads((args.map_root/"structure/summary.json").read_text())
    err=["e1_mm","e2_mm","e3_mm"]
    block["rms_mm"]=np.sqrt((block[err]**2).mean(axis=1))
    held=block[block.split=="held"]
    rms=held.groupby(["plane","stage"]).rms_mm.agg(lambda x:np.sqrt(np.mean(x**2))).unstack()
    plane=plane.join(rms,on="id")
    plane["change_BC_mm"]=plane.C-plane.B
    associations=json.loads((args.map_root/"structure/shared_plane_associations.json").read_text(encoding="utf8"))
    defs_path=Path(__file__).resolve().parent.parent/"geometry/geometry_metrics.json"
    defs=json.loads(defs_path.read_text(encoding="utf8"))["definitions"]
    track=np.loadtxt(args.map_root/"pgo/trajectory_after.tum")
    for j,assoc in enumerate(associations):
        ids=np.array(assoc["training_ids"])
        d=track[np.minimum(ids+2,len(track)-1),1:3]-track[np.maximum(ids-2,0),1:3]
        d=d[np.linalg.norm(d,axis=1)>.6]
        d/=np.linalg.norm(d,axis=1,keepdims=True)
        plane.loc[j,"train_span_s"]=np.ptp(track[ids,0])
        plane.loc[j,"train_cross_heading"]=len(d)>1 and np.min(np.abs(d@d.T))<np.cos(np.deg2rad(30))
        visited=[key for key,seg in defs.items() if any(np.any((ids>=lo)&(ids<=hi)) for lo,hi in seg["intervals"])]
        plane.loc[j,"train_segments"]="|".join(visited)
        plane.loc[j,"train_segment_count"]=len(visited)
    stats={}
    for surface in ["all","floor","ceiling","wall"]:
        planes=plane if surface=="all" else plane[plane.surface==surface]
        blocks=block if surface=="all" else block[block.surface==surface]
        record=dict(planes=len(planes),multi_visit=int((planes.span_s>75).sum()),cross_heading=int(planes.cross_heading.sum()),
                    both=int(((planes.span_s>75)&planes.cross_heading).sum()),held_improved=int((planes.change_BC_mm<0).sum()),
                    train_multi_visit=int((planes.train_span_s>75).sum()),train_cross_heading=int(np.count_nonzero(planes.train_cross_heading.to_numpy(dtype=bool))),
                    train_multiple_named_segments=int((planes.train_segment_count>1).sum()),
                    delta_held_rms_C_minus_B_mm=distribution(planes.change_BC_mm),normal_change_deg=distribution(planes.normal_change_deg),
                    anchor_offset_abs_mm=distribution(abs(planes.offset_m)*1000),error={})
        for (stage,split),rows in blocks.groupby(["stage","split"]):
            vals=np.abs(rows[err].to_numpy().ravel())
            record["error"].setdefault(split,{})[stage]=dict(points_abs_mm=distribution(vals),block_rms_mm=distribution(rows.rms_mm))
        stats[surface]=record
    for stage,side in (("B","before"),("C","after")):
        assert abs(stats["all"]["error"]["held"][stage]["points_abs_mm"]["rms"]-summary[f"held_plane_rms_{side}_m"]*1000)<.00001
    assert stats["all"]["multi_visit"]==summary["multi_visit_planes"]
    assert stats["all"]["cross_heading"]==summary["cross_heading_planes"]
    directions={}
    for kind,part in direction.groupby("kind"):
        directions[kind]={s:distribution(part[s+"_deg"]) for s in "ABC"}
    motion=direction[direction.kind=="velocity_balance"]
    force=dict(norm_m_s2=distribution(motion.force_norm),endpoint_velocity_difference_m_s=distribution(motion.delta_v_norm),
               first_mid_s=min(motion.t),last_mid_s=max(motion.t),unique_frames=int(direction.frame.nunique()),
               sampled_duration_s=len(motion)*8)
    plane.to_csv(out/"planes.csv",index=False,float_format="%.9g")
    direction.to_csv(out/"directions.csv",index=False,float_format="%.9g")
    block.to_csv(out/"blocks.csv",index=False,float_format="%.9g")
    plt.rcParams.update({"font.family":"Microsoft YaHei","axes.unicode_minus":False})
    fig,axs=plt.subplots(1,3,figsize=(13,6.5),layout="constrained")
    for typ,color,lab in (("floor","#aa6137","地"),("wall","#4f7394","墙"),("ceiling","#238076","顶")):
        pp=plane[plane.surface==typ]
        axs[0].scatter(pp.x,pp.y,s=13,c=color,alpha=.8,label=lab)
    axs[0].legend(loc="best",ncol=3,fontsize=8);axs[0].set_title("268 个平面支点")
    limit=np.percentile(abs(plane.change_BC_mm),95)
    im=axs[1].scatter(plane.x,plane.y,c=plane.change_BC_mm,s=19,cmap="RdBu_r",vmin=-limit,vmax=limit)
    fig.colorbar(im,ax=axs[1],shrink=.6,label="C − B / mm",extend="both")
    axs[1].set_title("各面留出 RMS 变化")
    im=axs[2].scatter(direction.x,direction.y,c=direction.t,cmap="viridis",s=18)
    fig.colorbar(im,ax=axs[2],shrink=.6,label="观测中点 / s")
    axs[2].set_title("惯性方向投影位置")
    for ax in axs: ax.set(aspect="equal",xlabel="X / m",ylabel="Y / m");ax.grid(alpha=.14)
    fig.savefig(out/"associations.png",dpi=165);plt.close(fig)
    fig,(ax,ecdf)=plt.subplots(2,1,figsize=(11,7),layout="constrained")
    for stage,col in zip("ABC",["#a75633","#217ba3","#208466"]):
        ordered=direction.sort_values("t")
        ax.plot(ordered.t,ordered[stage+"_deg"],color=col,lw=1,label=stage)
        static=ordered[ordered.kind=="stationary"]
        ax.scatter(static.t,static[stage+"_deg"],s=32,c=col,marker="s")
        e=np.sort(held[held.stage==stage].rms_mm);ecdf.plot(e,np.arange(1,len(e)+1)/len(e),color=col,label=stage)
    ax.set(xlabel="Sensor 相对时间 / s",ylabel="方向与地图竖直夹角 / °",title="实际入图方向残差（方点为两个静止观测）")
    ecdf.set(xlabel="留出块三点 RMS / mm",ylabel="累积比例",title="同一组 409 个留出块，每条轨迹仅用训练块重新拟面",xscale="symlog")
    for a in (ax,ecdf):a.grid(alpha=.19);a.legend()
    fig.savefig(out/"residuals.png",dpi=165);plt.close(fig)
    data=dict(plane_stats=stats,direction_residual_deg=directions,motion=force,
              graph={k:summary[k] for k in ("graph_initial","graph_final","iterations","optimization_and_association_s","rebuild_s","wall_clock_s","peak_rss_MiB")})
    (out/"metrics.json").write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
