"""Render comparative evidence from three finished B1 trials on a plotting workstation."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--pgo",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({"font.sans-serif":["Microsoft YaHei","Noto Sans CJK SC","DejaVu Sans"],
                         "axes.unicode_minus":False,"axes.spines.top":False,"axes.spines.right":False,"font.size":10})
    groups=["b1_p0","b1_auto","b1_joint"]
    records={}
    def read(g,f):
        return json.loads((a.pgo/g/f).read_text())
    for g in groups:
        records[g]={"summary":read(g,"summary.json"),"holdout":read(g,"holdout.json"),
                    "sensitivity":read(g,"weight_sensitivity.json"),"verification":read(g,"verification.json")}
    rows=[]
    for g,r in records.items():
        for k,h in enumerate(r["holdout"],1):
            for stage,values in h["point_to_point"].items():
                rows.append({"mode":g,"region":k,"target_id":h["target_id"],"source_id":h["source_id"],"stage":stage,**values})
    with (a.output/"heldout_point_metrics.csv").open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    g="b1_joint"
    pose=np.genfromtxt(a.pgo/g/"poses.csv",delimiter=",",names=True)
    fig,axes=plt.subplots(2,2,figsize=(12.8,8.2),layout="constrained")
    ax=axes[0,0]
    ax.plot(pose["before_x_m"],pose["before_y_m"],color="#c76a3d",lw=1.2,label="原始 B1")
    ax.plot(pose["after_x_m"],pose["after_y_m"],color="#006d77",lw=1,label="联合图")
    for i,h in enumerate(records[g]["holdout"],1):
        x,y=h["target_center_map_gravity_m"][:2]
        ax.scatter(x,y,s=24,c="#364a79",zorder=5);ax.annotate(str(i),(x,y),xytext=(4,3),textcoords="offset points")
    ax.set(title="平面轨迹与五处冻结回访",xlabel="X / m",ylabel="Y / m",aspect="equal");ax.legend(frameon=False)
    ax=axes[0,1]
    ax.plot(pose["t_s"]/60,pose["before_z_m"],color="#c76a3d",lw=1,label="原始 B1")
    ax.plot(pose["t_s"]/60,pose["after_z_m"],color="#006d77",lw=1,label="联合图")
    ax.set(title="重力系高度随时间的变化",xlabel="时间 / min",ylabel="Z / m");ax.legend(frameon=False)
    ax=axes[1,0]
    idx=np.arange(1,6)
    for off,g,stage,name,color in [(-.28,"b1_joint","before","原始 B1","#6d7585"),
                                    (-.09,"b1_p0","after","仅 P0","#c76a3d"),
                                    (.09,"b1_auto","after","仅自动","#71a9a1"),
                                    (.28,"b1_joint","after","联合图","#006d77")]:
        ax.bar(idx+off,[h["point_to_point"][stage]["source_median_m"] for h in records[g]["holdout"]],
               width=.17,label=name,color=color)
    ax.set(title="新访次到旧访次的点距中位数",xlabel="冻结区域",ylabel="距离 / m",xticks=idx)
    ax.legend(frameon=False,ncol=2)
    ax=axes[1,1]
    h=read("b1_joint","holdout_validated.json")["regions"]
    ids=[(0,"wall_a_1"),(2,"wall_a_1"),(2,"wall_a_2"),(3,"wall_a_2"),(4,"wall_a_1"),(4,"wall_a_2")]
    labels=[]
    for i,(k,label) in enumerate(ids):
        s=h[k]["quality"]["surfaces"][label]
        measured=[s[stage]["published"] for stage in ("before","after")]
        if not all(measured):
            raise ValueError(f"selected plot surface has insufficient evidence: region {k+1}, {label}")
        ax.bar(i-.17,measured[0]["two_visit_thickness_p90_p10_m"],width=.31,color="#c76a3d",label="原始 B1" if i==0 else None)
        ax.bar(i+.17,measured[1]["two_visit_thickness_p90_p10_m"],width=.31,color="#006d77",label="联合图" if i==0 else None)
        labels.append(f"区{k+1}\n墙{label[-1]}")
    ax.set(title="同一墙面两访次厚度 P90−P10",ylabel="厚度 / m",xticks=range(len(ids)),xticklabels=labels)
    ax.legend(frameon=False)
    for ax in axes.flat:
        ax.grid(alpha=.17)
    fig.savefig(a.output/"轨迹与冻结回访几何.png",dpi=150);plt.close(fig)
    corner=a.output/"corner_review"
    if (corner/"corner_points.npz").exists():
        vals=np.load(corner/"corner_points.npz")
        fig,axs=plt.subplots(1,2,figsize=(11,5),layout="constrained")
        for ax,stage,title in zip(axs,("before","after"),("里程计","联合优化")):
            for key,c,label in [("target","#365586","旧访次"),(stage,"#d0774f","新访次")]:
                xyz=vals[key]
                ax.scatter(xyz[:,0],xyz[:,1],s=1.3,c=c,alpha=.4,label=label)
            for key in ("wall_a_1","wall_a_2","wall_b_1"):
                v=vals.get(stage+"_"+key)
                if v is not None:
                    ax.scatter(v[:,0],v[:,1],c="#007b63",s=3,label="同法向墙面点" if key=="wall_a_1" else None)
            ax.axis("equal");ax.set(xlabel="局部 X / m",ylabel="局部 Y / m",title=title);ax.legend(markerscale=4,frameon=False)
        fig.suptitle("区域 2 两访次对照（关键帧 499 / 1404）")
        fig.savefig(corner/"区域2墙面支撑.png",dpi=150);plt.close(fig)


if __name__=="__main__":
    main()
