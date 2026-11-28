"""Third-corridor fixed-return and reselected-cell evaluation. Run from any directory."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import itertools
import gc

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent / "loops"))
from fetch_inputs import checkout_paths
ALGORITHM_ROOT, DEFAULT_DATA = checkout_paths(OUT)
sys.path.insert(0, str(ALGORITHM_ROOT / "tools"))
from evaluate_z_geometry import binary_map, read_spatial_sample, horizontal, plt

STAGES = "ABC"
NAMES = dict(A="前端", B="仅回环", C="结构与惯性")
COLORS = dict(A="#7a7777", B="#ce782b", C="#007c85")
# Only A XY and its traversal order were used to choose straight runs.
# Fit a principal horizontal axis to these inclusive keyframe intervals. Clip its
# length to the projections of the ordered endpoint frames; Z is never used.
SEG = {
    "main": ("主干走廊", [(163, 325), (644, 833)], (833, 323)),
    "upper": ("上部斜廊", [(382, 478), (500, 600)], (382, 478)),
    "lower_w": ("下部西斜廊", [(847, 909)], (847, 909)),
    "lower_s": ("下部南侧连接廊", [(938, 971)], (938, 971)),
    "lower_e": ("下部东斜廊", [(1004, 1035)], (1004, 1035)),
    "lower_mid": ("下部中央横廊", [(1056, 1071), (1130, 1147)], (1056, 1071)),
    "lower_n": ("下部北侧返程廊", [(1154, 1181)], (1154, 1181)),
}
# Piecewise XY paths follow actual third-run A keyframes. Grouped IDs on the
# top bend select one pass through the junction and skip repeated inspection turns.
CORNERS = {
    "top": ("主干转上斜廊", [305, 319, 327, 331, 342, 380, 390, 405], 342),
    "lower_entry": ("主干转下部西廊", [811, 822, 833, 840, 848, 858, 869], 840),
    "lower_tip": ("下部西廊转南侧", [886, 898, 908, 912, 939, 950, 963, 970], 912),
    "lower_east": ("南侧转下部东廊", [943, 954, 964, 974, 981, 1006, 1016, 1026], 974),
    "lower_middle": ("东廊转中央横廊", [1011, 1022, 1030, 1036, 1041, 1048, 1051], 1036),
}
SURF = {"floor": "地面", "ceiling": "顶面"}
RMS = lambda e: float(np.sqrt(np.mean(np.square(e))))
Q = lambda a, p: float(np.quantile(a, p))


def sha(path):
    with path.open("rb") as fp:
        return hashlib.file_digest(fp, "sha256").hexdigest()


def native_pairs(points, tree, maps):
    distances, rows = tree.query(points, k=2, workers=4)
    if np.max(distances[:, 0]) > 1e-6:
        raise ValueError("5 cm point not found in native map")
    ids = rows[:, 0]
    keep = np.ones(len(ids), bool)
    # Check every candidate, including ties of multiplicity >2. Discard exact
    # duplicates if a later stage placed them differently.
    for pos in np.flatnonzero(distances[:, 1] <= 1e-7):
        ties = tree.query_ball_point(points[pos], 1e-7)
        if any(np.max(np.linalg.norm(m[ties, :3] - m[ids[pos], :3], axis=1)) > 1e-4 for m in maps.values()):
            keep[pos] = False
    return ids[keep], {"queried": len(ids), "retained": int(keep.sum()),
                       "ambiguous_removed": int((~keep).sum()), "max_match_m": float(distances[:, 0].max())}


def cell_table(q, xy):
    df = pd.DataFrame(q, columns=list("xyz"))
    df["i"], df["j"] = np.floor(xy).astype(int).T
    g = df.groupby(["i", "j"])
    table = g.median()
    table["n"] = g.size()
    table["p90_p10_m"] = g.z.quantile(.9) - g.z.quantile(.1)
    return table[table.n >= 4].copy()


def cell_stats(tables, cohort, out):
    common = tables["A"].index.intersection(tables["B"].index).intersection(tables["C"].index)
    summary, res, zs = {}, {}, {}
    for k, table in tables.items():
        t = table.loc[common].copy()
        X = np.c_[t.x, t.y, np.ones(len(t))]
        coef = np.linalg.lstsq(X, t.z, rcond=None)[0]
        r = np.asarray(t.z - X @ coef)
        t["residual_z_m"] = r
        t["stage"], t["cohort"] = k, cohort
        out.append(t.reset_index())
        res[k], zs[k] = r, t.z
        summary[k] = dict(eligible=len(table), cells=len(t), points_in_cells=int(t.n.sum()),
                          min_m=t.z.min(), max_m=t.z.max(), range_m=np.ptp(t.z), p95_minus_p05_m=Q(t.z,.95)-Q(t.z,.05),
                          plane_abc=coef.tolist(), tilt_deg=np.degrees(np.arctan(np.linalg.norm(coef[:2]))),
                          residual_rms_m=RMS(r), residual_abs_p95_m=Q(abs(r),.95),
                          residual_p90_p10_m=Q(r,.9)-Q(r,.1), spatial_z_p90_p10_m=Q(t.z,.9)-Q(t.z,.1),
                          thickness_median_m=Q(t.p90_p10_m,.5), thickness_p95_m=Q(t.p90_p10_m,.95))
    for a,b in itertools.combinations(STAGES,2):
        summary[a+b] = dict(z_correlation=float(np.corrcoef(zs[a],zs[b])[0,1]),
                            residual_correlation=float(np.corrcoef(res[a],res[b])[0,1]))
    return summary


def segdefs(tr):
    defs = {}
    for key,(name,intervals,endids) in SEG.items():
        points=np.concatenate([tr[i:j+1,:2] for i,j in intervals])
        center=points.mean(0)
        axis=np.linalg.svd(points-center,full_matrices=False)[2][0]
        d=(tr[list(endids),:2]-center)@axis
        start,end=center+d[0]*axis,center+d[1]*axis
        defs[key]={"name":name, "intervals":intervals, "extent_frames":endids,
                   "start_xy":start.tolist(), "end_xy":end.tolist(), "length_m":float(np.linalg.norm(end-start))}
    return defs


def fit_profile(s,z):
    coef=np.polyfit(s,z,1)
    r=z-np.polyval(coef,s)
    return dict(slope_deg=np.degrees(np.arctan(coef[0])), delta_m=float(z[-1]-z[0]),
                rms_m=RMS(r), absres_p95_m=Q(abs(r),.95), n_bins=len(z), first_s=float(s[0]),last_s=float(s[-1]))


def normal_xyz(df):
    groups=df.groupby(["sbin","cbin"])
    med=groups[["x","y","z"]].median()[groups.size()>=3]
    design=np.c_[med.x,med.y,np.ones(len(med))]
    coef=least_squares(lambda c:design@c-med.z,np.linalg.lstsq(design,med.z,rcond=None)[0],loss="soft_l1",f_scale=.03).x
    n=np.r_[-coef[:2],1.]
    return n/np.linalg.norm(n)


def angle(n,m):
    return float(np.degrees(np.arccos(np.clip(np.dot(n,m),-1,1))))


def paired_profiles(maps,ids,defs):
    stats, profiles, normals = {}, [], {}
    for key,defn in defs.items():
        start,end=np.array(defn["start_xy"]),np.array(defn["end_xy"])
        axis=(end-start)/defn["length_m"]
        normals[key]={}
        stats[key]={}
        for surf,rows in ids.items():
            base=maps["A"][rows,:2]-start
            s=base@axis
            cross=base@np.array([-axis[1],axis[0]])
            use=(s>=0)&(s<defn["length_m"])&(abs(cross)<1.8)
            normals[key][surf]={}
            stats[key][surf]={}
            for k in STAGES:
                points=maps[k][rows[use],:3]
                df=pd.DataFrame(points,columns=list("xyz"))
                df["s"],df["sbin"],df["cbin"]=s[use],np.floor(s[use]).astype(int),np.floor(2*cross[use]).astype(int)
                groups=df.groupby("sbin")
                med=groups[["s","z"]].median()
                med["count"]=groups.size()
                med=med[med["count"]>=8].reset_index()
                ss,zz=med.s.to_numpy(),med.z.to_numpy()
                st=fit_profile(ss,zz)
                st["thirds"]=[]
                for i in range(3):
                    mask=(ss>=i*defn["length_m"]/3)&(ss<(i+1)*defn["length_m"]/3)
                    st["thirds"].append(fit_profile(ss[mask],zz[mask]) if mask.sum()>=3 else None)
                normals[key][surf][k]=normal_xyz(df)
                stats[key][surf][k]=st
                med["stage"],med["surface"],med["segment"]=k,surf,key
                profiles.append(med)
    full=pd.concat(profiles)
    normalrows=[]
    for k in STAGES:
        for key in defs:
            f,c=normals[key]["floor"][k],normals[key]["ceiling"][k]
            ss=full[(full.segment==key)&(full.stage==k)].pivot(index="sbin",columns="surface",values="z").dropna()
            h=ss.ceiling-ss.floor
            stats[key].setdefault("parallelism",{})[k]={"angle_floor_ceiling_deg":angle(f,c),
                "angle_floor_main_floor_deg":angle(f,normals["main"]["floor"][k]),
                "floor_n":f.tolist(),"ceiling_n":c.tolist(), "clearance_bins":len(h),
                "clearance_median_m":Q(h,.5), "clearance_p05_m":Q(h,.05),"clearance_p95_m":Q(h,.95),
                "clearance_p95_minus_p05_m":Q(h,.95)-Q(h,.05),"clearance_end_minus_start_m":float(h.iloc[-1]-h.iloc[0])}
        for surf in SURF:
            for a,b in itertools.combinations(defs,2):
                normalrows.append(dict(stage=k,surface=surf,segment_a=a,segment_b=b,
                                       angle_deg=angle(normals[a][surf][k],normals[b][surf][k])))
    return stats,full,pd.DataFrame(normalrows)


def independent_profile_bins(surfaces, defs, reference, stage):
    """Stage-specific returns and XY axis; project to equivalent distance on A."""
    rows=[]
    for key,definition in defs.items():
        start,end=np.array(definition["start_xy"]),np.array(definition["end_xy"])
        length=definition["length_m"]
        axis=(end-start)/length
        for surf,points in surfaces.items():
            delta=points[:,:2]-start
            s=delta@axis
            use=(s>=0)&(s<length)&(abs(delta@np.array([-axis[1],axis[0]]))<1.8)
            df=pd.DataFrame({"s_own":s[use],"s":s[use]*reference[key]["length_m"]/length,"z":points[use,2]})
            groups=df.groupby(np.floor(df.s).astype(int).rename("sbin"))
            med=groups.median()
            med["count"]=groups.size()
            med=med[med["count"]>=8].reset_index()
            med["segment"],med["surface"],med["stage"]=key,surf,stage
            rows.append(med)
    return pd.concat(rows)


def independent_profile_stats(bins):
    result={}
    for (key,surf),t in bins.groupby(["segment","surface"]):
        groups={k:g.set_index("sbin") for k,g in t.groupby("stage")}
        common=groups["A"].index.intersection(groups["B"].index).intersection(groups["C"].index)
        result.setdefault(key,{})[surf]={}
        for k,g in groups.items():
            selected=g.loc[common].sort_index()
            info=fit_profile(selected.s_own.to_numpy(),selected.z.to_numpy())
            info.update(own_bins=len(g),common_bins=len(selected))
            result[key][surf][k]=info
    return result


def projected_path(xy,path):
    arclength=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    dist=np.full(len(xy),np.inf)
    s=np.zeros(len(xy))
    for i,(p,q) in enumerate(zip(path,path[1:])):
        axis=(q-p)/np.linalg.norm(q-p)
        loc=np.clip((xy-p)@axis,0,arclength[i+1]-arclength[i])
        d=np.linalg.norm(xy-p-loc[:,None]*axis,axis=1)
        update=d<dist
        s[update]=arclength[i]+loc[update]
        dist[update]=d[update]
    return s,dist,arclength


def corner_profiles(maps,ids,tr):
    rows,summary=[],{}
    for key,(name,keyframes,origin) in CORNERS.items():
        path=tr[keyframes,:2]
        summary[key]=dict(name=name,keyframes=keyframes,origin_frame=origin,origin_xy=tr[origin,:2].tolist())
        for surf,idx in ids.items():
            summary[key][surf]={}
            s,d,arc=projected_path(maps["A"][idx,:2],path)
            signed=s-arc[keyframes.index(origin)]
            keep=(d<1.6)&(s>.5)&(s<arc[-1]-.5)&(signed>=-10)&(signed<10)
            for k in STAGES:
                pts=maps[k][idx[keep],:3]
                df=pd.DataFrame({"s":signed[keep],"z":pts[:,2]})
                groups=df.groupby(np.floor(df.s).astype(int).rename("sbin"))
                med=groups.median()
                med["count"]=groups.size()
                med=med[med["count"]>=8].reset_index()
                a,b=med[med.sbin.between(-8,-3)],med[med.sbin.between(2,7)]
                info=dict(before_fit_bins=len(a),after_fit_bins=len(b), measured_mid_bins=int(med.sbin.isin([-1,0]).sum()))
                if min(len(a),len(b))>=3:
                    ca,cb=np.polyfit(a.s,a.z,1),np.polyfit(b.s,b.z,1)
                    info.update(extrapolated_signed_gap_m=float(cb[1]-ca[1]),before_slope_deg=np.degrees(np.arctan(ca[0])),
                                after_slope_deg=np.degrees(np.arctan(cb[0])), before_fit_rms_m=RMS(a.z-np.polyval(ca,a.s)),
                                after_fit_rms_m=RMS(b.z-np.polyval(cb,b.s)))
                central=med.set_index("sbin").z
                if {-1,0}.issubset(central.index):
                    info["adjacent_signed_gap_m"]=float(central[0]-central[-1])
                changes=np.abs(np.diff(med.z))[np.diff(med.sbin)==1]
                info["adjacent_abs_p95_m"]=Q(changes,.95) if len(changes) else None
                summary[key][surf][k]=info
                med["stage"],med["surface"],med["corner"]=k,surf,key
                rows.append(med)
    return summary,pd.concat(rows)


def figs(out,grids,defs,prof,tr,corners):
    from matplotlib.colors import Normalize, TwoSlopeNorm
    from matplotlib.cm import ScalarMappable
    import matplotlib as mpl
    fig,axes=plt.subplots(1,3,figsize=(10,7.8),sharex=True,sharey=True,layout="constrained")
    base=grids[grids.cohort=="A_fixed"].copy()
    for field,title,cmap,filename in (("z","格内中位地面高度（m）","viridis","floor_height.png"),
                                     ("residual_z_m","地面相对各自拟合平面的垂直残差（m）","RdBu_r","floor_residual.png")):
        low,hi=base[field].min(),base[field].max()
        scale=Normalize(low,hi) if field=="z" else TwoSlopeNorm(0, -max(abs(low),abs(hi)),max(abs(low),abs(hi)))
        for ax,k in zip(axes,STAGES):
            ax.clear()
            f=base[base.stage==k]
            # Original grid centers preserve identical spatial support between panels.
            ax.scatter(f.i+.5,f.j+.5,c=f[field],s=7,marker="s",norm=scale,cmap=cmap,edgecolors="none")
            ax.set(aspect="equal",title=k+" · "+NAMES[k],xlabel="A 冻结格 X（m）")
            ax.grid(alpha=.15)
        axes[0].set_ylabel("A 冻结格 Y（m）")
        cb=fig.colorbar(ScalarMappable(norm=scale,cmap=cmap),ax=axes,fraction=.035,pad=.025,label=title)
        fig.savefig(out/filename,dpi=155)
        cb.remove()
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(6.5,8),layout="constrained")
    ax.plot(tr[:,0],tr[:,1],c="#bdbdbd",lw=1,label="A 关键帧轨迹")
    for i,(key,spec) in enumerate(defs.items()):
        ends=np.array([spec["start_xy"],spec["end_xy"]])
        ax.plot(*ends.T,lw=2.5,color=mpl.colormaps["tab10"](i),label=spec["name"])
    for i,(key,(_,frames,origin)) in enumerate(CORNERS.items(),1):
        ax.scatter(*tr[origin,:2],marker="x",c="#212121",s=32)
        ax.annotate(str(i),tr[origin,:2],xytext=(4,4),textcoords="offset points",fontsize=9)
    ax.set(aspect="equal",xlabel="A 地图 X（m）",ylabel="A 地图 Y（m）",title="第三轮直段范围和五处转角位置")
    ax.legend(loc="upper right",fontsize=8)
    ax.grid(alpha=.2)
    fig.savefig(out/"segments_xy.png",dpi=155)
    plt.close(fig)
    def plot_grid(table,specs,col,filename,xlabel):
        fig,axs=plt.subplots(len(specs),2,figsize=(10.5,2.1*len(specs)),layout="constrained",squeeze=False)
        for i,key in enumerate(specs):
            for j,(surf,label) in enumerate(SURF.items()):
                ax=axs[i,j]
                for k in STAGES:
                    f=table[(table[col]==key)&(table.surface==surf)&(table.stage==k)].set_index("sbin")
                    # Explicit missing rows create gaps in plotted profiles.
                    f=f.reindex(range(f.index.min(),f.index.max()+1))
                    ax.plot(f.s,f.z,lw=1.6,c=COLORS[k],label=k+" "+NAMES[k])
                ax.set(title=specs[key][0]+" · "+label,ylabel="Z（m）")
                if col=="corner": ax.axvline(0,color="#888",lw=.7,ls=":")
                if i==len(specs)-1: ax.set_xlabel(xlabel)
                ax.grid(alpha=.2)
        axs[0,0].legend(fontsize=8,ncol=3)
        fig.savefig(out/filename,dpi=155)
        plt.close(fig)
    plot_grid(prof,SEG,"segment","long_profiles.png","A 固定断面沿程距离（m）")
    plot_grid(corners,CORNERS,"corner","corner_profiles.png","距 A 冻结转角中心的有符号里程（m）")


def main(data,out):
    out.mkdir(exist_ok=True,parents=True)
    native_path=dict(A=data/"pgo/map_before.pcd",B=data/"pgo/map_after.pcd",C=data/"structure/map_after.pcd")
    trajpath=dict(A=data/"pgo/trajectory_before.tum",B=data/"pgo/trajectory_after.tum",C=data/"structure/trajectory_after.tum")
    maps={k:binary_map(p)[0] for k,p in native_path.items()}
    hashes={k:sha(p) for k,p in native_path.items()}
    if sha(data/"structure/map_before.pcd")!=hashes["A"]:
        raise ValueError("native sources differ")
    tr={k:np.loadtxt(p)[:,1:4] for k,p in trajpath.items()}
    for k in STAGES:
        if maps[k].shape!=maps["A"].shape or not np.array_equal(maps[k][:,3],maps["A"][:,3]):
            raise ValueError("native rows do not match")
    grids=[]
    result=dict(native_points=len(maps["A"]),native_sha256=hashes, floor_samples={}, geometry_sample_sha256={},
                native_tracking={}, cohorts={})
    independent={}
    ids_A={}
    defs=segdefs(tr["A"])
    independent_bins=[]
    for k in STAGES:
        print("surface selection",k,flush=True)
        p=data/"spatial_5cm"/(k+"_geometry.pcd")
        result["geometry_sample_sha256"][k]=sha(p)
        sample,_=read_spatial_sample(p)
        floor,_=horizontal(sample,tr[k],20,2.5)
        surfaces=dict(zip(SURF,horizontal(sample,tr[k],8,3.0)))
        independent_bins.append(independent_profile_bins(surfaces,segdefs(tr[k]),defs,k))
        independent[k]=cell_table(floor,floor[:,:2])
        result["floor_samples"][k]={"spatial_sample_points":len(sample), "floor_k20":len(floor),"supported_cells":len(independent[k])}
        if k in "AB":
            tree=cKDTree(maps[k][:,:3])
            idx,rep=native_pairs(floor,tree,maps)
            result["native_tracking"][k+"_floor"]=rep
            stats=cell_stats({stage:cell_table(m[idx,:3],maps[k][idx,:2]) for stage,m in maps.items()},k+"_fixed",grids)
            result["cohorts"][k+"_fixed"]=stats
            if k=="A":
                for surf,pts in surfaces.items():
                    ids_A[surf],result["native_tracking"]["A_profile_"+surf]=native_pairs(pts,tree,maps)
            del tree
        del sample
        gc.collect()
    result["cohorts"]["independent_common"]=cell_stats(independent,"independent_common",grids)
    grids=pd.concat(grids,ignore_index=True)
    segstats,prof,normals=paired_profiles(maps,ids_A,defs)
    cornerstats,corners=corner_profiles(maps,ids_A,tr["A"])
    independent_bins=pd.concat(independent_bins)
    result.update(segments=segstats,definitions=defs,corners=cornerstats,
                  independent_segments=independent_profile_stats(independent_bins))
    for filename,t in (("floor_cells.csv",grids),("long_profiles.csv",prof),("segment_normal_angles.csv",normals),("corner_profiles.csv",corners),
                       ("independent_profiles.csv",independent_bins)):
        t.to_csv(out/filename,index=False,float_format="%.8f")
    (out/"geometry_metrics.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False,default=lambda x:x.item())+"\n",encoding="utf8")
    figs(out,grids,defs,prof,tr["A"],corners)
    print("complete",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data",type=Path,default=DEFAULT_DATA,help="Third-run directory containing pgo/, structure/ and spatial_5cm/")
    parser.add_argument("--out",type=Path,default=OUT)
    opts=parser.parse_args()
    main(opts.data,opts.out)
