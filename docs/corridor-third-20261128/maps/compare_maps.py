"""Read all native PCD rows, verify map semantics and render matched metric projections."""
from argparse import ArgumentParser
from pathlib import Path
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, LinearSegmentedColormap
import numpy as np
import pandas as pd


def main():
    parser=ArgumentParser(description=__doc__)
    parser.add_argument("data",type=Path)
    parser.add_argument("repo",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    sys.path.insert(0,str(args.repo/"tools"))
    from evaluate_z_geometry import binary_map, read_spatial_sample
    out=args.out
    out.mkdir(parents=True,exist_ok=True)
    roots=[(args.data/"pgo","before"),(args.data/"pgo","after"),(args.data/"structure","after")]
    labels=["A  前端","B  全程 PGO","C  联合优化"]
    tracks=[np.loadtxt(folder/f"trajectory_{v}.tum") for folder,v in roots]
    clouds=[binary_map(folder/f"map_{v}.pcd")[0] for folder,v in roots]
    rgb=[binary_map(folder/f"map_rgb_{v}.pcd")[0] for folder,v in roots]
    before_count={}
    for name,base in [("map",clouds[0]),("map_rgb",rgb[0])]:
        other=binary_map(args.data/f"structure/{name}_before.pcd")[0]
        before_count[name]=int(np.count_nonzero(np.any(other.view('u4')!=base.view('u4'),axis=1)))
    assert not any(before_count.values())
    rgb_bits=rgb[0][:,3].view('u4')
    parts=np.array([(rgb_bits>>shift)&255 for shift in (16,8,0)])
    scalar_mismatch={key:[int(np.count_nonzero(d[0][:,3].view('u4')!=s[:,3].view('u4'))) for s in d]
                     for key,d in (("intensity",clouds),("packed_rgb",rgb))}
    assert not any(sum(x) for x in scalar_mismatch.values())
    report=dict(structure_before_row_mismatches=before_count,scalar_mismatches_ABC=scalar_mismatch,
        rgb_all_channels_equal=int(np.count_nonzero(np.ptp(parts,axis=0)==0)),
        geometry_points=len(clouds[0]),rgb_points=len(rgb_bits),stages={})
    for stage,cloud,tr in zip("ABC",clouds,tracks):
        report["stages"][stage]=dict(min_xyz_m=cloud[:,:3].min(0).tolist(),max_xyz_m=cloud[:,:3].max(0).tolist(),
            path_m=float(np.linalg.norm(np.diff(tr[:,1:4],axis=0),axis=1).sum()))
        for kind in ("geometry","rgb"):
            path=args.data/f"spatial_5cm/{stage}_{kind}.pcd"
            pts,fields=read_spatial_sample(path)
            report['stages'][stage][kind+'_5cm_fields']=fields
            report["stages"][stage][kind+"_5cm_points"]=len(pts)
            assert len(pts)>0 and np.isfinite(pts).all()
    cmap=LinearSegmentedColormap.from_list("ink",["#d6e9ef","#5298ad","#16465e","#091c2e"])
    cmap.set_bad("white")
    def projection(name,axes,step,limits=None,select=None):
        if limits is None:
            limits=[(float(np.floor(min(m[:,ax].min() for m in clouds)))-1,
                     float(np.ceil(max(m[:,ax].max() for m in clouds)))+1) for ax in axes]
        bins=[np.linspace(lo,hi,int(np.ceil((hi-lo)/step))+1) for lo,hi in limits]
        counts=[]
        for cloud in clouds:
            p=cloud if select is None else cloud[select(cloud)]
            h=np.histogram2d(p[:,axes[0]],p[:,axes[1]],bins=bins)[0].T
            if select is None: assert h.sum()==len(cloud)
            counts.append(h)
        vmax=max(float(np.percentile(h[h>0],99.5)) for h in counts)
        side=(axes==[0,1] or name=="cross_section")
        fig,axs=plt.subplots(1 if side else 3,3 if side else 1,figsize=(12,7.2) if side else (13,6.8),layout="constrained")
        for label,h,ax in zip(labels,counts,axs):
            im=ax.imshow(np.ma.masked_equal(h,0),extent=[*limits[0],*limits[1]],origin="lower",
                         norm=LogNorm(vmin=1,vmax=vmax),interpolation="nearest",cmap=cmap,aspect="equal")
            ax.set_title(label,loc="left",fontsize=11,pad=3)
            ax.set_xlabel("XYZ"[axes[0]]+" / m");ax.set_ylabel("XYZ"[axes[1]]+" / m")
        fig.colorbar(im,ax=axs,shrink=.68,pad=.01,label="每投影格回波数（共用对数色标）")
        fig.savefig(out/f"{name}.png",dpi=165,bbox_inches="tight")
        plt.close(fig)
        return dict(axes=axes,limits=limits,bins=[len(x)-1 for x in bins],display_max=vmax,counts=[int(h.sum()) for h in counts])
    report["projection"]={}
    for a in ([0,1],[0,2],[1,2]):
        name="".join("XYZ"[n] for n in a)
        report["projection"][name]=projection(name,a,.08)
    report["projection"]["junction"]=projection("junction",[0,1],.025,[(-6,16),(-11,15)],
                                                 lambda c:(c[:,2]>.20)&(c[:,2]<1.25))
    report["projection"]["cross_section"]=projection("cross_section",[0,2],.025,[(4,15),(-3,5)],
                                                      lambda c:np.abs(c[:,1]-40)<.45)
    colors=["#b24e33","#167faf","#26805b"]
    fig,(ax,az)=plt.subplots(1,2,figsize=(11.8,5.4),gridspec_kw={"width_ratios":[1,1.65]},layout="constrained")
    frame=pd.DataFrame({"frame":np.arange(len(tracks[0])),"sensor_s":tracks[0][:,0]-1590192670.200220672})
    for s,lab,tr,col in zip("ABC",labels,tracks,colors):
        for k,dim in enumerate("xyz",1): frame[f"{s}_{dim}_m"]=tr[:,k]
        ax.plot(tr[:,1],tr[:,2],color=col,linewidth=.85,label=lab)
        az.plot(frame.sensor_s,tr[:,3],color=col,linewidth=1.1,label=lab)
    ax.axis("equal");ax.set(xlabel="X / m",ylabel="Y / m",title="关键帧平面位置")
    az.set(xlabel="相对首个 LIO 状态 / s",ylabel="Z / m",title="同一批关键帧的重力系高度")
    az.legend();az.grid(alpha=.18);ax.grid(alpha=.18)
    fig.savefig(out/"trajectory.png",dpi=165)
    plt.close(fig)
    frame.to_csv(out/"trajectories.csv",index=False,float_format="%.8f")
    (out/"metrics.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
