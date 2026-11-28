"""Evaluate matched CloudCompare spatial samples against frozen corridor geometry."""
import argparse
import json
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

for font in ("Microsoft YaHei", "Noto Sans CJK SC", "SimHei"):
    if font in font_manager.get_font_names():
        plt.rcParams["font.family"]=font
        break
plt.rcParams["axes.unicode_minus"]=False

SEGMENT_NAMES={"Main corridor":"主干走廊","Upper diagonal":"上部斜支路",
               "Lower west":"下部左廊","Lower east":"下部右廊"}


def binary_map(path):
    with path.open("rb") as f:
        fields={}
        for line in f:
            words=line.decode("ascii").split()
            if words and not words[0].startswith("#"):
                fields[words[0]]=words[1:]
            if words[:1]==["DATA"]:
                break
        if fields["DATA"]!=["binary"] or fields["FIELDS"][:3]!=list("xyz") or fields["SIZE"]!=["4"]*4:
            raise ValueError("expected native xyzi or xyz/rgb binary PCD")
        return np.memmap(path,"<f4","r",offset=f.tell(),shape=(int(fields["POINTS"][0]),4)),fields["FIELDS"][-1]


def read_spatial_sample(path):
    """PCL binary_compressed stores each field contiguously after LZF decoding."""
    import struct
    with path.open("rb") as f:
        header={}
        for line in f:
            words=line.decode("ascii").split()
            if words and not words[0].startswith("#"):
                header[words[0]]=words[1:]
            if words[:1]==["DATA"]:
                break
        if header["DATA"]!=["binary_compressed"] or set(header["SIZE"])!={"4"} or set(header["COUNT"])!={"1"}:
            raise ValueError("expected CloudCompare binary_compressed 32-bit scalar fields")
        size,raw_size=struct.unpack("<II",f.read(8))
        data=f.read(size)
        if len(data)!=size or f.read(1):
            raise ValueError("PCD payload size mismatch")
    raw=bytearray()
    cursor=0
    while cursor<size:
        tag=data[cursor]
        cursor+=1
        if tag<32:
            length=tag+1
            raw.extend(data[cursor:cursor+length])
            cursor+=length
        else:
            length=tag>>5
            if length==7:
                length+=data[cursor]
                cursor+=1
            distance=((tag&31)<<8)+data[cursor]+1
            cursor+=1
            length+=2
            if distance>len(raw):
                raise ValueError("invalid LZF distance")
            start=len(raw)-distance
            run=raw[start:start+min(distance,length)]
            # LZF permits overlapping copies. Only up to 'length' bytes are needed.
            raw.extend((run*((length+distance-1)//distance))[:length])
    count=int(header["POINTS"][0])
    if len(raw)!=raw_size or raw_size!=count*4*len(header["FIELDS"]):
        raise ValueError("PCD uncompressed size mismatch")
    array=np.frombuffer(raw,"<f4").reshape(-1,count)
    return np.column_stack([array[header["FIELDS"].index(k)] for k in "xyz"]),header["FIELDS"]

def horizontal(a,traj,k,radius):
    sample=a[::3]
    distance,idx=cKDTree(traj[:,:2]).query(sample[:,:2],workers=4)
    h=sample[:,2]-traj[idx,2]
    use=(distance<radius)&(((h> -1.3)&(h<-.2))|((h>1.6)&(h<4.4)))
    points=sample[use]
    tree=cKDTree(a)
    accept=[]
    for start in range(0,len(points),25000):
        d,nn=tree.query(points[start:start+25000],k=k,workers=4)
        cloud=a[nn].copy()
        cloud-=cloud.mean(1,keepdims=True)
        val,axis=np.linalg.eigh(np.einsum("nki,nkj->nij",cloud,cloud))
        accept.extend((abs(axis[:,2,0])>np.cos(np.deg2rad(15))) &
                      (val[:,0]<.04*val.sum(1)) & (d[:,-1]<.5))
    good=np.array(accept)
    return points[good&(h[use]<-.2)],points[good&(h[use]>1.6)]


def grid(points):
    df=pd.DataFrame(points,columns=list("xyz"))
    groups=df.groupby([np.floor(df.x).astype(int).rename("i"),np.floor(df.y).astype(int).rename("j")])
    g=groups.median()
    g["thickness"]=groups.z.quantile(.9)-groups.z.quantile(.1)
    return g[groups.size()>=4].reset_index()


def plane_residual(x,y,z):
    X=np.column_stack([x,y,np.ones(len(x))])
    return z-X@np.linalg.lstsq(X,z,rcond=None)[0]


def geometry(a,traj,pack,out,figure_dir):
    frozen=pd.read_csv(pack.open("analysis/geometry/matched_floor_grid.csv"))
    floor,_=horizontal(a,traj,20,2.5)
    g=frozen.merge(grid(floor),on=["i","j"],how="left",validate="one_to_one")
    g.to_csv(out/"frozen_floor_cells.csv",index=False)
    valid=g.dropna(subset=["z"])
    e0=plane_residual(valid.x_after,valid.y_after,valid.z_after)
    e1=plane_residual(valid.x,valid.y,valid.z)
    rms=lambda x:float(np.sqrt(np.mean(np.square(x))))
    stats=dict(fixed_cells=len(g),supported_cells=len(valid),floor_rms_old_new_m=[rms(e0),rms(e1)],
               floor_residual_correlation=float(np.corrcoef(e0,e1)[0,1]),
               floor_width_p95_old_new_m=[float(x.quantile(.95)) for x in (valid.thickness_after,valid.thickness)])
    surfaces=horizontal(a,traj,8,3.)
    ref=pd.read_csv(pack.open("analysis/geometry/long_segments_profiles.csv"))
    fixed=json.load(pack.open("analysis/geometry/long_segments_matched_stats.json"))
    stats["segments"]={}
    all_bins=[]
    normals={}
    fig,axs=plt.subplots(4,2,figsize=(12,12),layout="constrained")
    for row,(seg,definition) in enumerate(fixed["definitions"].items()):
        start=np.array(definition["start_xy"])
        vector=np.array(definition["end_xy"])-start
        u=vector/np.linalg.norm(vector)
        cross=np.array([-u[1],u[0]])
        for col,(typ,q) in enumerate(zip(("floor","ceiling"),surfaces)):
            ds=(q[:,:2]-start)@u
            lateral=(q[:,:2]-start)@cross
            keep=(ds>0)&(ds<np.linalg.norm(vector))&(abs(lateral)<1.8)
            df=pd.DataFrame(q[keep],columns=list("xyz"))
            df["s"]=ds[keep]
            df["sbin"]=np.floor(ds[keep]).astype(int)
            df["cbin"]=np.floor(2*lateral[keep]).astype(int)
            cells=df.groupby(["sbin","cbin"])
            med=cells.median()[cells.size()>=3]
            A=np.column_stack([med.x,med.y,np.ones(len(med))])
            coef=least_squares(lambda par:A@par-med.z,np.linalg.lstsq(A,med.z,rcond=None)[0],loss="soft_l1",f_scale=.03).x
            n=np.r_[-coef[:2],1.]
            normals[(seg,typ)]=n/np.linalg.norm(n)
            samples=df.groupby("sbin")
            profile=samples.median()[samples.size()>=8].reset_index()
            old=ref[(ref.segment==seg)&(ref.surface==typ)]
            old_after=old[(old.series=="after")&old.sbin.isin(old[old.series=="before"].sbin)]
            joined=old_after.merge(profile[["sbin","s","z"]],on="sbin",how="left",suffixes=("_old","_new"))
            all_bins.append(joined)
            matched=joined.dropna(subset=["z_new"])
            result=dict(fixed_bins=len(joined),supported_bins=len(matched))
            for suffix,color,label in (("old","#af5c25","上一轮回环"),("new","#126392","本轮结构与惯性优化")):
                ss,zz=matched["s_"+suffix],matched["z_"+suffix]
                fit=np.polyfit(ss,zz,1)
                result[suffix]=dict(slope_deg=float(np.degrees(np.arctan(fit[0]))),rms_m=rms(zz-np.polyval(fit,ss)),
                                    end_minus_start_m=float(zz.iloc[-1]-zz.iloc[0]))
                axs[row,col].plot(ss,zz,color=color,label=label,lw=1.5)
            stats["segments"].setdefault(seg,{})[typ]=result
            axs[row,col].set(title=SEGMENT_NAMES[seg]+(" · 地面" if typ=="floor" else " · 顶面"),
                             xlabel="沿固定走廊的距离（m）",ylabel="地图高度（m）")
            axs[row,col].grid(alpha=.2)
    axs[0,0].legend()
    fig.savefig(figure_dir/"走廊地面与顶面_沿程高度.png",dpi=160)
    plt.close(fig)
    pd.concat(all_bins).to_csv(out/"frozen_profiles.csv",index=False)
    stats["normals"]={}
    for seg in fixed["definitions"]:
        n=normals[(seg,"floor")]
        angle=lambda m:float(np.degrees(np.arccos(np.clip(n@m,-1,1))))
        stats["normals"][seg]=dict(floor=n.tolist(),ceiling=normals[(seg,"ceiling")].tolist(),
            main_floor_angle_deg=angle(normals[("Main corridor","floor")]),
            floor_ceiling_angle_deg=angle(normals[(seg,"ceiling")]))
    return stats


def projections(old,new,out):
    clouds=[old[::max(1,len(old)//400000),:3],new[::max(1,len(new)//400000),:3]]
    lo=np.minimum(np.min(old[:,:3],0),np.min(new[:,:3],0))
    hi=np.maximum(np.max(old[:,:3],0),np.max(new[:,:3],0))
    for a,b in ((0,1),(0,2),(1,2)):
        fig,axs=plt.subplots(2,1,figsize=(10,8 if b==1 else 4),sharex=True,sharey=True,layout="constrained")
        for ax,pts,label in zip(axs,clouds,("上一轮回环 · watermark_joint","本轮 · 共享平面与惯性方向联合优化")):
            ax.scatter(pts[:,a],pts[:,b],c=pts[:,2],s=.06,cmap="viridis",vmin=-2,vmax=4,rasterized=True)
            ax.set(aspect="equal",title=label,xlim=(lo[a]-1,hi[a]+1),ylim=(lo[b]-.5,hi[b]+.5),
                   xlabel="XYZ"[a]+" (m)",ylabel="XYZ"[b]+" (m)")
            ax.grid(alpha=.16)
        fig.savefig(out/("同尺度_"+"XYZ"[a]+"XYZ"[b]+".png"),dpi=200)
        plt.close(fig)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("geometry","sampled_geometry","sampled_rgb","baseline","trajectory","reference_zip","metrics_dir"):
        p.add_argument(name,type=Path)
    args=p.parse_args()
    out=args.geometry.parent
    geom,_=read_spatial_sample(args.sampled_geometry)
    rgb,rgb_fields=read_spatial_sample(args.sampled_rgb)
    if not {"rgb","rgba"}.intersection(rgb_fields):
        raise ValueError("missing color field")
    baseline,_=binary_map(args.baseline)
    with zipfile.ZipFile(args.reference_zip) as pack:
        stats=geometry(geom,np.loadtxt(args.trajectory)[:,1:4],pack,args.metrics_dir,out)
    projections(baseline,binary_map(args.geometry)[0],out)
    stats.update(spatial_5cm_geometry=len(geom),spatial_5cm_rgb=len(rgb))
    (args.metrics_dir/"geometry_metrics.json").write_text(json.dumps(stats,indent=2)+"\n",encoding="utf8")
