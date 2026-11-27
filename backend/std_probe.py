"""Build an isolated, pinned upstream STD descriptor/search probe and prepare masked submaps."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import urllib.request


SHA="6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb"
FILES={"include/STDesc.h":"29655c448741c38888c05b7def9360d0266c847c2728f309e09b6ee2e0d008c5",
       "src/STDesc.cpp":"5993b11cda237941d78355a25f8fa63b5dd3de9e7a6cb4b306aca121a57adf0f",
       "LICENCE":"a78aa507531e05215fec3b74f0d130bb38fd3518bd3fb5292fa364976242ea89",
       "config/config_livox.yaml":"e5e20283cdc77ced2958cf0435b5dbc6f96233995f5b4a5b2ffb3a1d375ed604"}


def cut(s,first,last):
    start=s.index(first)
    return s[:start]+s[s.index(last,start):]


def build(root,source):
    import yaml
    root.mkdir(parents=True,exist_ok=False)
    for name,h in FILES.items():
        data=(source/name).read_bytes() if source else urllib.request.urlopen(f"https://raw.githubusercontent.com/hku-mars/STD/{SHA}/{name}",timeout=30).read()
        if hashlib.sha256(data).hexdigest()!=h:
            raise ValueError(name)
        path=root/name;path.parent.mkdir(exist_ok=True,parents=True);path.write_bytes(data)
    header=(root/"include/STDesc.h").read_text()
    header=re.sub(r'^#include <(?:ceres|ros|visualization_msgs)/.*\n','',header,flags=re.M)
    header=cut(header,"void read_parameters(","double time_inc(")
    header=cut(header,"void publish_std_pairs(","bool attach_greater_sort(")
    header=cut(header,"struct PlaneSolver {","class STDescManager {")
    (root/"descriptor.h").write_text("#include <chrono>\n"+header)
    src=(root/"src/STDesc.cpp").read_text().replace('#include "include/STDesc.h"','#include "descriptor.h"')
    src=cut(src,"void read_parameters(","void load_pose_with_time(")
    src=cut(src,"void publish_std_pairs(","void STDescManager::GenerateSTDescs(")
    src=cut(src,"void STDescManager::PlaneGeomrtricIcp(","void OctoTree::init_plane(")
    src=src.replace('ROS_ERROR_STREAM("No STDescs!");','')
    src=src.replace("int near_num = config_setting_.descriptor_near_num_;","int near_num = std::min<int>(corner_points->size(),config_setting_.descriptor_near_num_);")
    src=src.replace("int K = 3;","int K = std::min<int>(3, target_cloud->size());")
    (root/"descriptor.cpp").write_text(src)
    cfg=yaml.safe_load((root/"config/config_livox.yaml").read_text())
    (root/"params.inc").write_text("\n".join(f"cfg.{k}_ = {v};" for k,v in cfg.items())+"\n")
    command=["g++","-std=c++17","-O1","-g0","-fopenmp","-I/usr/include/eigen3","-I/usr/include/pcl-1.12",
             "-I"+str(root),str(root/"descriptor.cpp"),str(Path(__file__).with_name("std_driver.cpp")),
             "-lpcl_common","-lpcl_kdtree","-lflann","-llz4","-o",str(root/"std_probe")]
    subprocess.run(command,check=True)
    (root/"build.json").write_text(json.dumps({"commit":SHA,"sources":FILES,"compile":command,"params":cfg,
            "scope":"upstream GenerateSTDescs, SearchLoop, AddSTDescs; ROS1, Ceres refinement and whole mapper excluded"},indent=2))


def prepare(index,gravity,plan,out):
    import numpy as np
    from frame_io import frames_and_gravity,read_pcd,transform,voxel,save_json
    frames,G,_=frames_and_gravity(index,gravity)
    P=np.array([G@f.odom for f in frames])
    setup=json.loads(plan.read_text())
    excluded=set(setup["heldout_frame_indices"])
    centers=np.array(setup["holdout_centers_xy_m"])
    out.mkdir(parents=True,exist_ok=False)
    groups=[]
    with (out/"list.txt").open("w") as listing:
        for grp,lo in enumerate(range(0,len(frames),10)):
            hi=min(lo+10,len(frames))
            mid=(lo+hi-1)//2
            anchor=P[mid,:3,3]
            pcs=[]
            for k in range(lo,hi):
                if k in excluded:
                    continue
                raw,_=read_pcd(frames[k].cloud)
                world=transform(raw,P[k])
                for c in centers:
                    world=world[np.sum((world[:,:2]-c)**2,axis=1)>16.]
                pcs.append(voxel(world-anchor,.25))
            xyz=np.vstack(pcs) if pcs else np.empty((0,3))
            path=out/f"{grp:04d}.bin"
            xyz.astype("<f4").tofile(path)
            listing.write(str(path)+"\n")
            groups.append({"group":grp,"start_index":lo,"end_index":hi-1,"center_index":mid,"anchor_m":anchor.tolist(),"points":len(xyz)})
    save_json(out/"groups.json",groups)


def audit(result,groups,out):
    import csv
    import numpy as np
    import small_gicp
    from scipy.spatial import cKDTree
    from frame_io import transform,voxel,save_json
    poses=json.loads((groups/"groups.json").read_text())
    cases=[]
    for x in csv.DictReader(result.open()):
        i,j=int(x["target"]),int(x["group"])
        if i<0:
            continue
        T=np.eye(4)
        T[:3,:3]=np.array([[float(x[f"r{r}{c}"]) for c in range(3)] for r in range(3)])
        T[:3,3]=[float(x[n]) for n in ("tx","ty","tz")]
        expected=np.array(poses[j]["anchor_m"])-poses[i]["anchor_m"]
        first,second=[voxel(np.fromfile(groups/f"{k:04d}.bin",dtype="<f4").reshape(-1,3).astype(float),.18) for k in (i,j)]
        refined=small_gicp.align(first,second,init_T_target_source=T,registration_type="GICP",downsampling_resolution=.18,
                                 max_correspondence_distance=.9,max_iterations=40,num_threads=2)
        row={"source":j,"target":i,"score":float(x["score"]),"triangles":int(x["matched_triangles"]),
             "odom_anchor_distance_m":float(np.linalg.norm(expected)),"gicp_converged":bool(refined.converged)}
        for name,P in (("std",T),("gicp",refined.T_target_source)):
            s=transform(second,P)
            ds=cKDTree(first).query(s)[0];dt=cKDTree(s).query(first)[0]
            row[name]={"gravity_disagreement_deg":float(np.rad2deg(np.arccos(np.clip(P[2,2],-1,1)))),
                       "translation_prior_disagreement_m":float(np.linalg.norm(P[:3,3]-expected)),
                       "source_coverage_32cm":float(np.mean(ds<.32)),"target_coverage_32cm":float(np.mean(dt<.32)),
                       "source_distance_median_m":float(np.median(ds))}
        cases.append(row)
    save_json(out,cases)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest="mode",required=True)
    b=sub.add_parser("build");b.add_argument("--out",type=Path,required=True);b.add_argument("--source",type=Path)
    b=sub.add_parser("prepare")
    for k in ("index","gravity","plan","out"):
        b.add_argument("--"+k,type=Path,required=True)
    b=sub.add_parser("audit")
    for k in ("result","groups","out"):
        b.add_argument("--"+k,type=Path,required=True)
    args=p.parse_args()
    if args.mode=="build":
        build(args.out,args.source)
    elif args.mode=="prepare":
        prepare(args.index,args.gravity,args.plan,args.out)
    else:
        audit(args.result,args.groups,args.out)
