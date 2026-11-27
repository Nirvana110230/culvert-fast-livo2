"""Measure withheld surfaces in the fixed target-visit frame, without refitting poses."""
import numpy as np
from scipy.spatial import cKDTree

from frame_io import rigid_inverse, transform, voxel


def normal_at(points):
    tree = cKDTree(points)
    hood = points[tree.query(points, k=min(14, len(points)))[1]]
    centered = hood - hood.mean(axis=1, keepdims=True)
    v, e = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
    return e[:, :, 0], (v[:, 0]<.18*np.maximum(1e-10, v[:, 1])) & (v[:, 1]>.025*v[:, 2])


def find_planes(old):
    ns, planar = normal_at(old)
    groups = []
    floor = planar & (np.abs(ns[:, 2])>.93) & (old[:, 2]<-.18) & (old[:, 2]>-3)
    vertical = planar & (np.abs(ns[:, 2])<.22)
    directions = [(np.array([0., 0., 1.]), floor, "floor")]
    if np.count_nonzero(vertical) >= 20:
        a = np.arctan2(ns[vertical, 1], ns[vertical, 0]) % np.pi
        # Orthogonal indoor wall families share the fourth-harmonic mean.
        phi = np.angle(np.mean(np.exp(4j*a))) / 4
        for angle, name in [(phi, "wall_a"), (phi+np.pi/2, "wall_b")]:
            n = np.array([np.cos(angle), np.sin(angle), 0.])
            directions.append((n, vertical & (np.abs(ns @ n)>.94), name))
    for axis, mask, label in directions:
        dots = old @ axis
        for k in range(2 if label.startswith("wall") else 1):
            vals = dots[mask]
            if len(vals) < 24:
                break
            bins = np.floor(vals/.10).astype(int)
            locs, count = np.unique(bins, return_counts=True)
            center = (locs[np.argmax(count)]+.5)*.1
            take = mask & (np.abs(dots-center)<.13)
            if np.count_nonzero(take) < 24:
                break
            anchor = old[take]
            cent = np.mean(anchor, axis=0)
            _, _, basis = np.linalg.svd(anchor-cent, full_matrices=False)
            n = basis[-1]
            if np.dot(n, axis)<0:
                n = -n
            if np.dot(n, axis)>.93:
                groups.append((label + "_" + str(k+1), cent, n, basis[:2].T, anchor))
            mask = mask & (np.abs(dots-center)>.85)
    return groups


def plane_measure(source, center, normal, tangent, anchor):
    signed = (source-center) @ normal
    near = np.abs(signed) < .85
    tang = (source[near]-center) @ tangent
    foot = (anchor-center) @ tangent
    if len(tang) < 12:
        return {"n": 0}
    tree = cKDTree(foot)
    d = tree.query(tang)[0]
    vals = signed[near][d<.28]
    if len(vals)<12:
        return {"n": len(vals)}
    chosen = source[near][d<.28]
    # Limit influence of adjoining planes on the inter-visit thickness.
    bins = np.floor(vals/.10).astype(int)
    loc, ct = np.unique(bins, return_counts=True)
    peak = (loc[np.argmax(ct)]+.5)*.10
    kept = np.abs(vals-peak)<.20
    inliers = vals[kept]
    if len(inliers)<12:
        return {"n": len(inliers)}
    ref_signed = (anchor-center) @ normal
    mix = np.r_[np.quantile(ref_signed, np.linspace(0,1,101)), np.quantile(inliers,np.linspace(0,1,101))]
    center_fit=chosen[kept].mean(axis=0)
    _, _, basis = np.linalg.svd(chosen[kept]-center_fit, full_matrices=False)
    fit_normal=basis[-1]*np.sign(basis[-1]@normal)
    return {"n": len(inliers), "candidate_n": len(vals), "target_n": len(anchor),
            "target_footprint_coverage": float(np.mean(cKDTree((chosen[kept]-center)@tangent).query(foot)[0]<.28)),
            "offset_m": float(np.median(inliers)-np.median(ref_signed)),
            "source_normal":fit_normal.tolist(),"source_plane_centroid_m":center_fit.tolist(),
            "two_visit_thickness_p90_p10_m": float(np.quantile(mix, .9)-np.quantile(mix, .1)),
            "angle_deg": float(np.rad2deg(np.arccos(np.clip(abs(basis[-1]@normal), 0, 1))))}


def opening_gaps(points,center,n):
    # Scan visibility can create a similar gap. Only describe a geometric opening
    # candidate, with empty surface returns common to three height bands.
    direction=np.array([-n[1],n[0],0.])
    direction/=np.linalg.norm(direction)
    region=points[np.abs((points-center)@n)<.17]
    if len(region)<24:
        return []
    gaps=[]
    width=.16
    for z0,z1 in ((-.72,-.22),(-.22,.22),(.22,.72)):
        t=(region[(region[:,2]>=z0)&(region[:,2]<z1)]-center)@direction
        bins=np.unique(np.floor(t/width).astype(int))
        gap=[]
        for a,b in zip(bins[:-1],bins[1:]):
            left,right=(a+1)*width,b*width
            if .60<=right-left<=2.40 and np.count_nonzero((bins>=a-3)&(bins<=a))>=2 and np.count_nonzero((bins>=b)&(bins<=b+3))>=2:
                gap.append((float(left),float(right)))
        gaps.append(gap)
    output=[]
    for a,b in gaps[1]:
        matches=[min(g,key=lambda v:abs(a-v[0])+abs(b-v[1]),default=None) for g in gaps]
        if all(v and abs(v[0]-a)<=.48 and abs(v[1]-b)<=.48 for v in matches):
            left=float(np.median([v[0] for v in matches]))
            right=float(np.median([v[1] for v in matches]))
            output.append({"left_m":left,"right_m":right,"width_m":right-left})
    return output


def region_clouds(builder,before,after,i,j):
    origin=before[i,:3,3]
    target=transform(builder.scan(i),before[i])-origin
    old=voxel(target[np.sum(target[:,:2]**2,axis=1)<16],.14)
    scans={}
    for name,poses in (("before",before),("after",after)):
        correction=before[i]@rigid_inverse(poses[i])
        slices=[]
        for h in range(max(0,j-1),min(len(poses),j+2)):
            body=builder.scan(h)
            odom=transform(body,before[h])
            keep=np.sum((odom[:,:2]-before[j,:2,3])**2,axis=1)<16.
            slices.append(transform(body[keep],correction@poses[h])-origin)
        scans[name]=voxel(np.vstack(slices),.14)
    return old,scans


def evaluate(hold, builder, before, after):
    from surface_quality import certify
    result = []
    for i, j in hold:
        origin=before[i,:3,3]
        old,scans=region_clouds(builder,before,after,i,j)
        row = {"target_id":builder.frames[i].id,"source_id":builder.frames[j].id,"target_index":i,"source_index":j,
               "target_center_map_gravity_m":origin.tolist(),"identity":"odometry-proposed unlabelled revisit",
               "old_points":len(old),"point_to_point":{},"surfaces":[],"parallel_wall_spacing":[],
               "corner_candidates":[],"opening_candidates":[]}
        result.append(row)
        if min([len(old),*[len(x) for x in scans.values()]]) < 35:
            row["quality"]={"status":"insufficient_region_support","surfaces":{}}
            continue
        tree = cKDTree(old)
        for name, cloud in scans.items():
            d=tree.query(cloud)[0]
            reverse=cKDTree(cloud).query(old)[0]
            row["point_to_point"][name]={"source_points":len(cloud),"source_median_m":float(np.median(d)),
                "source_p90_m":float(np.quantile(d,.9)), "target_median_m":float(np.median(reverse)),
                "source_coverage_32cm":float(np.mean(d<.32)),"target_coverage_32cm":float(np.mean(reverse<.32))}
        planes = find_planes(old)
        for label, center, normal, tangent, anchor in planes:
            row["surfaces"].append({"type":label,"centroid_m":center.tolist(),"normal":normal.tolist(),
                                    **{n:plane_measure(v,center,normal,tangent,anchor) for n,v in scans.items()}})
            if label.startswith("wall"):
                openings=opening_gaps(old,center,normal)
                for op in openings:
                    result_gap={"wall":label,"target":op,"identity":"unlabelled gap in three height bands"}
                    for name,cloud in scans.items():
                        offset=row["surfaces"][-1][name].get("offset_m")
                        if offset is None:
                            continue
                        sg=opening_gaps(cloud,center+offset*normal,normal)
                        matches=[v for v in sg if abs(v["left_m"]-op["left_m"])<.65 and abs(v["right_m"]-op["right_m"])<.65]
                        if matches:
                            result_gap[name]=min(matches,key=lambda v:abs(v["left_m"]-op["left_m"])+abs(v["right_m"]-op["right_m"]))
                    row["opening_candidates"].append(result_gap)
        for a,pa in zip(row["surfaces"],planes):
            for b,pb in zip(row["surfaces"],planes):
                if not (a["type"].startswith("wall_a") and b["type"].startswith("wall_b")):
                    continue
                pair=np.array([pa[2][:2],pb[2][:2]])
                corner=np.linalg.solve(pair,np.array([pa[1][:2]@pair[0],pb[1][:2]@pair[1]]))
                if any(np.min(np.linalg.norm(p[4][:,:2]-corner,axis=1))>.60 for p in (pa,pb)):
                    continue
                c={"walls":[a["type"],b["type"]],"target_xy_m":corner.tolist(),
                   "target_angle_deg":float(np.rad2deg(np.arccos(abs(pa[2]@pb[2]))))}
                for name in scans:
                    if all("source_normal" in p[name] for p in (a,b)):
                        n=np.array([a[name]["source_normal"],b[name]["source_normal"]])
                        centers=np.array([a[name]["source_plane_centroid_m"],b[name]["source_plane_centroid_m"]])
                        c[name+"_angle_deg"]=float(np.rad2deg(np.arccos(np.clip(abs(n[0]@n[1]),0,1))))
                        if abs(np.linalg.det(n[:,:2]))>.5:
                            pos=np.linalg.solve(n[:,:2],np.sum(n*centers,axis=1))
                            c[name+"_xy_deviation_m"]=float(np.linalg.norm(pos-corner))
                row["corner_candidates"].append(c)
        for family in ("wall_a","wall_b"):
            pair=[p for p in row["surfaces"] if p["type"].startswith(family)]
            if len(pair)==2:
                a,b=pair
                n=np.array(a["normal"])
                span=float((np.array(b["centroid_m"])-a["centroid_m"])@n)
                if abs(np.dot(n,b["normal"])) < .95:
                    continue
                m={"family":family,"target_spacing_m":abs(span)}
                for name in scans:
                    if all("offset_m" in p[name] for p in pair):
                        m[name+"_source_spacing_m"]=abs(span+b[name]["offset_m"]*np.dot(n,b["normal"])-a[name]["offset_m"])
                row["parallel_wall_spacing"].append(m)
        row["quality"]=certify(row,planes,scans)
    return result
