"""Joint Pose3 and local plane adjustment from native keyframe observations."""
from collections import defaultdict

import gtsam
import numpy as np
from scipy.spatial import cKDTree

from frame_io import transform
from heldout import normal_at


def local_patches(builder, initial, selected):
    """Associate by spatial overlap of measured supports and reciprocal plane distances."""
    groups = defaultdict(list)
    for i in selected[::5]:
        if i in builder.excluded:
            continue
        body = builder.scan(i)
        body = body[np.linalg.norm(body,axis=1)<18]
        raw = transform(body,builder.poses[i])
        keep = np.ones(len(raw),bool)
        for c in builder.centers:
            keep &= np.sum((raw[:,:2]-c)**2,axis=1)>16
        body = body[keep]
        if len(body)<30:
            continue
        world = transform(body,initial[i])
        normals, flat = normal_at(world)
        direction = np.argmax(np.abs(normals),axis=1)
        # Z separating cells keep surfaces on different storeys distinct.
        labels = np.column_stack((np.floor(world/[6.,6.,2.]),direction)).astype(int)
        ids = np.flatnonzero(flat)
        order = np.lexsort(labels[ids].T[::-1])
        for part in np.split(ids[order],np.flatnonzero(np.any(np.diff(labels[ids[order]],axis=0),axis=1))+1):
            if len(part)<24:
                continue
            family = direction[part[0]]
            signs = np.sign(normals[part,family])
            normal = np.median(normals[part]*signs[:,None],axis=0)
            normal /= np.linalg.norm(normal)
            points = world[part[np.abs(normals[part]@normal)>.97]]
            if len(points)<24:
                continue
            # Separate parallel surfaces and clutter by signed-distance modes.
            base = points.mean(0)
            offset = (points-base)@normal
            bins, counts = np.unique(np.floor(offset/.12),return_counts=True)
            points = points[np.abs(offset-(bins[np.argmax(counts)]+.5)*.12)<.11]
            if len(points)<24:
                continue
            for _ in range(2):
                center = points.mean(0)
                vals, axes = np.linalg.eigh(np.cov((points-center).T,bias=True))
                n = axes[:,0] * (1 if axes[family,0]>=0 else -1)
                points = points[np.abs((points-center)@n)<.07]
            if len(points)<20 or vals[0]>.035**2 or vals[1]<.18**2:
                continue
            # Three quadrature points reproduce the support's in-plane
            # first and second moments without overweighting dense scans.
            triangle = np.array([[np.sqrt(2),0],[-1/np.sqrt(2),np.sqrt(1.5)],[-1/np.sqrt(2),-np.sqrt(1.5)]])
            support = center + (triangle*np.sqrt(vals[1:]))@axes[:,1:].T
            local = (support-initial[i,:3,3])@initial[i,:3,:3]
            label = tuple(labels[part[0]])
            tree = cKDTree(points)
            observation = (int(i),local)
            for group in groups[label]:
                if group["normal"]@n<.996 or abs((center-group["center"])@group["normal"])>.09:
                    continue
                if abs((group["center"]-center)@n)>.09:
                    continue
                if not any(np.count_nonzero(t.query(points,distance_upper_bound=.45)[0]<.45)>=8
                           for t in group["footprints"]):
                    continue
                group["obs"].append(observation)
                if len(group["footprints"])<5:
                    group["footprints"].append(tree)
                break
            else:
                typ = ("wall" if family!=2 else
                       "floor" if center[2]<initial[i,2,3] else "ceiling")
                groups[label].append(dict(center=center,normal=n,axes=axes[:,1:],kind=typ,
                                          obs=[observation],footprints=[tree]))
    result = []
    for alternatives in groups.values():
        for group in alternatives:
            obs = group["obs"]
            if len(obs)<4:
                continue
            ids = [k for k,_ in obs]
            if np.max(np.linalg.norm(initial[ids,:3,3]-initial[ids[0],:3,3],axis=1))<2:
                continue
            group.pop("footprints")
            chosen = np.linspace(0,len(obs)-1,min(14,len(obs)),dtype=int)
            group["obs"] = [obs[k] for k in chosen]
            result.append(group)
    return result


def point_factor(i, key, local, group, model):
    anchor, normal, axes = (group[name] for name in ("center","normal","axes"))
    def residual(this, values, H):
        pose = values.atPose3(i)
        R = pose.rotation().matrix()
        plane = values.atVector(key)
        v = normal + axes@plane[:2]
        length = np.linalg.norm(v)
        n = v/length
        lever = local @ R.T + (pose.translation()-anchor)
        if H is not None:
            body_n = n@R
            H[0] = np.column_stack((np.cross(local,body_n),np.broadcast_to(body_n,local.shape)))
            H[1] = np.column_stack((lever @ (axes - np.outer(n,n@axes))/length,np.ones(len(local))))
        return lever@n + plane[2]
    return gtsam.CustomFactor(model,[i,key],residual)


def joint_graph(frames, initial, archive, selected, groups, directions, fit_planes=True):
    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
    first, last = int(selected[0]), int(selected[-1])
    for i in selected:
        values.insert(int(i),gtsam.Pose3(initial[i]))
    graph.add(gtsam.PriorFactorPose3(first,gtsam.Pose3(initial[first]),gtsam.noiseModel.Isotropic.Sigma(6,1e-5)))
    for edge in archive:
        i,j=edge["target_index"],edge["source_index"]
        if i<first or j>last or j<first or i>last:
            continue
        if edge["kind"]=="odom":
            model = gtsam.noiseModel.Diagonal.Sigmas(np.array(edge["sigmas_rot_rad_trans_m"]))
        else:
            model = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(edge["huber_k"]),
                         gtsam.noiseModel.Gaussian.Covariance(np.array(edge["covariance_source_tangent"])))
        graph.add(gtsam.BetweenFactorPose3(i,j,gtsam.Pose3(np.array(edge["T_target_source"])),model))
    fitted_count = 0
    held_count = sum(len(group["obs"][2::5]) for group in groups)
    plane_model = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.8),
                                                  gtsam.noiseModel.Isotropic.Sigma(3,.065))
    for j, group in enumerate(groups if fit_planes else []):
        key = gtsam.symbol('l',j)
        values.insert(key,np.zeros(3))
        for k,(i,support) in enumerate(group["obs"]):
            if k%5==2:
                continue
            graph.add(point_factor(i,key,support,group,plane_model))
            fitted_count += 1
    for i,body,kind in directions:
        # Sigma includes motion differentiation, correlated odometry and unknown
        # accelerometer bias. Each disjoint interval contributes one direction.
        sigma = np.deg2rad(.7 if kind=="stationary" else 1.25)
        graph.add(gtsam.Pose3AttitudeFactor(i,gtsam.Unit3(np.array([0.,0.,1.])),
                    gtsam.noiseModel.Isotropic.Sigma(2,sigma),gtsam.Unit3(body)))
    params=gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(30)
    optimizer=gtsam.LevenbergMarquardtOptimizer(graph,values,params)
    result=optimizer.optimize()
    poses=np.array([result.atPose3(int(k)).matrix() for k in selected])
    # Refit each evaluation plane on TRAINING supports for each trajectory.
    # This removes the advantage of optimizing the plane's starting estimate.
    def evaluate(poses):
        errors=[]
        for group in groups:
            train, hold=[],[]
            for k,(i,points) in enumerate(group["obs"]):
                (hold if k%5==2 else train).extend(transform(points,poses[i-first]))
            a=np.array(train)
            c=a.mean(0)
            n=np.linalg.eigh((a-c).T@(a-c))[1][:,0]
            errors.extend((np.array(hold)-c)@n)
        return float(np.sqrt(np.mean(np.square(errors)))) if errors else None
    counts={k:sum(g["kind"]==k for g in groups) for k in ("floor","ceiling","wall")}
    turn=0
    long_visits=0
    associations=[]
    for j,group in enumerate(groups):
        ids=[i for i,_ in group["obs"]]
        plane=result.atVector(gtsam.symbol('l',j)) if fit_planes else np.zeros(3)
        n=group["normal"]+group["axes"]@plane[:2]
        associations.append(dict(surface=group["kind"],anchor=group["center"].tolist(),
            initial_normal=group["normal"].tolist(),optimized_normal=(n/np.linalg.norm(n)).tolist(),
            optimized_offset_at_anchor_m=float(plane[2]),training_ids=[i for k,i in enumerate(ids) if k%5!=2],
            validation_ids=[i for k,i in enumerate(ids) if k%5==2]))
        # Use trajectory tangents (mod pi) to distinguish a turn from a return trip.
        chords=initial[np.minimum(np.array(ids)+2,last),:2,3]-initial[np.maximum(np.array(ids)-2,first),:2,3]
        chords=chords[np.linalg.norm(chords,axis=1)>.6]
        if len(chords)>1:
            chords/=np.linalg.norm(chords,axis=1)[:,None]
            turn+=int(np.min(abs(chords@chords.T))<np.cos(np.deg2rad(30)))
        long_visits+=int((frames[max(ids)].stamp-frames[min(ids)].stamp)*1e-9>75)
    return poses,dict(planes=counts,planes_in_graph=fit_planes,cross_heading_planes=turn,multi_visit_planes=long_visits,
                      fitted_blocks=fitted_count,held_blocks=held_count,direction_factors=len(directions),
                      held_plane_rms_before_m=evaluate(initial[selected]),held_plane_rms_after_m=evaluate(poses),
                      graph_initial=graph.error(values),graph_final=graph.error(result),iterations=optimizer.iterations()),associations
