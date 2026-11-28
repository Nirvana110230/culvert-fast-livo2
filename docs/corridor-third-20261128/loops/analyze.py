"""Reproduce third-run trajectory, loop graph and fixed-identity holdout measurements."""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from fetch_inputs import HERE, ALGORITHM_ROOT, RUN, FULL, DEFAULT_DATA, sha
sys.path.insert(0, str(ALGORITHM_ROOT/'backend'))
from frame_io import frames_and_gravity, read_pcd, rigid_inverse, transform, voxel
from heldout import find_planes, normal_at, plane_measure
from surface_quality import certify
from verify import load_tum


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def save(name, obj):
    (HERE/name).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')


def write_rows(name, rows):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with (HERE/name).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, keys)
        writer.writeheader()
        for r in rows:
            writer.writerow({k:json.dumps(v, separators=(',', ':')) if isinstance(v, (list,dict)) else v for k,v in r.items()})


def stats(v):
    v = np.array(v)
    return dict(n=len(v), min=float(v.min()), median=float(np.median(v)), p90=float(np.quantile(v,.90)),
                p95=float(np.quantile(v,.95)), max=float(v.max()), rms=float(np.sqrt(np.mean(v*v))))


def mmatrix(rows):
    M = np.tile(np.eye(4), (len(rows), 1, 1))
    M[:, :3, :3] = Rotation.from_quat([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]).as_matrix()
    M[:, :3, 3] = [[float(r[k]) for k in ('tx','ty','tz')] for r in rows]
    return M


def log6(pose):
    """SE(3) right-tangent logarithm in omega, v order."""
    w = Rotation.from_matrix(pose[:3,:3]).as_rotvec()
    angle = np.linalg.norm(w)
    skew = np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
    factor = 1/12 if angle < 1e-5 else (1-angle/(2*np.tan(angle/2)))/angle**2
    return np.r_[w, (np.eye(3)-skew/2+factor*skew@skew)@pose[:3,3]]


def edge_key(e):
    return e['target_id'], e['source_id']


def analyze_graph(folder, live, times, raw, corrected, issued, odom, windows, data):
    graph = [json.loads(line) for line in (folder/'graph.jsonl').read_text().splitlines()]
    online = read(folder/'online_summary.json')
    offline = read(data/'pgo/summary.json')
    edges = {'online':read(folder/'edges.json'), 'offline':read(data/'pgo/edges.json')}
    suites = {'online':folder, 'offline':data/'pgo'}
    A = raw[:,:3,3]
    center = np.median(A[(times>=windows[0])&(times<=windows[1])],axis=0)
    d0 = np.linalg.norm(A[:,:2]-center[:2],axis=1)
    old = set()
    revisions = []
    queries = []
    rows = list(csv.DictReader((folder/'issued.csv').open()))
    for q in graph:
        k = q['available_through_id']
        curr = set(map(tuple,q['active_ids']))
        new, lost = sorted(curr-old), sorted(old-curr)
        assert q['revision']==int(rows[k]['revision']) and q['active_loops']==int(rows[k]['active_loops'])
        row = dict(frame=k, sensor_s=times[k], source=q['candidate_source_id'], revision=q['revision'],
                   elapsed_s=q['elapsed_s'], active=len(curr), added=new, removed=lost,
                   raw_x=A[k,0],raw_y=A[k,1],raw_z=A[k,2],raw_p0_xy_m=d0[k],candidates=len(q['candidates']))
        if old != curr:
            jump = live[k]@rigid_inverse(live[k-1])
            row.update(tf_translation_step_m=float(np.linalg.norm(live[k,:3,3]-live[k-1,:3,3])),
                       tf_angle_step_deg=float(Rotation.from_matrix(jump[:3,:3]).magnitude()*180/np.pi),
                       body_position_step_due_to_tf_m=float(np.linalg.norm(issued[k,:3,3]-transform(odom[k,:3,3],live[k-1]))))
            revisions.append(row.copy())
        queries.append(row)
        old=curr
    assert len(revisions)==online['revisions'] and old==set(map(edge_key,edges['online']))
    write_rows('online_queries.csv',queries)
    write_rows('online_revisions.csv',revisions)
    stats_by_mode = {}
    rows_by_mode = []
    candidate_sets = {}
    for key, all_rows in [('online',[c for q in graph for c in q['candidates']]),('offline',edges['offline'])]:
        factors = read(suites[key]/'factors.json')
        loop_factors = {(r['target_index'],r['source_index']):r for r in factors if r['kind']!='odom'}
        final = {edge_key(r):r for r in edges[key] if r['accepted']}
        assert set(final)==set(loop_factors)
        candidates={edge_key(r) for r in all_rows}
        candidate_sets[key]=candidates
        assert len(candidates)==len(all_rows)
        by_field = []
        for e in all_rows:
            i,j = edge_key(e)
            included=(i,j) in final
            row=dict(mode=key,target=i,source=j,target_s=times[i],source_s=times[j],time_gap_s=times[j]-times[i],
                     target_x=A[i,0],target_y=A[i,1],source_x=A[j,0],source_y=A[j,1],
                     prior_distance_m=float(np.linalg.norm(A[j]-A[i])), raw_target_p0_xy_m=d0[i],raw_source_p0_xy_m=d0[j],
                     final=included,reasons=e['reasons'])
            for k in ('coverage_source','coverage_target','rms_inlier_m','neighbor_diff_m','neighbor_diff_deg',
                      'normal_moment_eigenvalues','normalized_point_plane_eigenvalues','weakest_translation_axis_map_gravity',
                      'neighbor_converged','neighbor_inliers','correction_m','correction_deg','submap_sizes','inlier_correspondences'):
                row[k]=e.get(k)
            if included:
                f=loop_factors[i,j]
                M=corrected[key]
                resid=rigid_inverse(np.asarray(f['T_target_source']))@rigid_inverse(M[i])@M[j]
                cov=np.asarray(f['covariance_source_tangent'])
                x=log6(resid)
                norm=float(np.linalg.norm(np.linalg.solve(np.linalg.cholesky(cov),x)))
                row.update(posterior_residual_m=float(np.linalg.norm(resid[:3,3])),
                           posterior_residual_deg=float(np.linalg.norm(x[:3])*180/np.pi),whitened_norm=norm,
                           huber_weight=min(1.,f['huber_k']/max(1e-15,norm)),sigmas_rot_rad_trans_m=f['sigmas_rot_rad_trans_m'],
                           covariance_source_tangent=cov.tolist(),directional_inflation=final[i,j]['directional_variance_inflation'])
                if key=='offline':
                    assert abs(norm-e['final_whitened_norm'])<1e-6
                by_field.append(row)
            rows_by_mode.append(row)
        fields=('coverage_source','coverage_target','rms_inlier_m','neighbor_diff_m','neighbor_diff_deg',
                'posterior_residual_m','posterior_residual_deg','whitened_norm','huber_weight','time_gap_s','prior_distance_m')
        stats_by_mode[key]=dict(candidates=len(all_rows),final=len(by_field),queries=len({e['source_id'] for e in all_rows}),
            rejection_labels=dict(Counter(r for e in all_rows for r in e['reasons'])),
            accepted={k:stats([r[k] for r in by_field]) for k in fields},
            sigma_medians=np.median([r['sigmas_rot_rad_trans_m'] for r in by_field],axis=0).tolist(),
            normal_smallest_eigenvalue=stats([r['normal_moment_eigenvalues'][0] for r in by_field]),
            weakest_axis_dominant_xyz=dict(Counter('xyz'[np.argmax(np.abs(r['weakest_translation_axis_map_gravity']))] for r in by_field)),
            weakest_mode_variance_inflation=stats([max(r['directional_inflation']) for r in by_field]),
            terminal_p0=[(r['target'],r['source']) for r in by_field if r['source_s']>=times[-1]-120 and
                         r['raw_target_p0_xy_m']<=3 and r['raw_source_p0_xy_m']<=3],
            accepted_ids=[(r['target'],r['source']) for r in by_field],factor_count_without_anchor=len(factors))
    write_rows('edge_diagnostics.csv', rows_by_mode)
    accepted={k:set(map(tuple,v['accepted_ids'])) for k,v in stats_by_mode.items()}
    hold=read(data/'pgo/plan.json')
    retain=set(hold['heldout_frame_indices'])
    centers=np.array(hold['holdout_centers_xy_m'])
    touches={}
    for key, subset in accepted.items():
        touches[key]={}
        for i,j in sorted(subset):
            neighborhood={k for v in (i,j) for k in range(max(v-2,0),min(v+3,len(times)))}
            overlaps=sorted(neighborhood&retain)
            distance=float(np.linalg.norm(A[[i,j],None,:2]-centers[None,:,:],axis=2).min())
            if overlaps or distance<=4:
                touches[key][f'{i}-{j}']=dict(support_frame_overlap=overlaps, nearest_holdout_center_m=distance)
    fig,axes=plt.subplots(2,2,figsize=(12.5,8.0),layout='constrained')
    for ax,key in zip(axes[0],['online','offline']):
        ax.plot(A[:,0],A[:,1],color='#999999',lw=.8)
        ordered=sorted(accepted[key], key=lambda p:p[1])
        for i,j in ordered:
            ax.plot(A[[i,j],0],A[[i,j],1],c='#985312',lw=.75)
        scatter=ax.scatter(A[[j for _,j in ordered],0],A[[j for _,j in ordered],1],c=times[[j for _,j in ordered]],cmap='viridis',s=20)
        ax.scatter(*center[:2],s=45,c='red',marker='x',label='P0 start')
        ax.set(xlabel='gravity X [m]',ylabel='gravity Y [m]',title=f'{key.capitalize()} accepted pairs ({len(ordered)})',aspect='equal')
        fig.colorbar(scatter,ax=ax,label='source sensor time [s]')
        ax.legend(fontsize=8)
    axes[1,0].step([0]+[r['sensor_s'] for r in queries],[0]+[r['active'] for r in queries],where='post',color='#1e718d')
    axes[1,0].set(xlabel='sensor time [s]',ylabel='active loop factors',title='Streaming factor history')
    axes[1,1].stem([r['sensor_s'] for r in revisions],[r['body_position_step_due_to_tf_m'] for r in revisions],basefmt=' ')
    axes[1,1].set(xlabel='sensor time [s]',ylabel='same-pose shift [m]',title='Position discontinuity at each issued correction')
    fig.savefig(HERE/'graph_history.png',dpi=180);plt.close(fig)
    return dict(modes=stats_by_mode,common_candidates=sorted(candidate_sets['offline']&candidate_sets['online']),
                common_accepted=sorted(accepted['online']&accepted['offline']),holdout_overlap=touches,
                offline_raw_proposals=hold['raw_proposals'],online_revision_count=len(revisions),
                query_seconds=stats(online['query_wall_s']),query_seconds_sum=sum(online['query_wall_s']),
                max_same_pose_jump=max(revisions,key=lambda r:r['body_position_step_due_to_tf_m']),
                max_tf_angle=max(revisions,key=lambda r:r['tf_angle_step_deg']),
                max_tf_translation=max(revisions,key=lambda r:r['tf_translation_step_m']),
                first_revision=revisions[0],last_revision=revisions[-1],online_resources=dict(elapsed_s=online['elapsed_s'],
                    peak_rss_MiB=online['peak_rss_MiB'],rebuild=online['rebuild']),
                offline_resources={k:offline[k] for k in ['wall_clock_s','peak_rss_MiB','registration_phase_s','rebuild_s']},
                sensitivity=read(data/'pgo/weight_sensitivity.json'))


def displacements(ns, poses, kposes, windows, origin):
    t=(ns-origin)*1e-9
    rows={}
    for method,xyz in poses.items():
        med=[]
        win=[]
        for lo,hi in windows.reshape(2,2):
            x=xyz[(t>=lo)&(t<=hi)]
            m=np.median(x, axis=0)
            med.append(m)
            win.append(dict(n=len(x),xyz_m=m.tolist(),radius_p95_m=float(np.quantile(np.linalg.norm(x-m,axis=1),.95))))
        delta=med[1]-med[0]
        key_delta=kposes[method][-1,:3,3]-kposes[method][0,:3,3]
        rows[method]=dict(windows=win,delta_xyz_m=delta.tolist(),delta_xy_m=float(np.linalg.norm(delta[:2])),
                          delta_3d_m=float(np.linalg.norm(delta)),key_delta_xyz_m=key_delta.tolist(),
                          key_delta_3d_m=float(np.linalg.norm(key_delta)),dense_z_range_m=float(np.ptp(xyz[:,2])))
    return rows


def fixed_clouds(frames, poses, i, j):
    raw=poses['A']
    origin=raw[i,:3,3]
    def scan(k):
        return voxel(read_pcd(frames[k].cloud)[0],.12)
    tgt=transform(scan(i),raw[i])-origin
    old=voxel(tgt[np.sum(tgt[:,:2]**2,axis=1)<16],.14)
    sets={m:[] for m in poses}
    for k in range(j-1,j+2):
        b=scan(k)
        a=transform(b,raw[k])
        keep=np.sum((a[:,:2]-raw[j,:2,3])**2,axis=1)<16
        for m,T in poses.items():
            sets[m].append(transform(b[keep],raw[i]@rigid_inverse(T[i])@T[k])-origin)
    clouds={m:np.vstack(slices) for m,slices in sets.items()}
    _,ids=np.unique(np.floor(clouds['A']/.14).astype(np.int32),axis=0,return_index=True)
    return old,{m:v[ids] for m,v in clouds.items()}


def plane_support(source, normals, planar, c, n, tangent, anchor, expected_count):
    """Recover the precise normal-compatible points retained by heldout.plane_measure."""
    compatible=source[planar & (np.abs(normals@n)>=np.cos(np.deg2rad(15)))]
    signed=(compatible-c)@n
    keep=np.abs(signed)<.85
    candidate=compatible[keep]
    near=cKDTree((anchor-c)@tangent).query((candidate-c)@tangent)[0]<.28
    vals=signed[keep][near]
    bins,counts=np.unique(np.floor(vals/.10).astype(int),return_counts=True)
    peak=(bins[np.argmax(counts)]+.5)*.10
    support=candidate[near][np.abs(vals-peak)<.20]
    assert len(support)==expected_count
    return support


def plane_height(xy, centroid, normal):
    return float(centroid[2]-((xy-centroid[:2])@normal[:2])/normal[2])


def measure_regions(frames,poses,pairs,data):
    pp=[];planes_out=[];local=[];holdout=[]
    colors={'target':'#127891','source':'#dd5c23'}
    fig,axs=plt.subplots(len(pairs),3,figsize=(11.1,13),layout='constrained',sharex='row',sharey='row')
    for idx,(i,j) in enumerate(pairs):
        old,clouds=fixed_clouds(frames,poses,i,j)
        tree=cKDTree(old)
        all_nn={}
        for method,X in clouds.items():
            d,ind=tree.query(X)
            all_nn[method]=(d,ind)
            for direction,a in [('S_to_T',d),('T_to_S',cKDTree(X).query(old)[0])]:
                row=dict(region=idx+1,target=i,source=j,method=method,direction=direction,**stats(a),coverage_32cm=float(np.mean(a<.32)))
                pp.append(row)
        ref=read(data/'pgo/holdout.json')[idx]['point_to_point']['before']
        assert abs(np.median(all_nn['A'][0])-ref['source_median_m'])<1e-7
        groups=find_planes(old)
        dummy=dict(surfaces=[],parallel_wall_spacing=[],corner_candidates=[],opening_candidates=[])
        for label,c,n,basis,anchor in groups:
            dummy['surfaces'].append(dict(type=label,**{m:plane_measure(x,c,n,basis,anchor) for m,x in clouds.items()}))
        quality=certify(dummy,groups,clouds)
        source_normals={m:normal_at(v) for m,v in clouds.items()}
        for label,c,n,basis,anchor in groups:
            observations=quality['surfaces'][label]
            supports={m:plane_support(clouds[m],*source_normals[m],c,n,basis,anchor,g['normal_filtered_fit']['n'])
                      for m,g in observations.items() if g['valid']}
            common_xy=None
            if label.startswith('floor') and all(m in supports for m in 'ABC'):
                common=np.ones(len(anchor),bool)
                for m in 'ABC':
                    common &= cKDTree(supports[m][:,:2]).query(anchor[:,:2])[0]<.28
                if np.any(common):
                    xy=anchor[common,:2]
                    common_xy=xy[np.argmin(np.linalg.norm(xy-np.median(xy,axis=0),axis=1))]
            for method,g in quality['surfaces'][label].items():
                obs=g['normal_filtered_fit']
                rec=dict(region=idx+1,target=i,source=j,surface=label,method=method,valid=g['valid'],
                                       reasons=g['reasons'],reference_points=len(anchor),source_points=obs.get('n'),
                                       reference_footprint=obs.get('target_footprint_coverage'),
                                       offset_m=obs.get('offset_m') if g['valid'] else None,
                                       thickness_p90_p10_m=obs.get('two_visit_thickness_p90_p10_m') if g['valid'] else None,
                                       angle_deg=obs.get('angle_deg') if g['valid'] else None)
                if g['valid']:
                    source_center=np.array(obs['source_plane_centroid_m'])
                    source_normal=np.array(obs['source_normal'])
                    rec.update(source_rms_to_target_plane_m=float(np.sqrt(np.mean(((supports[method]-c)@n)**2))),
                               source_own_fit_rms_m=float(np.sqrt(np.mean(((supports[method]-source_center)@source_normal)**2))))
                    if common_xy is not None and cKDTree(supports[method][:,:2]).query(common_xy)[0]<.28:
                        rec.update(common_xy_relative_m=common_xy.tolist(),vertical_height_difference_m=(
                            plane_height(common_xy,source_center,source_normal)-plane_height(common_xy,c,n)))
                planes_out.append(rec)
        # Freeze nearby, planar target neighbourhoods and source point identities using A.
        # This measures their change along the original local normal without renewing correspondences.
        d,near=all_nn['A']
        normals,planar=normal_at(old)
        xyz=old[tree.query(old,k=min(14,len(old)))[1]].mean(axis=1)
        use=(d<.32)&planar[near]
        for method,X in clouds.items():
            residual=np.abs(np.einsum('ij,ij->i',X[use]-xyz[near[use]],normals[near[use]]))
            d2,near2=all_nn[method]
            matched=(d2<.32)&planar[near2]
            renew=np.abs(np.einsum('ij,ij->i',X[matched]-xyz[near2[matched]],normals[near2[matched]]))
            local.append(dict(region=idx+1,target=i,source=j,method=method,**stats(residual),
                              renewed_n=int(matched.sum()),renewed_rms_m=stats(renew)['rms'],renewed_p95_m=stats(renew)['p95']))
        # Slice chosen from target data only, orthogonal to the largest target wall's horizontal tangent.
        walls=[p for p in groups if p[0].startswith('wall')]
        label,c,n,_,anchor=max(walls,key=lambda p:len(p[-1]))
        wall_normal=n[:2]/np.linalg.norm(n[:2]); tangent=np.array([-wall_normal[1],wall_normal[0]])
        slab=float(np.median(anchor[:,:2]@tangent))
        for ax,method in zip(axs[idx],'ABC'):
            for X,name in [(old,'target'),(clouds[method],'source')]:
                part=X[np.abs(X[:,:2]@tangent-slab)<=.35]
                ax.scatter(part[:,:2]@wall_normal,part[:,2],color=colors[name],s=4,alpha=.55,lw=0,label=name)
            ax.set_aspect('equal',adjustable='box')
            ax.grid(alpha=.15)
            ax.set_title(f'H{idx+1}   {i}/{j}   {method}',fontsize=10)
            if idx==0 and method=='A': ax.legend(fontsize=8)
            if method=='A': ax.set_ylabel('relative height [m]')
            if idx==4: ax.set_xlabel('wall-normal coordinate [m]')
        holdout.append(dict(region=idx+1,target=i,source=j,target_points=len(old),source_points=len(clouds['A']),
                            center=poses['A'][i,:3,3].tolist(),slice_target_wall=label,slice_tangent_xy=tangent.tolist(),
                            slice_center_m=slab,slice_halfwidth_m=.35))
    fig.savefig(HERE/'holdout_cross_sections.png',dpi=170);plt.close(fig)
    write_rows('holdout_bidirectional.csv',pp)
    write_rows('holdout_surfaces.csv',planes_out)
    write_rows('holdout_fixed_point_to_plane.csv',local)
    return holdout,pp,planes_out,local


def main(data=DEFAULT_DATA):
    for file,digest in read(HERE/'input_sha256.json').items():
        if sha(HERE/'raw'/file)!=digest:
            raise ValueError('local copy differs from source manifest: '+file)
    root=HERE/'raw'/RUN
    online=root/'pgo'
    index=root/'keyframes/index.csv'
    plan=read(data/'pgo/plan.json')
    assert sha(index)==plan['index_sha256']
    frames,G,_=frames_and_gravity(index,root/'gravity_events.jsonl')
    origin=int((HERE/'raw'/FULL/'origin_ns.txt').read_text())
    windows=np.loadtxt(HERE/'raw'/FULL/'static_windows.txt')
    stamps=np.array([f.stamp for f in frames],dtype=np.int64)
    times=(stamps-origin)*1e-9
    odom=np.array([f.odom for f in frames])
    def trajectory(path,expected=None):
        t,T=load_tum(path)
        if expected is not None: assert np.array_equal(t,expected)
        return np.asarray(t),T
    raw=G@odom
    _,A=trajectory(data/'pgo/trajectory_before.tum',stamps)
    np.testing.assert_allclose(raw,A,atol=3e-9)
    _,B=trajectory(data/'pgo/trajectory_after.tum',stamps)
    _,C=trajectory(data/'structure/trajectory_after.tum',stamps)
    _,retro=trajectory(online/'trajectory_after.tum',stamps)
    _,history=trajectory(online/'map_to_odom.tum',stamps)
    np.testing.assert_allclose(history@odom,retro,atol=3e-8)
    entries=list(csv.DictReader((online/'issued.csv').open()))
    assert np.array_equal([int(r['stamp_ns']) for r in entries],stamps)
    assert np.array_equal([int(r['id']) for r in entries],[f.id for f in frames])
    live=mmatrix(entries)
    issued=live@odom
    poses={'A':raw,'B':B,'C':C,'issued':issued,'online_final':retro}
    result=analyze_graph(online,live,times,raw,{'offline':B,'online':retro},issued,odom,windows,data)
    ns,Ad=trajectory(data/'pgo/trajectory_dense_before.tum')
    _,Bd=trajectory(data/'pgo/trajectory_dense_after.tum',ns)
    _,Cd=trajectory(data/'structure/trajectory_dense_after.tum',ns)
    dense_time=(ns-origin)*1e-9
    key_index=np.searchsorted(stamps,ns,side='right')-1
    assert min(key_index)>=0
    odom_xyz=transform(Ad[:,:3,3],rigid_inverse(G))
    issued_dense=np.einsum('nij,nj->ni',live[key_index,:3,:3],odom_xyz)+live[key_index,:3,3]
    clip=np.clip(dense_time,times[0],times[-1])
    rot=Slerp(times,Rotation.from_matrix(history[:,:3,:3]))(clip)
    retro_dense=rot.apply(odom_xyz)+np.column_stack([np.interp(clip,times,history[:,k,3]) for k in range(3)])
    dense=dict(A=Ad[:,:3,3],B=Bd[:,:3,3],C=Cd[:,:3,3],issued=issued_dense,online_final=retro_dense)
    result.update(keyframes=len(stamps),dense_count=len(ns),windows_s=windows.tolist(),
                  dense_span_s=dense_time[[0,-1]].tolist(),keyframe_span_s=times[[0,-1]].tolist(),
                  endpoints=displacements(ns,dense,poses,windows,origin))
    d=np.linalg.norm(issued_dense-retro_dense,axis=1)
    result['issued_vs_final_dense_m']=dict(**stats(d),max_sensor_s=float(dense_time[np.argmax(d)]))
    totalshift=np.linalg.norm(issued[:,:3,3]-raw[:,:3,3],axis=1)
    angle=np.rad2deg(Rotation.from_matrix(live[:,:3,:3]@G[:3,:3].T).magnitude())
    tf_norm=np.linalg.norm(live[:,:3,3],axis=1)
    result['issued_correction_relative_to_initial_gravity']=dict(
        same_pose_m=dict(**stats(totalshift),last=float(totalshift[-1]),max_id=int(np.argmax(totalshift))),
        angle_deg=dict(**stats(angle),last=float(angle[-1]),max_id=int(np.argmax(angle))),
        tf_translation_m=dict(**stats(tf_norm),last=float(tf_norm[-1]),max_id=int(np.argmax(tf_norm))))
    h,p,s,l=measure_regions(frames,poses,plan['holdout_pairs_indices'],data)
    result.update(holdouts=h,bidirectional=p,surfaces=s,point_plane=l)
    saves={}
    for d in ('pgo','structure'):
        for path in sorted((data/d).iterdir()):
            if path.suffix in {'.tum','.json','.jsonl','.csv'}:
                saves[path.relative_to(data).as_posix()]=sha(path)
    save('offline_input_sha256.json',saves)
    save('metrics.json', result)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,default=DEFAULT_DATA,help='Third-run directory containing pgo/ and structure/')
    main(parser.parse_args().data)
