"""Cross-check static update candidates against archived posterior states without re-reading the bag."""
from pathlib import Path
import hashlib
import json
import re

import numpy as np
import pandas as pd

OUT=Path(__file__).resolve().parent
BASE=OUT.parent
RAW=BASE/'raw'


def span(a):
    return dict(zip(('min','median','max'),map(float,np.quantile(a,[0,.5,1]))))


def bias(df):
    return df[['bgx','bgy','bgz']].to_numpy()


def main():
    state_path=RAW/'keyframes/states.csv'
    candidate_path=BASE/'static_updates.csv'
    states=pd.read_csv(state_path)
    lio=states[states.stage.eq('LIO')].reset_index(drop=True)
    vio=states[states.stage.str.startswith('VIO')].reset_index(drop=True)
    assert np.array_equal(lio.stamp_ns,vio.stamp_ns) and len(lio)*2==len(states)
    epochs=vio.stamp_ns.to_numpy()
    secs=(epochs-epochs[0])*1e-9
    assert np.all(np.diff(epochs)>0)
    # Transition index i covers the window after the previous VIO posterior at epochs[i].
    step=bias(lio)[1:]-bias(vio)[:-1]
    visual_step=bias(vio)[1:]-bias(lio)[1:]
    fields=['var_rx','var_ry','var_rz','var_x','var_y','var_z']
    cov_step=lio[fields].to_numpy()[1:]-vio[fields].to_numpy()[:-1]
    magnitudes=np.linalg.norm(step,axis=1)
    candidates=pd.read_csv(candidate_path)
    ids=np.array([np.argmin(np.abs(secs-t)) for t in candidates.t_s])
    assert np.max(np.abs(secs[ids]-candidates.t_s))<1e-6 and np.min(np.diff(ids))>20
    actual={}
    for path in [RAW/'algorithm.log.1',RAW/'algorithm.log']:
        for t,n in re.findall(r'static gyro update t=(\d+\.\d+) n=(\d+)',path.read_text(encoding='utf8',errors='replace')):
            actual[(float(t),int(n))]=path.name
    records=[]
    for (number,r),i in zip(candidates.iterrows(),ids):
        neigh=i+np.r_[np.arange(-10,0),np.arange(1,11)]
        assert len(set(neigh)&set(ids))==0 and neigh.min()>=0 and neigh.max()<len(step)
        innovation=np.array([r['omega_minus_bg_'+k] for k in 'xyz'])
        direction=innovation/np.linalg.norm(innovation)
        assert np.allclose(bias(vio)[i],[r['pre_bg_'+k] for k in 'xyz'],atol=1e-11)
        projected=float(step[i]@direction)
        log_matches=[loc for (ts,n),loc in actual.items() if abs(float(epochs[i])*1e-9-ts)<1.1e-6 and n==r.n]
        assert bool(log_matches)==r.log_match
        row=dict(number=number+1,candidate_t_s=secs[i],epoch_ns=int(epochs[i]),lio_posterior_ns=int(epochs[i+1]),
                 log_file=log_matches[0] if log_matches else '',imu_points=int(r.n),
                 control_n=len(neigh),control_before_s=secs[neigh[0]],control_after_s=secs[neigh[-1]],
                 norm=float(magnitudes[i]),innovation_norm=float(np.linalg.norm(innovation)),
                 projected_rad_s=projected,cosine=projected/magnitudes[i],
                 projected_fraction=projected/np.linalg.norm(innovation),
                 control_norm_p50=float(np.median(magnitudes[neigh])),control_norm_max=float(max(magnitudes[neigh])),
                 norm_over_control_median=magnitudes[i]/np.median(magnitudes[neigh]),
                 control_max_projection=float(max(step[neigh]@direction)),
                 norm_rank_wins=int(sum(magnitudes[i]>magnitudes[neigh])),
                 projection_rank_wins=int(sum(projected>step[neigh]@direction)),
                 next_visual_norm=float(np.linalg.norm(visual_step[i])))
        for j,k in enumerate('xyz'):
            row['innovation_'+k]=innovation[j]
            row['observed_lio_step_'+k]=step[i,j]
            row['observed_vio_step_'+k]=visual_step[i,j]
        for j,k in enumerate(fields):
            row[k+'_step']=cov_step[i,j]
        row.update(rotation_variance_trace_step=sum(cov_step[i,:3]),
                   rotation_variance_neighbor_min=float(np.min(cov_step[neigh,:3].sum(axis=1))),
                   rotation_variance_neighbor_max=float(np.max(cov_step[neigh,:3].sum(axis=1))))
        records.append(row)
    frame=pd.DataFrame(records)
    frame.to_csv(OUT/'state_step_evidence.csv',index=False,float_format='%.12g')
    summary={}
    for group,g in [('early_no_direct_log',frame[frame.log_file=='']),('late_logged',frame[frame.log_file!=''])]:
        assert g.norm_rank_wins.min()==20 and g.projection_rank_wins.min()==20
        summary[group]=dict(n=len(g),bound_s=[g.candidate_t_s.iloc[0],g.candidate_t_s.iloc[-1]],both_strictly_greater=int(
            sum((g.norm_rank_wins==20)&(g.projection_rank_wins==20))),norm_over_neighbor_median=span(g.norm_over_control_median),
            cosine=span(g.cosine),projected_fraction=span(g.projected_fraction),amplitude_rad_s=span(g.norm),
            next_visual_amplitude_rad_s=span(g.next_visual_norm),
            rotation_variance_trace_within_neighbor_range=int(sum((g.rotation_variance_trace_step>=g.rotation_variance_neighbor_min)&
                                                                  (g.rotation_variance_trace_step<=g.rotation_variance_neighbor_max))))
    # Independent early-interval ranking by LIO bias jump magnitude alone.
    cutoff=secs[ids[30]]+1
    prefix=np.flatnonzero(secs[:-1]<=cutoff)
    top=prefix[np.argsort(magnitudes[prefix])[-31:]]
    summary['early_blind_rank']=dict(interval_s=[0,float(cutoff)],steps=len(prefix),top31_hits=int(sum(np.isin(ids[:31],top))),
                                    candidate_ids_outside_top31=list(map(int,np.flatnonzero(~np.isin(ids[:31],top))+1)))
    cpath=RAW/'sensors/camera/clock_latch.csv'
    c=pd.read_csv(cpath)
    ticks=c.device_timestamp_value.to_numpy()
    mid=(c.monotonic_before_ns-c.monotonic_before_ns.iloc[0]+(c.monotonic_after_ns-c.monotonic_before_ns)/2).to_numpy()
    slope0=np.median(np.diff(mid)/np.diff(ticks))
    resid=mid-slope0*(ticks-ticks[0])
    valid=np.abs(resid-np.median(resid))<1e8
    bad=np.flatnonzero(~valid)
    assert len(bad)==1
    i=int(bad[0])
    slope,intercept=np.polyfit((ticks-ticks[0])[valid],mid[valid],1)
    expected=ticks[i-1]+(mid[i]-mid[i-1])/(mid[i+1]-mid[i-1])*(ticks[i+1]-ticks[i-1])
    field_values=json.loads((RAW/'sensors/camera/parameters.active.json').read_text())['parameters']
    summary['camera_latch']=dict(rows=len(c),outlier_ordinal_zero_based=i,query_return_codes=[int(c.latch_rc.iloc[i]),int(c.read_rc.iloc[i])],
                                neighbors_raw_ticks=[int(t) for t in ticks[i-1:i+2]],neighbors_hex=[hex(int(t)) for t in ticks[i-1:i+2]],
                                expected_minus_observed_ticks=float(expected-ticks[i]),difference_to_2pow32_ticks=float((2**32)-(expected-ticks[i])),
                                fit_ns_per_raw_count=float(slope),fit_frequency_mhz=float(1000/slope),
                                observed_fit_residual_ms=float((mid[i]-(slope*(ticks[i]-ticks[0])+intercept))*1e-6),
                                timestamp_frequency_readback={k:field_values[k] for k in ('TimestampTickFrequency','GevTimestampTickFrequency')})
    summary['input_sha256']={p.relative_to(BASE).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in
                             [state_path,candidate_path,cpath,RAW/'algorithm.log',RAW/'algorithm.log.1']}
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


if __name__=='__main__':
    main()
