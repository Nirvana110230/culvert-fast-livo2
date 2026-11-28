"""Measure travel activity on the original dense LIO track in sensor seconds."""
from argparse import ArgumentParser
from pathlib import Path
import json
import numpy as np
import pandas as pd


def main():
    p=ArgumentParser(description=__doc__)
    p.add_argument("pgo",type=Path)
    p.add_argument("out",type=Path)
    a=p.parse_args()
    raw=np.loadtxt(a.pgo/"trajectory_dense_before.tum",dtype=str)
    stamps=np.array([int(x.replace(".","")) for x in raw[:,0]],dtype=np.int64)
    t=(stamps-stamps[0])*1e-9
    xyz=raw[:,1:4].astype(float)
    edges=np.r_[np.arange(0,t[-1],1),t[-1]]
    coords=np.column_stack([np.interp(edges,t,c) for c in xyz.T])
    dt=np.diff(edges)
    steps=np.linalg.norm(np.diff(coords,axis=0),axis=1)
    speed=steps/dt
    table=pd.DataFrame(dict(lo_s=edges[:-1],hi_s=edges[1:],step_m=steps,speed_m_s=speed))
    table.to_csv(a.out/"motion_windows.csv",index=False,float_format="%.8f")
    stats=[]
    for gate in (.02,.05,.10,.15):
        active=speed>gate
        stats.append(dict(threshold_m_s=gate,active_seconds=float(dt[active].sum()),
             first_active_s=float(edges[:-1][active][0]),last_active_end_s=float(edges[1:][active][-1]),
             length_when_active_m=float(steps[active].sum()),length_all_m=float(steps.sum())))
    (a.out/"motion_metrics.json").write_text(json.dumps(stats,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
