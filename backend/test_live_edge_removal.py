"""Regression for a proposed loop that disqualifies itself and an older active factor."""
import io
from pathlib import Path
from unittest.mock import patch

import numpy as np

from frame_io import Frame


def main():
    from live_tf import CausalGraph
    g=CausalGraph(np.eye(4),io.StringIO())
    for k,x in enumerate([0,10,20,10,0,10,20,10,0]):
        pose=np.eye(4);pose[0,3]=x
        g.frames.append(Frame(k,k*15*10**9,pose,Path("unused"),None))
        g.poses.append(pose);g.times.append(k*15.);g.track.append(k*10.)
    g.model={}
    g.correction[2,3]=3.
    g.edges=[{"target_id":3,"source_id":5,"target_index":3,"source_index":5,"accepted":True}]
    def estimate(i,j,kind):
        return {"target_id":i,"source_id":j,"target_index":i,"source_index":j,"kind":kind,"accepted":True,"reasons":[]}
    def discard_all(P,t,model,active):
        for e in active:
            e["accepted"]=False
        return P.copy(),[],{},[]
    with patch.object(g.builder,"estimate",estimate),patch("live_tf.screen",side_effect=lambda new,P,t:new),patch("live_tf.solve_screened",side_effect=discard_all):
        end=np.eye(4);end[0,3]=10.
        g.push(Frame(9,135*10**9,end,Path("unused"),None))
    assert g.edges==[] and g.revisions==1 and np.allclose(g.correction,np.eye(4))
    print("existing factor removal, active set, revision and TF correction verified")


if __name__=="__main__":
    main()
