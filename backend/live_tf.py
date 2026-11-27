"""Causal PGO TF publisher for an append-only frontend keyframe index."""
import argparse
import copy
import csv
import json
from pathlib import Path
import resource
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from rclpy.signals import SignalHandlerOptions
from frame_io import frame_from_row, rebuild, rigid_inverse, save_json, session_gravity, write_tum
from loop_edges import EdgeBuilder, screen
from noise import calibrate
from run import optimize, solve_screened


class CausalGraph:
    def __init__(self,G,events):
        self.G=G
        self.correction=G.copy()
        self.frames=[]
        self.poses=[]
        self.times=[]
        self.track=[]
        self.builder=EdgeBuilder(self.frames,[],[],set())
        self.edges=[]
        self.model=None
        self.last_query=0
        self.revisions=0
        self.events=events
        self.graph_seconds=[]

    def push(self,frame):
        if self.frames and (frame.stamp<=self.frames[-1].stamp or frame.id<=self.frames[-1].id):
            raise ValueError("out-of-order or duplicate committed keyframe")
        self.frames.append(frame)
        self.poses.append(self.G@frame.odom)
        self.times.append((frame.stamp-self.frames[0].stamp)*1e-9)
        n=len(self.frames)
        self.track.append(0. if n==1 else self.track[-1]+np.linalg.norm(self.poses[-1][:3,3]-self.poses[-2][:3,3]))
        # j+2 must already exist to validate against disjoint neighbouring scans.
        j=n-3
        if j<2 or self.times[j]<75 or self.track[j]<18 or self.times[j]-self.times[self.last_query]<15:
            return
        self.last_query=j
        P=np.array(self.poses)
        self.builder.poses=P
        t=np.array(self.times)
        distance=np.linalg.norm(P[:j,:3,3]-P[j,:3,3],axis=1)
        candidates=np.flatnonzero((distance<=5)&(t[j]-t[:j]>=75)&(self.track[j]-np.array(self.track[:j])>=18))
        if not len(candidates):
            return
        representatives=[]
        for k in candidates[np.argsort(distance[candidates],kind="stable")]:
            if all(abs(t[k]-t[i])>=75 for i in representatives):
                representatives.append(int(k))
            if len(representatives)==2:
                break
        started=time.monotonic()
        if self.model is None:
            self.model=calibrate(self.builder,P,t)
        new=[self.builder.estimate(i,j,"automatic") for i in representatives]
        screened=screen(new,P,t)
        tested=[]
        for e in screened:
            i=e["target_index"]
            if any(abs(t[j]-t[v["source_index"]])<20 and abs(t[i]-t[v["target_index"]])<20 for v in self.edges):
                e["accepted"]=False;e["reasons"].append("overlapping_accepted_constraint")
            else:
                tested.append(e)
        if tested:
            active=copy.deepcopy(self.edges)+tested
            after,_,_,updated=solve_screened(P,t,self.model,active)
            old_keys={(e["target_id"],e["source_id"]) for e in self.edges}
            new_keys={(e["target_id"],e["source_id"]) for e in updated}
            if old_keys!=new_keys:
                self.correction=after[-1]@rigid_inverse(frame.odom)
                self.edges=updated
                self.revisions+=1
        elapsed=time.monotonic()-started
        self.graph_seconds.append(elapsed)
        self.events.write(json.dumps({"available_through_id":frame.id,"available_through_stamp_ns":frame.stamp,
                                      "candidate_source_id":self.frames[j].id,"revision":self.revisions,
                                      "active_loops":len(self.edges),"active_ids":[[e["target_id"],e["source_id"]] for e in self.edges],
                                      "elapsed_s":elapsed,"candidates":new},allow_nan=False)+"\n")
        self.events.flush()


def publish(node,broadcaster,frame,T,parent,child):
    msg=TransformStamped()
    msg.header.frame_id=parent;msg.child_frame_id=child
    msg.header.stamp.sec=frame.stamp//10**9;msg.header.stamp.nanosec=frame.stamp%10**9
    tr=msg.transform.translation
    tr.x,tr.y,tr.z=map(float,T[:3,3])
    q=Rotation.from_matrix(T[:3,:3]).as_quat()
    rot=msg.transform.rotation
    rot.x,rot.y,rot.z,rot.w=map(float,q)
    broadcaster.sendTransform(msg)
    return [*T[:3,3],*q]


def rebuild_final(graph,out):
    start=time.monotonic()
    before=np.array(graph.poses)
    if graph.edges:
        after,factors,solver=optimize(before,np.array(graph.times),graph.model,graph.edges)
        save_json(out/"factors.json",factors)
    else:
        after=before.copy()
        solver={"loop_count":0,"solution":"raw local odometry with session gravity"}
    write_tum(out/"trajectory_before.tum",graph.frames,before)
    write_tum(out/"trajectory_after.tum",graph.frames,after)
    history=[pose@rigid_inverse(f.odom) for pose,f in zip(after,graph.frames)]
    write_tum(out/"map_to_odom.tum",graph.frames,history)
    save_json(out/"edges.json",graph.edges)
    maps=rebuild(out,graph.frames,before,after)
    summary={"keyframes":len(graph.frames),"maps":maps,"graph":solver,"elapsed_s":time.monotonic()-start,
             "source":"causal graph final accepted factors, fixed; full-history batch map reconstruction",
             "accepted":[{"target_id":e["target_id"],"source_id":e["source_id"]} for e in graph.edges],
             "gravity_frame":"map_gravity","map_to_odom":"final retrospective trajectory; issued.csv retains actual broadcast history"}
    save_json(out/"summary.json",summary)
    return {"status":"complete","frames":len(graph.frames),"elapsed_s":summary["elapsed_s"]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("index","gravity","out"):
        p.add_argument("--"+key,type=Path,required=True)
    p.add_argument("--parent",default="map")
    p.add_argument("--child",default="odom",help="must be an alias of the session camera_init axes and origin")
    p.add_argument("--complete",type=Path,help="optional producer-finished marker; drain committed rows then exit")
    p.add_argument("--rebuild",action="store_true",help="rebuild before/after historical maps after a normal producer EOF")
    args=p.parse_args()
    if args.parent==args.child:
        p.error("TF parent and child must differ")
    args.out.mkdir(exist_ok=False,parents=True)
    started=time.monotonic()
    stopped=False
    def request_stop(signum,_):
        nonlocal stopped
        stopped=True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGTERM,request_stop)
    node=rclpy.create_node("livo_pgo_sidecar")
    broadcaster=TransformBroadcaster(node)
    try:
        while rclpy.ok() and not stopped:
            if args.index.is_file() and args.gravity.is_file() and args.gravity.stat().st_size:
                if args.gravity.read_bytes().endswith(b"\n"):
                    break
            if args.complete and args.complete.is_file():
                marker=args.complete.read_text().strip()
                count=json.loads(marker).get("frames") if marker.startswith("{") else None
                status="no_initialized_keyframes" if count in (None,0) else "incomplete_session"
                save_json(args.out/"online_summary.json",{"status":status,"published":0,"producer_frames":count,
                                                         "rebuild":{"status":"no_input"}})
                if status=="incomplete_session":
                    raise ValueError("producer finished with frames but without a complete index/gravity event")
                return
            rclpy.spin_once(node,timeout_sec=.05)
        if not rclpy.ok() or stopped:
            save_json(args.out/"online_summary.json",{"status":"interrupted","published":0,"rebuild":{"status":"no_input"}})
            return
        G,record=session_gravity(args.index,args.gravity)
        with args.index.open("r",encoding="utf-8-sig",newline="") as inp,(args.out/"issued.csv").open("x",newline="") as log, (args.out/"graph.jsonl").open("x") as ge:
            graph=CausalGraph(G,ge)
            writer=csv.writer(log);writer.writerow(["id","stamp_ns","revision","active_loops","tx","ty","tz","qx","qy","qz","qw"])
            header=None
            finished=False
            while rclpy.ok() and not stopped:
                pos=inp.tell()
                line=inp.readline()
                if not line.endswith("\n"):
                    inp.seek(pos)
                    if args.complete and args.complete.is_file():
                        # The producer writes the terminal marker after its
                        # last full row; re-read after observing the marker.
                        final=inp.readline()
                        if final.endswith("\n"):
                            inp.seek(pos);continue
                        if final:
                            raise ValueError("incomplete final CSV record")
                        marker=args.complete.read_text().strip()
                        if marker.startswith("{") and json.loads(marker).get("frames",len(graph.frames))!=len(graph.frames):
                            raise ValueError("producer frame count differs from drained keyframe count")
                        finished=True
                        break
                    rclpy.spin_once(node,timeout_sec=.04)
                    continue
                values=next(csv.reader([line]))
                if header is None:
                    header=values
                    continue
                if len(values)!=len(header):
                    raise ValueError("broken committed index row")
                frame=frame_from_row(args.index,dict(zip(header,values)))
                if not frame.cloud.is_file():
                    raise FileNotFoundError(frame.cloud)
                graph.push(frame)
                fields=publish(node,broadcaster,frame,graph.correction,args.parent,args.child)
                writer.writerow([frame.id,frame.stamp,graph.revisions,len(graph.edges),*fields])
                log.flush()
                rclpy.spin_once(node,timeout_sec=0)
            elapsed=time.monotonic()-started
            rebuilt={"status":"not_requested" if not args.rebuild else "no_input" if not graph.frames else "interrupted"}
            if finished and args.rebuild and graph.frames:
                rebuilt=rebuild_final(graph,args.out)
            save_json(args.out/"online_summary.json",{"status":"complete" if finished else "interrupted",
                      "rebuild":rebuilt,"published":len(graph.frames),"revisions":graph.revisions,
                      "active_edges":[[e["target_id"],e["source_id"]] for e in graph.edges],"session":record,
                      "parent":args.parent,"child":args.child,"stamp":"source estimator measurement, integer ns",
                      "elapsed_s":elapsed,"query_wall_s":graph.graph_seconds,
                      "peak_rss_MiB":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024})
            if graph.model:
                save_json(args.out/"causal_noise.json",graph.model)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
