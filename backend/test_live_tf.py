"""Deliver an archived index in write order and check the independent ROS /tf subscriber."""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
from frame_io import save_json


def listen(path,n):
    import rclpy
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node=rclpy.create_node("pgo_tf_independent_observer")
    observations=[]
    def got(msg):
        for m in msg.transforms:
            if (m.header.frame_id,m.child_frame_id)!=("map","odom"):
                continue
            p=m.transform.translation;q=m.transform.rotation;t=m.header.stamp
            observations.append([t.sec*10**9+t.nanosec,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    sub=node.create_subscription(TFMessage,"/tf",got,4096)
    deadline=time.monotonic()+600
    ready=path/"subscriber_ready"
    while len(observations)<n and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        if not ready.exists() and node.count_publishers("/tf")>0:
            ready.touch()
    with (path/"observed.csv").open("x",newline="") as f:
        wr=csv.writer(f);wr.writerow(["stamp_ns","tx","ty","tz","qx","qy","qz","qw"]);wr.writerows(observations)
    node.destroy_subscription(sub);node.destroy_node();rclpy.shutdown()
    if len(observations)!=n:
        raise ValueError(f"expected {n} TF samples, observed {len(observations)}")


def replay_and_verify(index,gravity,out,rebuild):
    out=out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    delivery=out/"input";delivery.mkdir()
    (delivery/"clouds").symlink_to((index.parent/"clouds").resolve(),target_is_directory=True)
    shutil.copy2(index.parent/"metadata.txt",delivery/"metadata.txt")
    shutil.copy2(gravity,delivery/"gravity_events.jsonl")
    with index.open("rb") as f:
        header=f.readline();entries=f.readlines()
    feed=delivery/"index.csv";feed.write_bytes(header)
    done=delivery/"writer_finished"
    observed=[sys.executable,__file__,"--listen",str(out),"--count",str(len(entries))]
    mapper=[sys.executable,str(Path(__file__).with_name("live_tf.py")),"--index",str(feed),
            "--gravity",str(delivery/"gravity_events.jsonl"),"--out",str(out/"publisher"),"--complete",str(done)]
    if rebuild:
        mapper.append("--rebuild")
    children=[]
    with (out/"subscriber.log").open("x") as observer_log,(out/"publisher.log").open("x") as pub_log:
        try:
            for cmd,log in ((observed,observer_log),(mapper,pub_log)):
                children.append(subprocess.Popen(cmd,stdout=log,stderr=log))
            until=time.monotonic()+35
            while not (out/"subscriber_ready").exists():
                if any(c.poll() is not None for c in children) or time.monotonic()>until:
                    raise RuntimeError("observer handshake failed; inspect node logs")
                time.sleep(.05)
            # Wait for both endpoints' DDS discovery after observing the matched
            # publisher. Each newline commits exactly one existing immutable PCD.
            time.sleep(1.)
            with feed.open("ab",buffering=0) as writer:
                for row in entries:
                    writer.write(row)
                    time.sleep(.012)
            done.write_text(json.dumps({"frames":len(entries)})+"\n")
            if children[1].wait(timeout=600) or children[0].wait(timeout=60):
                raise RuntimeError("publisher or observer failed; inspect separate logs")
        finally:
            for c in children:
                if c.poll() is None:
                    c.terminate()
                    c.wait(timeout=10)
    received=list(csv.DictReader((out/"observed.csv").open()))
    issued=list(csv.DictReader((out/"publisher/issued.csv").open()))
    assert len(received)==len(issued)==len(entries)
    expected=list(csv.DictReader(index.open()))
    xyzq=["tx","ty","tz","qx","qy","qz","qw"]
    worst=0.
    for a,b,c in zip(received,issued,expected):
        assert int(a["stamp_ns"])==int(b["stamp_ns"])==int(c["stamp_ns"])
        worst=max(worst,max(abs(float(a[v])-float(b[v])) for v in xyzq))
    assert worst<1e-9
    first=np.array([float(received[0][v]) for v in xyzq[:3]])
    last=np.array([float(received[-1][v]) for v in xyzq[:3]])
    tx=float(np.linalg.norm(last-first))
    assert int(issued[-1]["revision"])>0 and tx>.1
    sample_ids={0,len(entries)-1}
    for k in range(1,len(issued)):
        if issued[k]["revision"]!=issued[k-1]["revision"]:
            sample_ids.update([k-1,k])
    save_json(out/"wire_verification.json",{"verified":True,"messages":len(received),"distinct_tf_timestamps":len(set(r["stamp_ns"] for r in received)),
        "numeric_max_abs":worst,"first_to_last_correction_translation_m":tx,"first_tf":received[0],"last_tf":received[-1],
        "final_revision":int(issued[-1]["revision"]),"tf_at_revision_boundaries":[received[i] for i in sorted(sample_ids)],
        "ros_domain_id":os.getenv("ROS_DOMAIN_ID"),"interface":"/tf tf2_msgs/msg/TFMessage map -> odom",
        "input":"accelerated append-only delivery of archived rows, 12 ms pacing; no real-time performance claim"})


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--index",type=Path);p.add_argument("--gravity",type=Path);p.add_argument("--out",type=Path)
    p.add_argument("--listen",type=Path);p.add_argument("--count",type=int)
    p.add_argument("--rebuild",action="store_true")
    args=p.parse_args()
    if args.listen:
        listen(args.listen,args.count)
    else:
        replay_and_verify(args.index,args.gravity,args.out,args.rebuild)
