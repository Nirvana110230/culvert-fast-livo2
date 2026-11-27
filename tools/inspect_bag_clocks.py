"""Read original ROS 2 SQLite shards without materializing image or lidar payloads."""
import argparse
import bisect
import csv
import ctypes as ct
import ctypes.util
import json
import math
import sqlite3
import statistics
import struct
from pathlib import Path


def distribution(values):
    a = sorted(values)
    return None if not a else {"n":len(a),"min":a[0],"median":statistics.median(a),
                               "p95":a[math.ceil(.95*len(a))-1],"p99":a[math.ceil(.99*len(a))-1],"max":a[-1]}


class PrefixReader:
    """sqlite3_blob_read requests twelve bytes even when the ROS image is 3.75 MiB."""
    dbapi = ct.CDLL(ctypes.util.find_library("sqlite3"))
    def __init__(self, uri):
        self.db = ct.c_void_p()
        self.blob = ct.c_void_p()
        self.buf = ct.create_string_buffer(12)
        self.check(self.dbapi.sqlite3_open_v2(uri.encode(),ct.byref(self.db),0x41,None))
    def check(self, rc):
        if rc:
            raise RuntimeError("SQLite incremental read failed: "+str(rc))
    def stamp(self, row):
        if self.blob:
            self.check(self.dbapi.sqlite3_blob_reopen(self.blob,ct.c_int64(row)))
        else:
            self.check(self.dbapi.sqlite3_blob_open(self.db,b"main",b"messages",b"data",
                                                   ct.c_int64(row),0,ct.byref(self.blob)))
        self.check(self.dbapi.sqlite3_blob_read(self.blob,self.buf,12,0))
        data = self.buf.raw
        if data[:2] != b"\x00\x01":
            raise ValueError("non little-endian CDR1 serialization")
        s,ns = struct.unpack_from("<iI",data,4)
        if ns >= 10**9:
            raise ValueError("invalid CDR1 time")
        return s*10**9+ns
    def close(self):
        if self.blob:
            self.check(self.dbapi.sqlite3_blob_close(self.blob))
        self.check(self.dbapi.sqlite3_close(self.db))


def topic_stats(records, low_ms, high_ms):
    deltas = [(b[0]-a[0])/1e6 for a,b in zip(records, records[1:])]
    arrivals = [(b[1]-a[1])/1e6 for a,b in zip(records,records[1:])]
    def item(i):
        r=records[i+1]
        return {"previous_header_ns":records[i][0],"header_ns":r[0],"bag_ns":r[1],
                "header_dt_ms":deltas[i],"bag_dt_ms":arrivals[i],"shard":r[2],"rowid":r[3]}
    return {"messages":len(records),"header_dt_ms":distribution(deltas),"bag_dt_ms":distribution(arrivals),
            "header_duplicates_adjacent":deltas.count(0),"header_backsteps":sum(t<0 for t in deltas),
            "header_lt_threshold":sum(t<low_ms for t in deltas),"header_gt_threshold":sum(t>high_ms for t in deltas),
            "thresholds_ms":[low_ms,high_ms],"ten_largest_header_gaps":[item(i) for i in sorted(range(len(deltas)),key=deltas.__getitem__,reverse=True)[:10]],
            "largest_bag_gap":item(max(range(len(arrivals)),key=arrivals.__getitem__))}


def inspect_shared_reads(camera, anomaly_frames):
    def load(files,key):
        data={}
        for path in files:
            with path.open(newline="") as stream:
                for row in csv.DictReader(stream):
                    identity=int(row[key])
                    if identity in data: raise ValueError(f"duplicate {key} in camera timing trace")
                    data[identity]={k:int(v) for k,v in row.items()}
        return data
    writer=load(camera.parent.glob("timeshare_writes.csv*"),"scan_sequence")
    reader=load(camera.glob("timeshare_reads.csv*"),"frame_num")
    if not writer or not reader:
        return {"available":False}
    ordered=sorted(writer.values(),key=lambda w:w["write_after_mono_ns"])
    ends=[w["write_after_mono_ns"] for w in ordered]
    near={n+dx for n in anomaly_frames for dx in range(-1,2)}
    confirmed=overlap=mismatch=outside=0; samples=[]; concurrent=[]
    for frame,row in sorted(reader.items()):
        rb=row["read_before_mono_ns"]; re=row["read_after_mono_ns"]
        i=bisect.bisect_right(ends,rb)-1
        if i<0 or i+1>=len(ordered):
            outside+=1;continue
        left,right=ordered[i:i+2]
        if right["write_before_mono_ns"]<=re:
            overlap+=1
            concurrent.append({"frame":frame,"read_mono_ns":[rb,re],"overlapping_write_mono_ns":[
                right["write_before_mono_ns"],right["write_after_mono_ns"]],"previous_sequence":left["scan_sequence"],
                "overlapping_sequence":right["scan_sequence"],"read_value_matches_previous":row["shared_stamp_ns"]==left["shared_stamp_ns"],
                "read_value_matches_overlapping":row["shared_stamp_ns"]==right["shared_stamp_ns"]})
            continue
        agree=left["shared_stamp_ns"]==row["image_header_ns"]==row["shared_stamp_ns"]
        confirmed+=agree; mismatch+=not agree
        if frame in near:
            samples.append({"frame":frame,"image_header_ns":row["image_header_ns"],"last_completed_write_sequence":left["scan_sequence"],
                            "reader_stamp_matches_writer":agree,"read_after_last_write_ms":(rb-left["write_after_mono_ns"])/1e6,
                            "next_write_after_read_ms":(right["write_before_mono_ns"]-re)/1e6})
    return {"available":True,"writes":len(writer),"reads_retained":len(reader),"unambiguous_matching":confirmed,
            "unambiguous_mismatches":mismatch,"overlapping_read_write_intervals":overlap,"unbracketed":outside,
            "concurrent_reads":concurrent,
            "missing_direct_trace_frames":[f for f in anomaly_frames if f not in reader],
            "examples_around_actual_abnormal_headers":samples}


def run(bag, camera):
    names=["/hik_camera/image_timeshare","/livox/lidar","/livox/imu","/hardware_sync/camera_metadata"]
    records={name:[] for name in names[:3]}
    metadata=[]
    shards=sorted(bag.glob("*.db3"),key=lambda p:int(p.stem.rsplit("_",1)[1]))
    for shard,f in enumerate(shards):
        uri=f.resolve().as_uri()+"?mode=ro&immutable=1"
        db=sqlite3.connect(uri,uri=True); reader=PrefixReader(uri)
        try:
            ids=dict(db.execute("SELECT id,name FROM topics"))
            for row,topic,bagtime in db.execute("SELECT id,topic_id,timestamp FROM messages ORDER BY id"):
                n=ids[topic]
                if n in records:
                    records[n].append((reader.stamp(row),bagtime,shard,row))
                elif n==names[3]:
                    data=db.execute("SELECT data FROM messages WHERE id=?",(row,)).fetchone()[0]
                    if data[:2]!=b"\x00\x01": raise ValueError("invalid ROS String encoding")
                    size=struct.unpack_from("<I",data,4)[0]
                    if data[8+size-1]!=0: raise ValueError("invalid ROS String size")
                    meta=json.loads(data[8:8+size-1]); meta["bag_ns"]=bagtime
                    metadata.append(meta)
        finally:
            reader.close(); db.close()
    result={"shards":len(shards),"read_mode":"SQLite immutable read-only, CDR1 integer Header, ROS String metadata",
            "streams":{n:topic_stats(records[n],*( (2.5,7.5) if n==names[2] else (50,150))) for n in names[:3]}}
    images=records[names[0]]; lidar_times={r[0] for r in records[names[1]]}
    frames={int(m["frame_num"]):m for m in metadata}
    ordered=sorted(frames)
    successive=list(zip(ordered,ordered[1:]))
    def sample(m):
        return {"frame_num":m["frame_num"],"host_after_get_ns":m["host_after_get_ns"],
                "host_publish_done_ns":m["publish_done_ns"],"device_timestamp_raw":m["device_timestamp_raw"],
                "get_to_publish_ms":(m["publish_done_ns"]-m["host_after_get_ns"])/1e6}
    delays=[sample(m)["get_to_publish_ms"] for m in metadata]
    trace={}
    for f in sorted(camera.glob("timeshare_reads.csv*")):
        with f.open(newline="") as source:
            for r in csv.DictReader(source):
                k=int(r["frame_num"])
                if k in trace: raise ValueError("duplicate timeshare trace frame")
                trace[k]=int(r["image_header_ns"])
    # Verify proposed ordinal pairing on EVERY camera-driver trace row available,
    # including duplicate stamps. Frame gaps or missing metadata disable this join.
    ordinal=len(images)==len(metadata) and len(frames)==len(metadata) and all(
        m["frame_num"]==ordered[0]+i for i,m in enumerate(metadata))
    mismatches=sum(images[k-ordered[0]][0]!=v for k,v in trace.items()
                   if ordinal and 0<=k-ordered[0]<len(images))
    covered=sum(0<=k-ordered[0]<len(images) for k in trace)
    joined=ordinal and covered>0 and mismatches==0
    details=[]
    for i in range(1,len(images)):
        prev,cur=images[i-1],images[i]; dt=(cur[0]-prev[0])/1e6
        if 50 <= dt <= 150: continue
        d={"shard":cur[2],"rowid":cur[3],"image_header_ns":cur[0],"previous_header_ns":prev[0],
           "dt_ms":dt,"bag_interval_ms":(cur[1]-prev[1])/1e6}
        if joined:
            m=metadata[i]; old=metadata[i-1]
            d.update(sample(m)); d.update({"previous_frame":old["frame_num"],"previous_get_to_publish_ms":sample(old)["get_to_publish_ms"],
                "sdk_get_mono_interval_ms":(m["host_monotonic_after_get_ns"]-old["host_monotonic_after_get_ns"])/1e6,
                "device_interval_ticks":m["device_timestamp_raw"]-old["device_timestamp_raw"],
                "frame_has_direct_trace":m["frame_num"] in trace})
        details.append(d)
    result["camera"]={"metadata":len(metadata),"unique_frame_nums":len(ordered),"first_last_frames":[ordered[0],ordered[-1]],
        "unreported_frame_ids":sum(b-a-1 for a,b in successive),"lost_packets_total":sum(m["lost_packets"] for m in metadata),
        "shared_image_headers_matching_lidar":sum(t in lidar_times for t,*_ in images),"direct_timeshare_trace_rows_in_range":covered,
        "ordinal_joint_validated_on_trace":joined,"trace_header_mismatch":mismatches if ordinal else None,
        "sdk_get_to_publish_done_ms":distribution(delays),"get_to_publish_gt_100ms":sum(t>100 for t in delays),
        "largest_publish_delays":[sample(m) for m in sorted(metadata,key=lambda m:m["publish_done_ns"]-m["host_after_get_ns"],reverse=True)[:10]],
        "sdk_frame_device_interval_ticks":distribution(frames[b]["device_timestamp_raw"]-frames[a]["device_timestamp_raw"] for a,b in successive),
        "device_timestamp_frequency":"unconverted_ticks","abnormal_shared_header_intervals":details}
    result["shared_clock_trace"]=inspect_shared_reads(camera,[d["frame_num"] for d in details if "frame_num" in d])
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("bag",type=Path); p.add_argument("camera_logs",type=Path)
    args=p.parse_args(); print(json.dumps(run(args.bag,args.camera_logs),indent=2,allow_nan=False))
