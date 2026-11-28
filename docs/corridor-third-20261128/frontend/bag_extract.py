"""Stream bounded CDR fields from a closed ROS2 SQLite bag. Run on its host.
python bag_extract.py BAG > clocks.csv.gz  (stdout is binary)
Codes: I image, C camera info, L scan, U IMU, M metadata, E event, T latch.
Each row starts code,shard,rowid,record_ns. Source columns are in SCHEMA.
"""
import csv
import ctypes as ct
import ctypes.util
import gzip
import io
import json
from pathlib import Path
import sqlite3
import struct
import sys

NAMES = {"/hik_camera/image_timeshare":"I","/hik_camera/camera_info_timeshare":"C",
         "/livox/lidar":"L","/livox/imu":"U","/hardware_sync/camera_metadata":"M",
         "/hardware_sync/camera_events":"E","/hardware_sync/camera_clock":"T"}
SCHEMA = {"I":["header_ns"],"C":["header_ns"],"L":["header_ns"],
          "U":["header_ns","wx","wy","wz","ax","ay","az"],
          "M":["frame_num","device_timestamp_raw","host_after_get_ns","host_monotonic_after_get_ns",
               "publish_done_ns","lost_packets","exposure_frame_us","gain_frame","trigger_index","frame_counter"],
          "E":["event_name","event_device_timestamp_raw","host_realtime_ns","host_monotonic_ns","block_id_raw"],
          "T":["monotonic_before_ns","monotonic_after_ns","device_timestamp_value","latch_rc","read_rc"]}

def check(code):
    if code!=0: raise RuntimeError("SQLite rc "+str(code))
def run(bag):
    lib=ct.CDLL(ctypes.util.find_library("sqlite3"))
    paths=sorted(bag.glob("*.db3"),key=lambda p:int(p.stem.split("_")[-1]))
    if not paths: raise ValueError("No shards")
    with gzip.GzipFile(fileobj=sys.stdout.buffer,mode="wb",compresslevel=1,mtime=0) as out, io.TextIOWrapper(out,encoding="utf-8",newline="") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        for path in paths:
            shard=int(path.stem.split("_")[-1])
            uri=path.resolve().as_uri()+"?mode=ro&immutable=1"
            db=sqlite3.connect(uri,uri=True)
            handle,blob=ct.c_void_p(),ct.c_void_p()
            check(lib.sqlite3_open_v2(uri.encode(),ct.byref(handle),0x41,None))
            buf=ct.create_string_buffer(2048)
            try:
                topics={k:NAMES[v] for k,v in db.execute("select id,name from topics")}
                for rid,tid,b in db.execute("select id,topic_id,timestamp from messages order by id"):
                    typ=topics[tid]
                    if blob:
                        check(lib.sqlite3_blob_reopen(blob,ct.c_int64(rid)))
                    else:
                        check(lib.sqlite3_blob_open(handle,b"main",b"messages",b"data",ct.c_int64(rid),0,ct.byref(blob)))
                    size=lib.sqlite3_blob_bytes(blob)
                    n=12 if typ in ("I","C","L") else size
                    if n>len(buf): raise ValueError((typ,size))
                    check(lib.sqlite3_blob_read(blob,buf,n,0))
                    raw=buf.raw[:n]
                    if raw[:4]!=b"\x00\x01\x00\x00": raise ValueError("CDR LE encoding")
                    if typ in ("I","C","L","U"):
                        s,ns=struct.unpack_from("<iI",raw,4)
                        if ns>=10**9: raise ValueError("bad nanoseconds")
                        fields=[s*10**9+ns]
                        if typ=="U":
                            length=struct.unpack_from("<I",raw,12)[0]
                            start=4+((12+length+7)//8)*8
                            fields.extend(struct.unpack_from("<3d",raw,start+104))
                            fields.extend(struct.unpack_from("<3d",raw,start+200))
                    else:
                        length=struct.unpack_from("<I",raw,4)[0]
                        if length>n-8 or raw[8+length-1]!=0: raise ValueError("Invalid String")
                        obj=json.loads(raw[8:8+length-1])
                        fields=[obj[k] for k in SCHEMA[typ]]
                    writer.writerow([typ,shard,rid,b,*fields])
            finally:
                if blob: check(lib.sqlite3_blob_close(blob))
                check(lib.sqlite3_close(handle))
                db.close()

if __name__=="__main__": run(Path(sys.argv[1]))
