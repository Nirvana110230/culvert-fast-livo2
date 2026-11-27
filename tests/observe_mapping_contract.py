"""Read-only wire check for the sensor-time navigation source and the PGO TF."""
import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, help="one new replay in this directory")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    seen = {name: set() for name in ("state", "scan", "tf")}
    incorrect = []
    def record(name, header, expected):
        if header.frame_id != expected:
            incorrect.append([name, header.frame_id])
        seen[name].add(header.stamp.sec*10**9+header.stamp.nanosec)
    def state(msg):
        record("state",msg.header,"camera_init")
        if msg.child_frame_id != "imu":
            incorrect.append(["state_child",msg.child_frame_id])
    def scan(msg):
        record("scan",msg.header,"livo_lidar")
        if msg.width*msg.height == 0 or {s.name for s in msg.fields} != {"x","y","z","intensity"}:
            incorrect.append(["cloud",msg.width])
    def transforms(msg):
        for tf in msg.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                record("tf",tf.header,"map")
    rclpy.init()
    node = rclpy.create_node("corridor_mapping_wire_check")
    node.create_subscription(Odometry,"/culvert_source/state",state,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,"/culvert_source/scan",scan,qos_profile_sensor_data)
    node.create_subscription(TFMessage,"/tf",transforms,100)
    done = None
    last_check = 0
    run = None
    while rclpy.ok() and (done is None or time.monotonic()-done < 2):
        rclpy.spin_once(node,timeout_sec=.1)
        if done is None and time.monotonic()-last_check >= 1:
            last_check = time.monotonic()
            for file in args.runs.glob("*/session.json"):
                if json.loads(file.read_text())["state"] in {"complete","failed"}:
                    done, run = time.monotonic(), file.parent
    node.destroy_node()
    rclpy.shutdown()
    if run is None:
        raise SystemExit("No completed replay")
    with (run/"keyframes/index.csv").open() as f:
        index = {int(row["stamp_ns"]) for row in csv.DictReader(f)}
    pairs = seen["state"] & seen["scan"]
    report = dict(run=str(run), counts={k:len(v) for k,v in seen.items()}, same_stamp_pairs=len(pairs),
                  keyframes=len(index), keyframes_with_full_pair=len(index & pairs),
                  keyframes_with_tf=len(index & seen["tf"]), foreign_tf_stamps=len(seen["tf"]-index),
                  wrong_frame_or_layout=incorrect[:10])
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(report)
    if incorrect or not (index & pairs & seen["tf"]) or seen["tf"]-index:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
