"""Image-domain measurements on every recorded B1 RGB frame, keyed by the input header."""
import argparse
import csv
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/hik_camera/image_timeshare")
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    with args.output.open("x", newline="") as out:
        rows = csv.writer(out)
        rows.writerow(["image_header_ns", "record_ns", "mean_luma", "mean_abs_gradient_1px", "dark_fraction_lt16"])
        while reader.has_next():
            _, raw, record = reader.read_next()
            msg = deserialize_message(raw, Image)
            if msg.encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"Expected 8-bit RGB camera, found {msg.encoding}")
            rgb = np.ndarray((msg.height, msg.width, 3), np.uint8, buffer=msg.data,
                             strides=(msg.step, 3, 1))
            if msg.encoding == "bgr8":
                rgb = rgb[:, :, ::-1]
            # Spatially distributed pixel pairs, with one-pixel differences at native resolution.
            # Each image is measured, independent of callback acceptance or visual tracking.
            def luma(a):
                return a @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
            centre = luma(rgb[:-1:12, :-1:12])
            dx = luma(rgb[:-1:12, 1::12]) - centre
            dy = luma(rgb[1::12, :-1:12]) - centre
            ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            rows.writerow([ns, record, float(centre.mean()), float((abs(dx)+abs(dy)).mean()/2),
                           float(np.mean(centre < 16))])


if __name__ == "__main__":
    main()
