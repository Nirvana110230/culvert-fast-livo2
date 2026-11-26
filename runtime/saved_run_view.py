"""Open a committed map_gravity archive directly; no TF broadcaster and no hardware."""
import argparse
from decimal import Decimal, ROUND_FLOOR
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from gravity import GRAVITY
from gravity_export import validate_map
from map_store import FILES, pcd_info, ros_cloud, tum_rows


def ros_stamp(text):
    value = Decimal(text)
    seconds = int(value.to_integral_value(rounding=ROUND_FLOOR))
    nanos = int(((value - seconds) * 1000000000).to_integral_value())
    return seconds + nanos // 1000000000, nanos % 1000000000


def display(directory, no_gui, seconds):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fast_livo2_saved_map_view")
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = []
    rviz = None
    try:
        for kind in ("geometry", "rgb"):
            path = directory / FILES[kind]
            h = pcd_info(path)
            data = np.memmap(path, mode="r", dtype=h.dtype, offset=h.offset, shape=(h.count,))
            msg = ros_cloud(data, GRAVITY, node.get_clock().now().to_msg())
            msg.height, msg.width = h.height, h.width
            msg.row_step = h.width * msg.point_step
            pub = node.create_publisher(PointCloud2, "/saved_experiment/" + kind, qos)
            pub.publish(msg)
            pubs.append(pub)
            del data, msg
        path = RosPath()
        path.header.frame_id, path.header.stamp = GRAVITY, node.get_clock().now().to_msg()
        for stamp, p, q in tum_rows(directory / FILES["poses"]):
            pose = PoseStamped()
            pose.header.frame_id = GRAVITY
            pose.header.stamp.sec, pose.header.stamp.nanosec = ros_stamp(stamp)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p)
            orientation = pose.pose.orientation
            orientation.x, orientation.y, orientation.z, orientation.w = map(float, q)
            path.poses.append(pose)
        pub = node.create_publisher(RosPath, "/saved_experiment/path", qos)
        pub.publish(path)
        pubs.append(pub)
        if not no_gui:
            env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":1"),
                       XAUTHORITY=os.environ.get("XAUTHORITY", str(Path.home() / ".Xauthority")),
                       QT_QPA_PLATFORM="xcb", LIBGL_ALWAYS_SOFTWARE="1", LP_NUM_THREADS="2")
            rviz = subprocess.Popen(["rviz2", "-d", str(Path(__file__).with_name("saved_run.rviz"))],
                                    env=env, start_new_session=True)
        until = time.monotonic() + seconds if seconds > 0 else float("inf")
        while not stopped and time.monotonic() < until and (rviz is None or rviz.poll() is None):
            rclpy.spin_once(node, timeout_sec=.2)
        if rviz and rviz.poll() not in (None, 0):
            raise RuntimeError("RViz 异常退出：" + str(rviz.returncode))
    finally:
        if rviz and rviz.poll() is None:
            from rviz_window import stop_rviz
            stop_rviz(rviz, env)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--run", type=Path, help="正式会话目录")
    source.add_argument("--map", type=Path, help="含 manifest.json 的独立成品目录")
    parser.add_argument("--check", action="store_true", help="独立读取并核对坐标、点数、轨迹和校验码")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--seconds", type=float, default=0, help="正值仅限制本次离线查看的时长")
    args = parser.parse_args()
    directory = (args.map if args.map is not None else args.run / "map").resolve()
    result = validate_map(directory)
    print(f"{directory}  {GRAVITY}  {result['counts']}", flush=True)
    if not args.check:
        os.environ.setdefault("ROS_DOMAIN_ID", "219")
        display(directory, args.no_gui, args.seconds)


if __name__ == "__main__":
    main()
