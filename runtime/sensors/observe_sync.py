"""Low-cost raw ROS observer. Arrival/header comparisons never establish hardware sync."""
import csv
import json
import pathlib
import signal
import struct
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from livox_ros_driver2.msg import CustomMsg


def header(data):
    if data[:2] != b'\x00\x01':
        raise ValueError('Require little endian CDR1 encapsulation')
    sec, nano, length = struct.unpack_from('<iII', data, 4)
    if not 0 <= nano < 1000000000 or length < 1 or 16 + length > len(data):
        raise ValueError('Invalid CDR header')
    return sec * 1000000000 + nano, data[16:15 + length].decode(), 16 + length


def align(pos, size):
    return 4 + (pos - 4 + size - 1) // size * size


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    running = True
    counts = {'image': 0, 'metadata': 0, 'lidar': 0, 'imu': 0}
    previous = {}
    nonadvancing = {}
    error_count = 0
    errors = []
    files = []
    writers = {}
    fields = {
        'image': ['header_ns', 'observer_host_ns', 'observer_monotonic_ns', 'width', 'height', 'encoding', 'step', 'data_bytes'],
        'lidar': ['header_ns', 'observer_host_ns', 'observer_monotonic_ns', 'timebase_ns', 'points'],
        'imu': ['header_ns', 'observer_host_ns', 'observer_monotonic_ns'],
    }
    for name, names in fields.items():
        f = (out / (name + '.csv')).open('w', newline='')
        files.append(f)
        writers[name] = csv.writer(f)
        writers[name].writerow(names)
    metadata_file = (out / 'camera_metadata.jsonl').open('w')
    files.append(metadata_file)
    events_file = (out / 'camera_events.jsonl').open('w')
    files.append(events_file)
    events_count = 0

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('joint_sensor_time_observer')

    def error(name, exception):
        nonlocal error_count
        error_count += 1
        if len(errors) < 100:
            errors.append({'topic': name, 'error': repr(exception), 'host_ns': time.time_ns()})

    def record(name, data):
        host, mono = time.time_ns(), time.monotonic_ns()
        try:
            stamp, frame, pos = header(data)
            if name in previous and stamp <= previous[name]:
                nonadvancing[name] = nonadvancing.get(name, 0) + 1
            previous[name] = stamp
            if name == 'image':
                pos = align(pos, 4)
                height, width, ln = struct.unpack_from('<III', data, pos)
                pos += 12
                encoding = data[pos:pos + ln - 1].decode()
                pos += ln
                big = data[pos]
                pos = align(pos + 1, 4)
                step, length = struct.unpack_from('<II', data, pos)
                pos += 8
                if (width, height, encoding, big, step, length) != (1280, 1024, 'rgb8', 0, 3840, 3932160):
                    raise ValueError('Unexpected image geometry/encoding')
                if len(data) != pos + length:
                    raise ValueError('Unexpected image serialized data length')
                writers[name].writerow([stamp, host, mono, width, height, encoding, step, length])
            elif name == 'lidar':
                pos = align(pos, 8)
                timebase, points = struct.unpack_from('<QI', data, pos)
                writers[name].writerow([stamp, host, mono, timebase, points])
            else:
                writers[name].writerow([stamp, host, mono])
            counts[name] += 1
        except Exception as e:
            error(name, e)

    def metadata(message):
        try:
            m = json.loads(message.data)
            if not isinstance(m, dict) or 'frame_num' not in m or 'host_after_get_ns' not in m:
                raise ValueError('Missing metadata identity')
            metadata_file.write(json.dumps({'observer_host_ns': time.time_ns(),
                                            'observer_monotonic_ns': time.monotonic_ns(),
                                            'metadata': m}) + '\n')
            counts['metadata'] += 1
        except Exception as e:
            error('metadata', e)

    def camera_event(message):
        nonlocal events_count
        try:
            event = json.loads(message.data)
            events_file.write(json.dumps({'observer_host_ns': time.time_ns(),
                                          'observer_monotonic_ns': time.monotonic_ns(),
                                          'event': event}) + '\n')
            events_count += 1
        except Exception as e:
            error('camera_events', e)

    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    imu_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
    image_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(Image, '/hik_camera/image_timeshare', lambda data: record('image', data), image_qos, raw=True),
        node.create_subscription(CustomMsg, '/livox/lidar', lambda data: record('lidar', data), reliable, raw=True),
        node.create_subscription(Imu, '/livox/imu', lambda data: record('imu', data), imu_qos, raw=True),
        node.create_subscription(String, '/hardware_sync/camera_metadata', metadata, reliable),
        node.create_subscription(String, '/hardware_sync/camera_events', camera_event, reliable),
    ]
    (out / 'observer.started').write_text('node initialized; no data assumed\n')
    last_flush = 0
    ready = False
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if not ready and all(counts[k] >= (100 if k == 'imu' else 5) for k in counts):
                graph = {'nodes': node.get_node_names_and_namespaces(), 'topics': node.get_topic_names_and_types()}
                (out / 'graph_ready.json').write_text(json.dumps(graph, indent=2))
                (out / 'observer.ready').write_text('real image/metadata/lidar>=5 and imu>=100\n')
                ready = True
            if time.monotonic() - last_flush >= .5:
                for f in files:
                    f.flush()
                (out / 'live_counts.json').write_text(json.dumps({'counts': counts, 'error_count': error_count,
                                                                'last_header_ns': previous, 'host_ns': time.time_ns()}))
                last_flush = time.monotonic()
    finally:
        for f in files:
            f.close()
        (out / 'observer_summary.json').write_text(json.dumps({'counts': counts, 'camera_events': events_count, 'nonadvancing_headers': nonadvancing,
                                                              'error_count': error_count, 'first_errors': errors,
                                                              'synchronization_verified': False,
                                                              'note': 'Counts/rate and host headers alone do not prove hardware synchronization'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return int(error_count > 0)


if __name__ == '__main__':
    raise SystemExit(main())
