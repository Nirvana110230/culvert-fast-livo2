"""Read the running mapper's parameters or replay recorded CDR input over ROS 2."""
import argparse
import signal
import time
from pathlib import Path

import yaml

from run_files import write_json


def contract_parameters(expected):
    return {
        'lio.max_iterations': expected.get('lio.max_iterations', expected.get('lio.min_iterations', 5)),
        'vio.exposure_estimate_en': expected.get('vio.exposure_estimate_en',
                                                 expected.get('imu.exposure_estimate_en', True)),
        'imu.b_gyr_cov': expected.get('imu.b_gyr_cov', 0.0001),
        'imu.b_acc_cov': expected.get('imu.b_acc_cov', 0.0001),
    }


def topics(params):
    return {
        params['common.lid_topic']: ('livox_ros_driver2/msg/CustomMsg', 32),
        params['common.imu_topic']: ('sensor_msgs/msg/Imu', 2000),
        params['common.img_topic']: ('sensor_msgs/msg/Image', 8),
    }


def actual_parameters(node, run, expected):
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters

    client = node.create_client(GetParameters, '/laserMapping/get_parameters')
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError('laserMapping 参数服务未就绪')
    listing = node.create_client(ListParameters, '/laserMapping/list_parameters')
    if not listing.wait_for_service(timeout_sec=10):
        raise RuntimeError('laserMapping 参数列表服务未就绪')
    listed = listing.call_async(ListParameters.Request(prefixes=[], depth=0))
    rclpy.spin_until_future_complete(node, listed, timeout_sec=10)
    if not listed.done() or listed.result() is None:
        raise RuntimeError('读取运行参数名称超时')
    declared = set(listed.result().result.names)
    contract = contract_parameters(expected)
    expected = dict(expected, **contract)
    names = [name for name in expected if name in declared]
    response = client.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(node, response, timeout_sec=12)
    if not response.done() or response.result() is None:
        raise RuntimeError('读取 laserMapping 实际参数超时')
    fields = [None, 'bool_value', 'integer_value', 'double_value', 'string_value',
              'byte_array_value', 'bool_array_value', 'integer_array_value',
              'double_array_value', 'string_array_value']
    values, undeclared = {}, sorted(set(expected) - declared)
    for name, item in zip(names, response.result().values):
        if not item.type:
            undeclared.append(name)
            continue
        value = getattr(item, fields[item.type])
        values[name] = list(value) if item.type >= 5 else value
    mismatches = [key for key in values if values[key] != expected[key]]
    required = {key for key in expected if key.startswith(('parameter_blackboard.', 'extrin_calib.',
                'time_offset.', 'common.', 'vio.lru_', 'vio.local_map_'))}
    required.update(('uav.gravity_align_en', 'publish.pub_effect_point_en'))
    required.update(contract)
    required.update(key for key in ('lio.min_iterations', 'imu.exposure_estimate_en') if key in expected)
    missing = sorted(required.difference(values))
    write_json(run / 'active_parameters.json', {
        'node': '/laserMapping', 'host_ns': time.time_ns(), 'parameters': values,
        'undeclared': undeclared, 'missing_required': missing, 'mismatches': mismatches,
    })
    if missing or mismatches:
        raise RuntimeError(f'运行参数与快照不一致，缺失 {missing}，差异 {mismatches}')


def playback(node, run, path, rate, expected, stopped):
    import rclpy
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    metadata_file = path / 'metadata.yaml' if path.is_dir() else None
    metadata = (yaml.safe_load(metadata_file.read_text())['rosbag2_bagfile_information']
                if metadata_file and metadata_file.exists() else {})
    storage = metadata.get('storage_identifier') or ('mcap' if path.suffix == '.mcap' else 'sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    wanted = topics(expected)
    actual = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic, (msg_type, _) in wanted.items():
        if actual.get(topic) != msg_type:
            raise RuntimeError(f'bag 需要 {topic} ({msg_type})，查得 {actual.get(topic)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    publishers = {topic: node.create_publisher(get_message(msg_type), topic, QoSProfile(
                      depth=depth, reliability=ReliabilityPolicy.RELIABLE))
                  for topic, (msg_type, depth) in wanted.items()}
    counts = dict.fromkeys(wanted, 0)
    manifest = {'path': str(path), 'storage_id': storage, 'rate': rate,
                'stored_counts': {item['topic_metadata']['name']: item['message_count']
                                  for item in metadata.get('topics_with_message_count', [])
                                  if item['topic_metadata']['name'] in wanted},
                'published': counts, 'eof': False, 'stopped': False}
    try:
        deadline = time.monotonic() + 30
        while not stopped():
            seen = [any(info.node_name == 'laserMapping'
                        for info in node.get_subscriptions_info_by_topic(topic))
                    and pub.get_subscription_count() > 0 for topic, pub in publishers.items()]
            foreign = {t: [info.node_name for info in node.get_publishers_info_by_topic(t)
                           if info.node_name != node.get_name()] for t in wanted}
            if any(foreign.values()):
                raise RuntimeError(f'当前 ROS 域已有其他传感器发布者 {foreign}')
            if all(seen):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('实际 laserMapping 未匹配 bag 的全部三路发布者')
            rclpy.spin_once(node, timeout_sec=.05)

        first_stamp = None
        origin = time.monotonic()
        while not stopped() and reader.has_next():
            topic, serialized, stamp = reader.read_next()
            if first_stamp is None:
                first_stamp = stamp
            target = origin + (stamp - first_stamp) / (1e9 * rate)
            while not stopped() and time.monotonic() < target:
                time.sleep(min(.05, max(0.0, target - time.monotonic())))
            if not stopped():
                # rclpy publishes the original serialized message without editing headers.
                publishers[topic].publish(serialized)
                counts[topic] += 1
        manifest['eof'] = not stopped()
        for topic, publisher in publishers.items():
            if hasattr(publisher, 'wait_for_all_acked'):
                manifest.setdefault('dds_acks', {})[topic] = publisher.wait_for_all_acked(
                    Duration(seconds=10))
        if manifest['eof'] and (not all(counts.values()) or
                any(counts[t] != n for t, n in manifest['stored_counts'].items())):
            raise RuntimeError('回放发送计数与 bag 元数据不一致')
        if False in manifest.get('dds_acks', {}).values():
            raise RuntimeError('DDS 未在等待窗口内确认全部可靠输入')
    finally:
        manifest['stopped'] = stopped()
        manifest['finished_host_ns'] = time.time_ns()
        write_json(run / 'replay.json', manifest)


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['parameters', 'play'])
    parser.add_argument('run', type=Path)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--rate', type=float, default=1.)
    args = parser.parse_args()
    params = yaml.safe_load((args.run / 'resolved.yaml').read_bytes())['/**']['ros__parameters']
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fast_livo2_session_' + args.action)
    stopped = False

    def request(*_):
        nonlocal stopped
        stopped = True

    if args.action == 'play':
        signal.signal(signal.SIGINT, request)
        signal.signal(signal.SIGTERM, request)
    try:
        if args.action == 'parameters':
            actual_parameters(node, args.run, params)
        else:
            playback(node, args.run, args.bag, args.rate, params, lambda: stopped)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
