"""Live-only Mid-360S, camera trigger and shared timestamp sequencing."""
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

from run_files import ROOT, digest, read_json, write_json
from sensors.cadence_gate import cadence_ready


SENSORS = ROOT / 'runtime/sensors'
CONFIG = ROOT / 'configs/sensors'
DRIVER = ROOT / 'install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
CAMERA = ROOT / 'install/bin/camera_sync'


def require_free_pipeline(live):
    blocked = {'fastlivo_mapping'}
    if live:
        blocked.update(('livox_ros_driver2_node', 'mvs_camera_node', 'camera_sync',
                        'camera_probe', 'sync_probe', 'MVS'))
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            exe = (proc / 'exe').readlink().name
            if exe in blocked:
                raise RuntimeError(f'已有 {exe}，PID {proc.name}，请先结束该会话')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue


def preflight():
    for key in ("FAST_LIVO2_RADAR_SERIAL", "FAST_LIVO2_CAMERA_SERIAL"):
        if not os.environ.get(key, "").strip():
            raise RuntimeError(f"{key} is required for live acquisition")
    for path in (DRIVER, CAMERA, CONFIG / 'MID360s.json', SENSORS / 'radar_device.py',
                 SENSORS / 'radar_packets.py', CONFIG / 'fastdds_camera.xml'):
        if not path.is_file():
            raise FileNotFoundError(path)
    if Path(os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')).resolve() != (
            CONFIG / 'fastdds_camera.xml').resolve():
        raise RuntimeError('主线 Fast DDS 相机配置未生效')
    interface = 'enP3p49s0'
    if (Path('/sys/class/net') / interface / 'carrier').read_text().strip() != '1':
        raise RuntimeError(f'雷达网口 {interface} 无载波')
    address = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'dev',
                                                 interface], text=True))
    expected_ip = json.loads((CONFIG / 'MID360s.json').read_bytes())['Mid360s']['host_net_info'][0]['host_ip']
    if not any(item.get('local') == expected_ip and item.get('prefixlen') == 24
               for device in address for item in device['addr_info']):
        raise RuntimeError('雷达网口地址与已验证配置不一致')


def radar_action(action):
    # These controls produce a single small JSON result. A full data volume must
    # never prevent a standby command or wire check from reaching the device.
    proc = subprocess.run([sys.executable, SENSORS / 'radar_device.py', action],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=28)
    try:
        result = json.loads(proc.stdout)
    except ValueError:
        raise RuntimeError(f'雷达 {action} 返回 {proc.returncode}，无有效状态记录') from None
    if proc.returncode:
        raise RuntimeError(f"雷达 {action} 失败 {result.get('error', proc.returncode)}")
    return result


def standby(folder):
    state = radar_action('standby')
    proc = subprocess.run(['sudo', '-n', 'python3', SENSORS / 'radar_packets.py',
                           '--seconds', '2', '--expect', 'idle'], stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=8)
    try:
        wire = json.loads(proc.stdout)
    except ValueError:
        raise RuntimeError('雷达已执行待机命令，未获得被动抓包核查结果') from None
    good = (state['ack']['ret_code'] == 0 and state['ack']['error_key'] == 0
            and state['confirmed']['values']['current_mode'] == 2 and wire['passed']
            and wire['capture_socket_drops'] == 0 and wire['counts'] == {'imu': 0, 'point': 0})
    if not good or proc.returncode:
        raise RuntimeError('雷达待机回读或线上点云/IMU 停止核查失败')
    result = {'state_and_wire_check': True, 'mode': 2, 'wire_seconds': wire['seconds'],
              'ack': state['ack'], 'wire': wire}
    try:
        write_json(folder / 'radar_standby.json', state)
        write_json(folder / 'radar_idle_packets.json', wire)
    except OSError as exc:
        result['evidence_write_error'] = str(exc)
    return result


class Hardware:
    def __init__(self, run, children, env, control):
        self.folder = run / 'sensors'
        self.children = children
        self.env = env
        self.control = control
        self.result = {}
        self.radar_start_attempted = False

    def start(self, wait_until):
        out = self.folder
        for directory in (out, out / 'camera', out / 'observe'):
            directory.mkdir()
        source = CONFIG / 'driver.yaml'
        config = yaml.safe_load(source.read_bytes())
        params = config['/**']['ros__parameters']
        radar_config = (source.parent / params['user_config_path']).resolve(strict=True)
        params['user_config_path'] = str(radar_config)
        resolved = out / 'driver.yaml'
        resolved.write_text(yaml.safe_dump(config), encoding='utf-8')
        write_json(out / 'config_manifest.json', {
            'source': str(source), 'source_sha256': digest(source),
            'driver_config': str(resolved), 'driver_sha256': digest(resolved),
            'lidar_json': str(radar_config), 'lidar_sha256': digest(radar_config),
            'trigger': {'line': 'Line0', 'edge': 'RisingEdge'},
        })
        shared = out / 'timeshare'
        shared.write_bytes(bytes(16))
        env = dict(self.env, LIVO_TIMESHARE_PATH=str(shared),
                   LIVO_TIMESHARE_TRACE=str(out / 'timeshare_writes.csv'))
        launch = self.children.start
        launch('observer', [sys.executable, SENSORS / 'observe_sync.py', out / 'observe'], env, out)
        wait_until((out / 'observe/observer.started').exists, 20, ['observer'], '传感器观察器')
        # Persist before the first control request. stop can recover even after a lost ACK.
        (out / 'radar_start_attempted').touch()
        write_json(self.control, {'run': str(out.parent), 'radar_start_attempted': True})
        self.radar_start_attempted = True
        write_json(out / 'radar_start.json', radar_action('start'))
        launch('driver', [DRIVER, '--ros-args', '--disable-external-lib-logs', '--params-file', resolved, '-r',
                          '__node:=mid360s_joint_driver'], env, out)
        wait_until(lambda: int.from_bytes(shared.read_bytes()[8:], 'little') != 0,
                   45, ['observer', 'driver'], '本会话共享雷达时戳')
        camera_env = dict(env)
        camera_lib = str(Path(env['MVS_ROOT']) / 'lib/aarch64')
        camera_env['LD_LIBRARY_PATH'] = os.pathsep.join(filter(None, (camera_lib, env.get('LD_LIBRARY_PATH'))))
        launch('camera', [CAMERA, '--mode', 'trigger', '--line', 'Line0', '--edge', 'RisingEdge',
                          '--seconds', '0', '--output', out / 'camera'], camera_env, out)
        children = ['observer', 'driver', 'camera']
        wait_until((out / 'observe/observer.ready').exists, 40, children, '三路真实输入')

        def cadence():
            streams = {}
            for key in ('lidar', 'imu', 'image'):
                with (out / 'observe' / (key + '.csv')).open() as rows:
                    streams[key] = list(csv.DictReader(rows))
            try:
                ready, details = cadence_ready(streams, time.monotonic_ns())
            except (ValueError, TypeError, KeyError):
                return False  # The observer may still be writing its last CSV row.
            if ready:
                write_json(out / 'transport_cadence_ready.json', {
                    'transport_only': True, 'common_time_axis_verified': False, 'details': details})
            return ready

        wait_until(cadence, 25, children, '三路输入持续到达')
        self.children.stop('observer', 8)

    def stop(self):
        errors = []
        result = {'radar_start_attempted': self.radar_start_attempted}
        self.result = result
        for name in ('camera', 'driver', 'observer'):
            if name in self.children.items and 'returncode' not in self.children.records[name]:
                try:
                    self.children.stop(name, 30)
                except Exception as exc:
                    errors.append(str(exc))
        if result['radar_start_attempted']:
            try:
                result['radar_idle'] = standby(self.folder)
                self.control.unlink(missing_ok=True)
                if result['radar_idle'].get('evidence_write_error'):
                    errors.append('保存雷达待机回读记录失败')
            except Exception as exc:
                errors.append(f'雷达待机未证实 {exc}')
        if 'camera' in self.children.items:
            camera = read_json(self.folder / 'camera/driver_summary.json')
            result['camera'] = camera
            if camera.get('settings_restored') is not True:
                errors.append('相机设定和曝光事件设定恢复未证实')
        result['errors'] = errors
        if self.folder.exists():
            write_json(self.folder / 'session.json', result)
        return errors
