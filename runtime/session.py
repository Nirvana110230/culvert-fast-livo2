"""One FAST-LIVO2 session owns its inputs, mapper, gravity TF and map writer."""
import argparse
import datetime
import fcntl
import json
import math
import os
import resource
import shutil
import signal
import sys
import time
import uuid
from pathlib import Path

from hardware import Hardware, preflight, require_free_pipeline, standby
from run_files import (ALGORITHM, ROOT, Children, append_bounded, digest, latest_csv_row, proc_identity,
                       read_json, same_process, signal_owned, snapshot, write_json)


RUNTIME = ROOT / 'runtime'
STATE = ROOT / 'state'
DEFAULT_RUNS = Path('/data/robot/maps/fast_livo2/runs')
ACTIVE = STATE / 'active.json'
LATEST = STATE / 'latest_run.txt'


def data_directory(path):
    path = path.expanduser().resolve()
    if path.is_relative_to(ROOT):
        raise ValueError('会话数据路径必须独立于代码目录')
    if path.is_relative_to('/data') and not Path('/data').is_mount():
        raise RuntimeError('/data 数据卷尚未挂载')
    path.mkdir(parents=True, exist_ok=True)
    return path


def latest():
    try:
        return Path(LATEST.read_text().strip())
    except FileNotFoundError:
        return None


def status_data():
    active = read_json(ACTIVE)
    run = Path(active['run']) if active else latest()
    if run is None:
        return {'active': False, 'state': 'idle', 'run': None}
    state = read_json(run / 'session.json')
    control = read_json(STATE / 'last_result.json')
    if control.get('run') == str(run):
        state = control
    result = {'active': bool(active and same_process(active)), 'run': str(run),
              'state': state.get('state', 'starting'), 'mode': state.get('mode'),
              'stop_reason': state.get('stop_reason'), 'errors': state.get('errors', []),
              'processes': {}, 'resources': read_json(run / 'resources.json'),
              'processing': latest_csv_row(run / 'Log/processing.csv'),
              'visual_cache': latest_csv_row(run / 'Log/visual_lru.csv'),
              'map': read_json(run / 'map/result.json') or read_json(run / 'map/status.json')}
    if active and not result['active']:
        result['state'] = 'recovery_pending'
    for name, identity in read_json(run / 'processes.json').items():
        result['processes'][name] = {'pid': identity.get('pid'), 'alive': same_process(identity),
                                     'returncode': identity.get('returncode')}
    return result


class Monitor:
    def __init__(self, run):
        self.run = run
        self.started = time.monotonic()
        self.next = 0.0
        self.low_since = None
        self.summary = {'samples': 0, 'min_available_bytes': None, 'min_disk_free_bytes': None,
                        'max_algorithm_rss_bytes': 0}

    def sample(self, children, *, final=False):
        now = time.monotonic()
        if now < self.next and not final:
            return
        self.next = now + 2
        memory = {}
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, value = line.split(':')
            if key in ('MemAvailable', 'MemTotal'):
                memory[key] = int(value.split()[0]) * 1024
        free = shutil.disk_usage(self.run).free
        rss = {}
        for name, proc in children.items():
            if proc.poll() is None:
                try:
                    pages = int(Path('/proc', str(proc.pid), 'statm').read_text().split()[1])
                    rss[name] = pages * os.sysconf('SC_PAGE_SIZE')
                except (FileNotFoundError, ProcessLookupError):
                    pass
        row = {'host_ns': time.time_ns(), 'elapsed_s': now - self.started,
               'process_rss_bytes': rss, 'mem_available_bytes': memory['MemAvailable'],
               'mem_total_bytes': memory['MemTotal'], 'disk_free_bytes': free}
        for key, value in [('min_available_bytes', memory['MemAvailable']),
                           ('min_disk_free_bytes', free)]:
            old = self.summary[key]
            self.summary[key] = value if old is None else min(old, value)
        self.summary['samples'] += 1
        self.summary['max_algorithm_rss_bytes'] = max(
            self.summary['max_algorithm_rss_bytes'], rss.get('algorithm', 0))
        self.summary['latest'] = row
        append_bounded(self.run / 'resources.jsonl', (json.dumps(row) + '\n').encode())
        write_json(self.run / 'resources.json', self.summary)
        if final:
            return
        if free < 2 * 1024 ** 3:
            raise RuntimeError('数据卷剩余不足 2 GiB，开始保存')
        if memory['MemAvailable'] < 1536 * 1024 ** 2:
            if self.low_since is None:
                self.low_since = now
            if now - self.low_since >= 10:
                raise RuntimeError('系统 MemAvailable 连续 10 秒低于 1.5 GiB，开始保存')
        else:
            self.low_since = None


class RequestedStop(Exception):
    pass


def observed_process(run, pid, expected):
    proc = Path('/proc') / str(pid)
    exe = (proc / 'exe').resolve(strict=True)
    if exe != ALGORITHM.resolve(strict=True) or digest(proc / 'exe') != expected['sha256']:
        raise RuntimeError('当前算法进程身份与会话记录不一致')
    argv = [arg.decode() for arg in (proc / 'cmdline').read_bytes().split(b'\x00') if arg]
    if not any(arg == '--params-file' and argv[i + 1:i + 2] == [str(run / 'resolved.yaml')]
               for i, arg in enumerate(argv)):
        raise RuntimeError('算法进程没有消费本会话参数文件')
    paths = set()
    for line in (proc / 'maps').read_text().splitlines():
        parts = line.split(maxsplit=5)
        if len(parts) == 6 and parts[5].startswith('/'):
            paths.add(parts[5])
    actual = dict(expected, **proc_identity(pid), argv=argv, loaded_files=sorted(paths),
                  cwd=str((proc / 'cwd').resolve()), observed_host_ns=time.time_ns())
    write_json(run / 'algorithm_process.json', actual)


def heartbeat(run):
    try:
        p = (run / 'Log/processing.csv').stat()
        return p.st_ino, p.st_size, p.st_mtime_ns
    except FileNotFoundError:
        return None


def drain_inputs(run, children):
    """After the only source ends, wait for queued cycles to become quiescent."""
    last = heartbeat(run)
    changed = time.monotonic()
    while time.monotonic() - changed < 3.0:
        children.require_alive(['algorithm', 'archive'])
        time.sleep(.15)
        current = heartbeat(run)
        if last != current:
            last, changed = current, time.monotonic()


def session(args, run):
    children = Children(run, STATE / 'owned_processes.json')
    monitor = Monitor(run)
    state = {'session_id': run.name, 'run': str(run), 'mode': args.action,
             'state': 'starting', 'started_host_ns': time.time_ns(), 'errors': [],
             'stop_reason': None, 'data_device': run.stat().st_dev,
             'ros_domain_id': os.environ.get('ROS_DOMAIN_ID')}
    started = time.monotonic()
    signal_received = []
    hardware = None
    hold = False
    params = {}

    def transition(value):
        state['state'] = value
        write_json(run / 'session.json', state)

    def request(sig, _):
        if not signal_received:
            signal_received.append(signal.Signals(sig).name)

    def check():
        if signal_received or (run / 'stop.request').exists():
            raise RequestedStop
        monitor.sample(children.items)

    def wait_until(predicate, timeout, names, detail):
        deadline = time.monotonic() + timeout
        while True:
            check()
            children.require_alive(names)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'等待{detail}超时')
            time.sleep(.1)

    def stop_child(name, timeout, **kwargs):
        try:
            children.stop(name, timeout, **kwargs)
        except Exception as exc:
            state['errors'].append(str(exc))

    def get_result(path):
        try:
            return read_json(path)
        except (OSError, ValueError) as exc:
            state['errors'].append(f'读取 {path.name} 失败 {exc}')
            return {}

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request)
    write_json(ACTIVE, {**proc_identity(os.getpid()), 'run': str(run)})
    tmp = LATEST.with_suffix('.tmp')
    tmp.write_text(str(run) + '\n')
    tmp.replace(LATEST)
    transition('starting')
    print(f'会话 {run}', flush=True)
    try:
        (run / 'Log/result').mkdir(parents=True)
        (run / 'map').mkdir()
        (run / 'roslog').mkdir()
        overrides = {}
        if args.interval_deskew:
            overrides['imu.interval_deskew'] = True
        if args.stationary_gyro_bias:
            overrides['imu.stationary_start_gyro_bias'] = True
        params, manifest = snapshot(run, ROOT / 'configs', overrides)
        identity = {'session_id': run.name, 'executable': str(ALGORITHM),
                    'sha256': digest(ALGORITHM)}
        write_json(run / 'algorithm_identity.json', identity)
        env = dict(os.environ, ROS_LOG_DIR=str(run / 'roslog'), MALLOC_ARENA_MAX='2',
                   FAST_LIVO2_SESSION_ID=run.name, FAST_LIVO2_CONFIG_SHA256=manifest['resolved_sha256'],
                   FAST_LIVO2_ALGORITHM_SHA256=identity['sha256'], FAST_LIVO2_GRAVITY_DIR=str(run))
        if args.keyframes or args.loop_backend:
            env['FAST_LIVO_KEYFRAMES'] = str(run / 'keyframes')
        if args.diagnostics:
            env['FAST_LIVO_DIAGNOSTICS_DIR'] = str(run / 'diagnostics')
        if args.strict_windows:
            env['FAST_LIVO_STRICT_WINDOWS'] = '1'
        state['strict_image_windows'] = env.get('FAST_LIVO_STRICT_WINDOWS') == '1'
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        check()
        require_free_pipeline(args.action == 'start')
        if args.action == 'start':
            preflight()
        else:
            args.bag = args.bag.expanduser().resolve(strict=True)
        children.start('archive', [sys.executable, '-u', RUNTIME / 'archive.py', run], env)
        wait_until((run / 'map/ready').exists, 30, ['archive'], '地图订阅与输出')
        if args.action == 'start':
            print('设备保持静止，正在启动雷达、IMU 与外触发相机。', flush=True)
            hardware = Hardware(run, children, env, STATE / 'hardware_active.json')
            hardware.start(wait_until)
        command = ['nice', '-n', '5', ALGORITHM, '--ros-args', '--disable-external-lib-logs',
                   '--params-file', run / 'resolved.yaml']
        cores = sorted(set(os.sched_getaffinity(0)).intersection(range(4, 8)))
        if cores:
            command = ['taskset', '-c', ','.join(map(str, cores))] + command
        algorithm = children.start('algorithm', command, env)
        live = ['camera', 'driver'] if hardware else []
        owned = ['algorithm', 'archive'] + live
        probe = children.start('parameters', [sys.executable, RUNTIME / 'ros_input.py',
                                              'parameters', run], env)
        wait_until(lambda: probe.poll() is not None, 60, owned, '运行中的算法参数')
        children.stop('parameters', 2)
        observed_process(run, algorithm.pid, identity)
        if args.loop_backend:
            prefix = os.environ.get('FAST_LIVO2_PGO_PREFIX', str(ROOT/'backend/pydeps'))
            pgo_env = dict(env, PYTHONPATH=prefix + os.pathsep + env.get('PYTHONPATH',''),
                           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2')
            command = ['nice','-n','10',sys.executable,ROOT/'backend/live_tf.py',
                       '--index',run/'keyframes/index.csv','--gravity',run/'gravity_events.jsonl',
                       '--out',run/'pgo','--complete',run/'keyframes/complete.json','--rebuild']
            children.start('loop_backend', command, pgo_env)
        player = None
        if not hardware:
            player = children.start('replay', [sys.executable, RUNTIME / 'ros_input.py', 'play',
                                    run, '--bag', args.bag, '--rate', str(args.rate)], env)
        transition('running')
        print('融合已启动，Ctrl+C 或 stop.sh 停止并保存。', flush=True)
        announced = False
        last_progress, changed = heartbeat(run), time.monotonic()
        while True:
            check()
            children.require_alive(owned)
            if player and player.poll() is not None and not hold:
                children.stop('replay', 2)
                if not read_json(run / 'replay.json').get('eof'):
                    raise RuntimeError('回放提前结束')
                state['stop_reason'] = 'bag_eof'
                if not args.wait_stop:
                    break
                hold = True
                transition('awaiting_stop')
                print('回放到达结尾，正在等待 stop.sh。', flush=True)
            if not announced:
                status = read_json(run / 'map/status.json')
                if status.get('gravity_ready') and status.get('poses', 0):
                    announced = True
                    print('本会话重力与轨迹已就绪。', flush=True)
            beat = heartbeat(run)
            if beat != last_progress:
                last_progress, changed = beat, time.monotonic()
            if hardware and time.monotonic() - changed > 25.0:
                raise RuntimeError('实时融合已连续 25 秒无处理进展，开始保存')
            time.sleep(.2)
    except RequestedStop:
        state['stop_reason'] = 'user_' + (signal_received[0] if signal_received else 'stop')
    except Exception as exc:
        state['errors'].append(str(exc))
        state['stop_reason'] = 'failure'
    finally:
        try:
            transition('stopping')
        except OSError as exc:
            state['errors'].append(f'保存停止状态失败 {exc}')
        if 'parameters' in children.items and 'returncode' not in children.records['parameters']:
            stop_child('parameters', 4, interrupt_ok=True)
        if 'replay' in children.items and 'returncode' not in children.records['replay']:
            stop_child('replay', 40, interrupt_ok=True)
        if hardware:
            try:
                state['errors'].extend(hardware.stop())
            except Exception as exc:
                state['errors'].append(str(exc))
            state['sensor_result'] = hardware.result
        if 'algorithm' in children.items:
            try:
                if children.items['algorithm'].poll() is None:
                    drain_inputs(run, children)
            except Exception as exc:
                state['errors'].append(str(exc))
        stop_child('algorithm', 45)
        # Archive can legitimately take arbitrarily long to finish a large map.
        stop_child('archive', None)
        if 'loop_backend' in children.items:
            sidecar = children.items['loop_backend']
            committed = get_result(run / 'keyframes/complete.json')
            if committed:
                sidecar.wait()  # Drain the committed prefix and rebuild historical clouds on NVMe.
            stop_child('loop_backend', 30, interrupt_ok=True)
            result = get_result(run / 'pgo/online_summary.json')
            state['loop_backend'] = result
            frames = committed.get('frames')
            rebuilt = result.get('rebuild', {})
            empty = frames == 0 and (result.get('status') == 'no_initialized_keyframes' or
                                     (result.get('published') == 0 and result.get('status') == 'complete'
                                      and rebuilt.get('status') == 'no_input'))
            if not empty and not (result.get('status') == 'complete' and frames is not None
                                  and result.get('published') == frames and rebuilt.get('frames') == frames
                                  and rebuilt.get('status') == 'complete'):
                state['errors'].append('回环发布或历史地图重建未完成')
        state['map'] = get_result(run / 'map/result.json')
        if 'archive' in children.items and state['map'].get('status') != 'complete':
            state['errors'].append('地图归档未完成 ' + str(state['map'].get('error', '无最终结果')))
        state['mapper_counters'] = get_result(run / 'Log/input_counters.json')
        if args.action == 'replay':
            play = get_result(run / 'replay.json')
            counts = state['mapper_counters']
            if play.get('eof') and counts:
                try:
                    state['replay_input_delta'] = {
                        key: play['published'][params[setting]] - counts[key]
                        for key, setting in [('lidar_rx', 'common.lid_topic'), ('imu_rx', 'common.imu_topic'),
                                             ('image_rx', 'common.img_topic')]}
                    if any(state['replay_input_delta'].values()):
                        state['errors'].append('回放发送与算法接收计数有差异')
                except (KeyError, TypeError) as exc:
                    state['errors'].append(f'回放输入计数不完整 {exc}')
        state['processes'] = {name: proc.poll() for name, proc in children.items.items()}
        state['elapsed_s'] = time.monotonic() - started
        state['ended_host_ns'] = time.time_ns()
        state['errors'].extend(f'{k} 日志写入失败 {v}' for k, v in children.log_errors.items())
        try:
            monitor.sample(children.items, final=True)
            transition('failed' if state['errors'] else 'complete')
        except OSError as exc:
            state['state'] = 'failed'
            state['errors'].append(f'会话输出卷写入失败 {exc}')
        # The control volume is independent of the NVMe output. Retain the
        # bounded final result even if the data disk filled up during shutdown.
        write_json(STATE / 'last_result.json', state)
        sensor_result = state.get('sensor_result', {})
        radar_safe = (not sensor_result.get('radar_start_attempted') or
                      sensor_result.get('radar_idle', {}).get('state_and_wire_check'))
        if radar_safe and all(proc.poll() is not None for proc in children.items.values()):
            ACTIVE.unlink(missing_ok=True)
        print(f"{state['state']}  {run / 'map'}", flush=True)
        for error in state['errors']:
            print(error, file=sys.stderr)
    return int(bool(state['errors']))


def recover(active):
    """Recover surviving owned children in the same order as a normal shutdown."""
    run = Path(active['run'])
    owned = read_json(STATE / 'owned_processes.json')
    if owned.get('run') == str(run):
        children = owned['processes']
    else:
        children = read_json(run / 'processes.json')
    try:
        state = read_json(run / 'session.json')
    except (OSError, ValueError):
        state = {}
    errors = []
    for name in ('parameters', 'replay', 'camera', 'driver', 'observer', 'algorithm', 'loop_backend', 'archive'):
        record = children.get(name)
        if not record or not same_process(record):
            continue
        signal_owned(record, signal.SIGINT)
        deadline = time.monotonic() + 40
        while same_process(record):
            if name != 'archive' and time.monotonic() > deadline:
                signal_owned(record, signal.SIGKILL)
                errors.append(f'强制结束遗留 {name}')
                break
            time.sleep(.15)
    sensor_dir = run / 'sensors'
    radar_safe = True
    hardware_control = STATE / 'hardware_active.json'
    if read_json(hardware_control).get('run') == str(run) or (sensor_dir / 'radar_start_attempted').exists():
        try:
            state.setdefault('sensor_result', {})['radar_idle'] = standby(sensor_dir)
            hardware_control.unlink(missing_ok=True)
        except Exception as exc:
            radar_safe = False
            errors.append(f'雷达待机未证实 {exc}')
    if any(same_process(item) for item in children.values()):
        raise RuntimeError('仍有未退出的本会话子进程')
    try:
        saved = read_json(run / 'map/result.json')
    except (OSError, ValueError):
        saved = {}
    state.update(run=str(run), state='failed', stop_reason='supervisor_exit_recovered',
                 ended_host_ns=time.time_ns(), map=saved)
    state.setdefault('errors', []).extend(['会话监督进程提前退出'] + errors)
    try:
        write_json(run / 'session.json', state)
    except OSError as exc:
        state['errors'].append(f'归档卷写入失败 {exc}')
    write_json(STATE / 'last_result.json', state)
    if radar_safe:
        ACTIVE.unlink(missing_ok=True)
    print(f'已处理上次异常退出的会话 {run}', file=sys.stderr)
    if errors:
        raise RuntimeError('；'.join(errors))


def stop_current():
    active = read_json(ACTIVE)
    if not active:
        print('当前没有运行中的会话。')
        return 0
    run = Path(active['run'])
    if same_process(active):
        signal_owned(active, signal.SIGINT)
        try:
            (run / 'stop.request').touch()
        except OSError:
            pass  # The verified supervisor still receives SIGINT if its output disk failed.
        print(f'正在等待正常停止和地图写入 {run}', flush=True)
        while same_process(active):
            time.sleep(.25)
    if read_json(ACTIVE) == active:
        with (STATE / 'session.lock').open('a+') as lock:
            while read_json(ACTIVE) == active:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    time.sleep(.15)
                    continue
                if read_json(ACTIVE) == active:
                    recover(active)
                break
    result = read_json(run / 'session.json')
    control = read_json(STATE / 'last_result.json')
    if control.get('run') == str(run):
        result = control
    print(f"{result.get('state', 'unknown')}  {run / 'map'}")
    return int(result.get('state') != 'complete')


def cli():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    start = sub.add_parser('start', help='现场连续 LIVO')
    replay = sub.add_parser('replay', help='通过 ROS 回放三路原始输入')
    for action in (start, replay):
        action.add_argument('--loop-backend', action='store_true', help='导出关键帧并启动独立 map→odom 回环进程')
        action.add_argument('--stationary-gyro-bias', action='store_true', help='使用已知静止初段估计初始陀螺偏置')
        action.add_argument('--interval-deskew', action='store_true', help='使用对应 IMU 积分区间的运动量去畸变')
        action.add_argument('--diagnostics', action='store_true', help='保存逐窗时间、惯导初始化和点面统计')
        action.add_argument('--strict-windows', action='store_true', help='使用半开时间窗分配雷达点')
        action.add_argument('--keyframes', action='store_true', help='按估计器参考时刻保存后端局部关键帧')
        action.add_argument('--runs-dir', type=Path,
                            default=Path(os.environ.get('FAST_LIVO2_RUNS_DIR', DEFAULT_RUNS)))
    replay.add_argument('bag', type=Path)
    replay.add_argument('--rate', type=float, default=1.0)
    replay.add_argument('--wait-stop', action='store_true', help='bag 结束后等待 stop.sh 正常归档')
    sub.add_parser('stop')
    status = sub.add_parser('status')
    status.add_argument('--json', action='store_true')
    view = sub.add_parser('view')
    source = view.add_mutually_exclusive_group()
    source.add_argument('--run', type=Path, help='打开已归档会话的完整地图')
    source.add_argument('--map', type=Path, help='打开已导出的独立地图目录')
    source.add_argument('--pgo', type=Path, help='打开独立回环后端的历史重建地图')
    view.add_argument('--before', action='store_true', help='查看同一批关键帧的原始拼接地图')
    source.add_argument('--foxglove', action='store_true', help='实时 Foxglove，仅回环端口 8765')
    view.add_argument('--check', action='store_true', help='独立核验归档文件')
    view.add_argument('--no-gui', action='store_true', help='发布归档地图供订阅')
    view.add_argument('--seconds', type=float, default=0, help='可选，离线查看时长')
    args = parser.parse_args()
    if args.action == 'status':
        info = status_data()
        if args.json:
            print(json.dumps(info, ensure_ascii=False, indent=2))
        else:
            print(f"{info['state']}  {info['run'] or ''}")
            if info.get('run'):
                alive = [f"{name}={record['pid']}" for name, record in info['processes'].items()
                         if record['alive']]
                if alive:
                    print('运行进程  ' + '  '.join(alive))
                resource_row = info['resources'].get('latest')
                if resource_row:
                    print(f"已运行 {resource_row['elapsed_s']:.1f} s  "
                          f"可用内存 {resource_row['mem_available_bytes']/1024**3:.2f} GiB  "
                          f"数据盘剩余 {resource_row['disk_free_bytes']/1024**3:.2f} GiB")
                if info['map']:
                    out = info['map']
                    print(f"RGB 点 {out.get('rgb_points', 0)}  几何点 {out.get('geometry_points', 0)}  "
                          f"轨迹 {out.get('poses', 0)}  坐标 {out.get('output_frame') or out.get('frame', '未确认')}")
                processing = info['processing']
                if processing:
                    print(f"收到图像 {processing.get('image_rx', '?')}  "
                          f"视觉更新帧 {processing.get('visual_update_frames', '?')}")
            if info.get('errors'):
                print('；'.join(info['errors']))
        return 0
    if args.action == 'stop':
        return stop_current()
    if args.action == 'view':
        if args.before and not args.pgo:
            parser.error('--before 需要 --pgo')
        if args.run or args.map or args.pgo:
            path = (args.run or args.map or args.pgo).expanduser().resolve(strict=True)
            source_arg = '--run' if args.run else ('--pgo' if args.pgo else '--map')
            command = [sys.executable, str(RUNTIME / 'saved_run_view.py'),
                       source_arg, str(path), '--seconds', str(args.seconds)]
            if args.before:
                command.append('--before')
            if args.check:
                command.append('--check')
            if args.no_gui:
                command.append('--no-gui')
            # Offline topics must not join a live viewer's discovery domain.
            os.environ['ROS_DOMAIN_ID'] = os.environ.get('FAST_LIVO2_OFFLINE_DOMAIN', '219')
            os.execv(sys.executable, command)
        if args.check or args.no_gui or args.seconds:
            parser.error('--check、--no-gui 和 --seconds 需要指定 --run 或 --map')
        from view_live import display
        display(args.foxglove)
        return 0
    if args.action == 'replay' and (not math.isfinite(args.rate) or args.rate <= 0):
        parser.error('--rate 必须为有限正数')
    STATE.mkdir(exist_ok=True)
    with (STATE / 'session.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有建图或回放会话在运行')
        previous = read_json(ACTIVE)
        if previous:
            if same_process(previous):
                raise RuntimeError('会话监督进程仍在运行')
            recover(previous)
        runs = data_directory(args.runs_dir)
        # UTC ISO basic form is unambiguous and valid for the estimator's session identity.
        now = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        run = runs / f'{now}_{uuid.uuid4().hex[:10]}'
        run.mkdir()
        return session(args, run)


if __name__ == '__main__':
    try:
        raise SystemExit(cli())
    except (RuntimeError, OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
