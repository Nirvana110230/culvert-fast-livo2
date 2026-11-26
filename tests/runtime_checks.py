"""Linux orchestration regressions. Fixtures never access physical sensors."""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'runtime'))
import ros_input
import run_files
import session
import hardware


def fixture_environment(root):
    session.ROOT = root / 'code'
    session.STATE = session.ROOT / 'state'
    session.ACTIVE = session.STATE / 'active.json'
    session.LATEST = session.STATE / 'latest_run.txt'
    session.RUNTIME = session.ROOT / 'runtime'
    session.ALGORITHM = session.ROOT / 'mapper'


class Runtime(unittest.TestCase):
    def test_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            values, manifest = run_files.snapshot(run, ROOT / 'configs')
            self.assertEqual([Path(s['path']).name for s in manifest['sources']],
                             ['base.yaml', 'camera.yaml', 'extrinsic.yaml', 'time_overlay.yaml',
                              'display_only.yaml', 'runtime_policy.yaml'])
            self.assertEqual(manifest['resolved_sha256'], run_files.digest(run / 'resolved.yaml'))
            self.assertEqual(values['common.img_topic'], '/hik_camera/image_timeshare')
            self.assertEqual(values['common.img_en'], 1)
            self.assertEqual(values['time_offset.img_time_offset'], .1)
            self.assertIs(values['uav.gravity_align_en'], False)
            self.assertIs(values['publish.pub_effect_point_en'], True)
            self.assertNotIn('vio.history_image_budget_mib', values)
            origins = run_files.read_json(ROOT / 'records/source_origin.json')
            maintained = run_files.read_json(ROOT / 'records/maintained_source_sha256.json')
            for source in manifest['sources']:
                rel = Path(source['path']).relative_to(ROOT).as_posix()
                self.assertEqual(source['sha256'], maintained[rel], rel)
                if rel.startswith('configs/calibration/'):
                    self.assertEqual(source['sha256'], origins[rel]['sha256'], rel)

    def test_sensor_trace_rollover(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            source = out / 'exercise.cpp'
            source.write_text('''#include "scan_timestamp_writer.hpp"
int main() {
  ScanTimestampWriter writer;
  for (unsigned i=1; i<=90000; ++i) writer.publish(i, i, i+1, 2);
  SharedTimestamp reader(timestamp_environment("LIVO_TIMESHARE_PATH"), false);
  return reader.read_valid() == 90000 ? 0 : 1;
}
''')
            exe = out / 'exercise'
            subprocess.run(['c++', '-std=c++14', '-I', ROOT / 'src/livox_ros_driver2/src',
                            source, '-o', exe], check=True)
            share, trace = out / 'share', out / 'writes.csv'
            share.write_bytes(bytes(16))
            subprocess.run([exe], env=dict(os.environ, LIVO_TIMESHARE_PATH=str(share),
                                           LIVO_TIMESHARE_TRACE=str(trace)), check=True)
            previous = out / 'writes.csv.1'
            lines = trace.read_text().splitlines()
            old = previous.read_text().splitlines()
            self.assertEqual(lines[0], old[0])
            self.assertEqual(int(lines[1].split(',')[0]), int(old[-1].split(',')[0]) + 1)
            self.assertEqual(lines[-1].split(',')[:2], ['90000', '90000'])
            self.assertLess(max(previous.stat().st_size, trace.stat().st_size), 1024**2 + 256)

    def test_process_identity_and_bounded_console(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            children = run_files.Children(root)
            proc = children.start('writer', [sys.executable, '-c',
                'import os; chunk = bytes(65536); [os.write(1,chunk) for i in range(160)]'],
                dict(os.environ))
            record = children.records['writer']
            self.assertFalse(run_files.same_process(dict(record, boot_id='a different boot')))
            proc.wait(10)
            children.stop('writer', 1)
            self.assertFalse(run_files.same_process(record))
            for suffix in ('', '.1'):
                self.assertLessEqual((root / ('writer.log' + suffix)).stat().st_size, 4 * 1024**2)

    def test_standby_with_unavailable_data_volume(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            missing = root / 'unmounted' / 'run'
            control = root / 'hardware_active.json'
            run_files.write_json(control, {'run': str(missing)})
            hw = hardware.Hardware(missing, run_files.Children(missing), {}, control)
            hw.radar_start_attempted = True
            ack = {'ack': {'ret_code': 0, 'error_key': 0}, 'confirmed': {'values': {'current_mode': 2}}}
            wire = {'passed': True, 'capture_socket_drops': 0, 'counts': {'imu': 0, 'point': 0},
                    'seconds': 2.0}
            reply = subprocess.CompletedProcess([], 0, stdout=json.dumps(wire).encode())
            with patch.object(hardware, 'radar_action', return_value=ack) as radar, \
                    patch.object(hardware.subprocess, 'run', return_value=reply):
                errors = hw.stop()
            radar.assert_called_once_with('standby')
            self.assertTrue(hw.result['radar_idle']['state_and_wire_check'])
            self.assertIn('保存雷达待机回读记录失败', errors)
            self.assertFalse(control.exists())

    def test_slow_archive_and_external_stop(self):
        # Exercise actual PID checks, subprocess shutdown, a native executable,
        # unique staging, closing the archive last, and a fresh second session.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            code = root / 'code'
            runtime = code / 'runtime'
            runtime.mkdir(parents=True)
            (code / 'configs').symlink_to(ROOT / 'configs', target_is_directory=True)
            (root / 'bag').mkdir()
            (code / 'mapper.c').write_text('''#include <signal.h>
#include <stdio.h>
#include <unistd.h>
volatile sig_atomic_t running=1;
void stop(int sig){(void)sig; running=0;}
int main(void){signal(SIGINT,stop); while(running) usleep(30000);
 FILE *f=fopen("Log/input_counters.json","w");
 fputs("{\\"lidar_rx\\":2,\\"imu_rx\\":3,\\"image_rx\\":1}",f);
 return fclose(f);}
''')
            subprocess.run(['cc', code / 'mapper.c', '-o', code / 'mapper'], check=True)
            (runtime / 'ros_input.py').write_text(
                "import json, pathlib, sys\n"
                "if sys.argv[1] == 'play':\n"
                " pathlib.Path('replay.json').write_text(json.dumps({'eof':True,'published':"
                "{'/livox/lidar':2,'/livox/imu':3,'/hik_camera/image_timeshare':1}}))\n")
            (runtime / 'archive.py').write_text(
                "import json, pathlib, signal, time\nrunning=True\n"
                "def stop(*_):\n global running\n running=False\n"
                "signal.signal(signal.SIGINT,stop)\npathlib.Path('map/ready').touch()\n"
                "while running: time.sleep(.03)\n"
                "assert pathlib.Path('Log/input_counters.json').exists()\ntime.sleep(.4)\n"
                "pathlib.Path('map/result.json').write_text(json.dumps({'status':'complete'}))\n")
            command = [sys.executable, __file__, 'worker', str(root)]
            ids = []
            for mode in ('hold', 'auto'):
                args = command + ['replay', str(root / 'bag'), '--runs-dir', str(root / 'runs')]
                if mode == 'hold':
                    args.append('--wait-stop')
                with subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True) as proc:
                    if mode == 'hold':
                        limit = time.monotonic() + 10
                        while proc.poll() is None and time.monotonic() < limit:
                            latest = code / 'state/latest_run.txt'
                            if latest.exists():
                                run = Path(latest.read_text().strip())
                                s = run_files.read_json(run / 'session.json')
                                if s.get('state') == 'awaiting_stop':
                                    break
                            time.sleep(.05)
                        else:
                            proc.terminate()
                            self.fail(proc.communicate(timeout=10)[0])
                        status = subprocess.check_output(command + ['status', '--json'], text=True)
                        self.assertTrue(json.loads(status)['active'])
                        conflict = subprocess.run(args, capture_output=True)
                        self.assertEqual(conflict.returncode, 1)
                        subprocess.run(command + ['stop'], check=True, timeout=15, capture_output=True)
                    log = proc.communicate(timeout=15)[0]
                    self.assertEqual(proc.returncode, 0, log)
                latest = Path((code / 'state/latest_run.txt').read_text().strip())
                state = run_files.read_json(latest / 'session.json')
                self.assertEqual(state['state'], 'complete')
                self.assertTrue(all(v == 0 for v in state['processes'].values()))
                self.assertFalse((code / 'state/active.json').exists())
                self.assertEqual(state['replay_input_delta'], {'lidar_rx':0,'imu_rx':0,'image_rx':0})
                self.assertEqual(run_files.read_json(latest / 'algorithm_process.json')['cwd'], str(latest))
                ids.append(state['session_id'])
            self.assertNotEqual(*ids)

    @unittest.skipUnless(importlib.util.find_spec('rosbag2_py'), '需要先 source ROS 2 Humble')
    def test_ros_raw_playback_and_parameter_service(self):
        import rclpy, rosbag2_py
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        # A private ROS context checks CDR transport independent of installed Livox hardware.
        os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 10)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        receiver = rclpy.create_node('laserMapping')
        sender = rclpy.create_node('fast_livo2_session_play')
        receive = []
        executor = SingleThreadedExecutor()
        executor.add_node(receiver)
        th = threading.Thread(target=executor.spin)
        th.start()
        try:
            receiver.declare_parameter('common.img_en', 1)
            receiver.declare_parameter('uav.gravity_align_en', False)
            receiver.declare_parameter('publish.pub_effect_point_en', True)
            receiver.declare_parameter('lio.max_iterations', 5)
            receiver.declare_parameter('imu.b_acc_cov', 0.0001)
            receiver.declare_parameter('imu.b_gyr_cov', 0.0001)
            receiver.declare_parameter('vio.exposure_estimate_en', True)
            with tempfile.TemporaryDirectory() as tmp:
                run = Path(tmp)
                ros_input.actual_parameters(sender, run, {'common.img_en':1,'uav.gravity_align_en':False,
                                            'publish.pub_effect_point_en':True, 'absent.diagnostic': 0})
                actual = run_files.read_json(run / 'active_parameters.json')
                self.assertEqual(actual['undeclared'], ['absent.diagnostic'])
                data = [serialize_message(String(data=f'frame-{i}')) for i in range(6)]
                t = '/runtime_input_test'
                receiver.create_subscription(String, t, lambda msg: receive.append(msg), 8, raw=True)
                bag = run / 'bag'
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                            rosbag2_py.ConverterOptions('', ''))
                writer.create_topic(rosbag2_py.TopicMetadata(name=t, type='std_msgs/msg/String',
                                    serialization_format='cdr'))
                for i, value in enumerate(data):
                    writer.write(t, value, 1_500_000_000 + 50_000_000 * i)
                del writer
                with patch.object(ros_input, 'topics', return_value={t: ('std_msgs/msg/String', 8)}):
                    ros_input.playback(sender, run, bag, 1., {}, lambda: False)
                deadline = time.monotonic() + 2
                while len(receive) < len(data) and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertEqual(receive, data)
                play = run_files.read_json(run / 'replay.json')
                self.assertEqual(play['published'][t], len(data))
                self.assertEqual(play.get('dds_acks'), {t: True})
        finally:
            executor.shutdown()
            th.join()
            sender.destroy_node()
            receiver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'worker':
        fixture_environment(Path(sys.argv[2]))
        sys.argv = [str(session.RUNTIME / 'session.py'), *sys.argv[3:]]
        raise SystemExit(session.cli())
    unittest.main()
