"""Attach an RViz or loopback Foxglove consumer to one mapping session."""
import os
import signal
import subprocess
import time
from pathlib import Path
from rviz_window import stop_rviz

from run_files import ROOT, read_json, same_process


def display(foxglove):
    active_path = ROOT / 'state/active.json'
    owner = read_json(active_path)
    if owner and not same_process(owner):
        raise RuntimeError('上次会话监督进程已退出')
    if not owner and not foxglove:
        raise RuntimeError('当前没有活动会话，使用 view.sh --run <会话目录> 查看已存地图')
    env = os.environ.copy()
    if foxglove:
        command = ['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge', '--ros-args',
                   '--disable-external-lib-logs', '--params-file', ROOT / 'configs/foxglove.yaml']
        print('Foxglove 连接 ws://127.0.0.1:8765；远程客户端通过 SSH 本地转发连接。', flush=True)
    else:
        env.setdefault('DISPLAY', ':1')
        env.setdefault('XAUTHORITY', str(Path.home() / '.Xauthority'))
        env.setdefault('QT_QPA_PLATFORM', 'xcb')
        env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
        env.setdefault('LP_NUM_THREADS', '2')
        command = ['rviz2', '-d', ROOT / 'runtime/continuous.rviz']
    stopped = False

    def request(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request)
    signal.signal(signal.SIGTERM, request)
    proc = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        while not stopped and proc.poll() is None:
            current = read_json(active_path)
            if owner and (current != owner or not same_process(owner)):
                break
            if not owner and current and same_process(current):
                owner = current
            time.sleep(.2)
        if proc.returncode not in (None, 0):
            raise RuntimeError(f'查看器退出码 {proc.returncode}')
    finally:
        if not foxglove:
            stop_rviz(proc, env)
        elif proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(4)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
