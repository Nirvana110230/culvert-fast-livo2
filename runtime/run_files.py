"""Session-local configuration, process identity and bounded logs."""
import hashlib
import csv
import json
import os
import signal
import subprocess
import threading
import time
from functools import lru_cache
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
ALGORITHM = ROOT / 'install/fast_livo/lib/fast_livo/fastlivo_mapping'
PARAMETERS = ('base.yaml', 'calibration/camera.yaml', 'calibration/extrinsic.yaml',
              'time_overlay.yaml', 'display_only.yaml', 'runtime_policy.yaml')


def read_json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return {}


def write_json(path, value):
    tmp = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with tmp.open('w', encoding='utf-8') as output:
        json.dump(value, output, indent=2, ensure_ascii=False)
        output.write('\n')
    tmp.replace(path)


def digest(path):
    sha = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            sha.update(chunk)
    return sha.hexdigest()


def latest_csv_row(path):
    """Inspect one complete sample without rereading a whole rolling log."""
    try:
        with path.open('rb') as stream:
            header = stream.readline()
            offset = stream.tell()
            size = stream.seek(0, os.SEEK_END)
            stream.seek(max(offset, size - 8192))
            tail = stream.read().split(b'\n')[:-1]
        if not tail:
            return {}
        names, values = list(csv.reader([header.decode().strip(), tail[-1].decode().strip()]))
        return dict(zip(names, values)) if len(names) == len(values) else {}
    except (FileNotFoundError, UnicodeError, csv.Error):
        return {}


def flatten(values, prefix=''):
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            yield from flatten(value, name + '.')
        else:
            yield name, value


def snapshot(run, config):
    parameters = {}
    sources = []
    for layer in PARAMETERS:
        path = config / layer
        content = path.read_bytes()
        parameters.update(flatten(yaml.safe_load(content)['/**']['ros__parameters']))
        sources.append({'path': str(path), 'sha256': hashlib.sha256(content).hexdigest()})
    output = run / 'resolved.yaml'
    output.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}, sort_keys=True),
                      encoding='utf-8')
    manifest = {'run': str(run), 'sources': sources, 'resolved_sha256': digest(output)}
    write_json(run / 'parameters_manifest.json', manifest)
    return parameters, manifest


@lru_cache(maxsize=1)
def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def proc_identity(pid):
    proc = Path('/proc') / str(pid)
    try:
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] in ('Z', 'X'):
            return None
        return {'pid': pid, 'ticks': fields[19], 'uid': proc.stat().st_uid, 'boot_id': boot_id()}
    except (FileNotFoundError, ProcessLookupError):
        return None


def same_process(record):
    if not record:
        return False
    identity = proc_identity(record['pid'])
    return bool(identity and all(record.get(key) == value for key, value in identity.items()))


def signal_owned(record, sig):
    """Keep the kernel identity stable across the last validation and the signal."""
    if not same_process(record) or record['uid'] != os.getuid():
        return False
    try:
        fd = os.pidfd_open(record['pid'])
    except ProcessLookupError:
        return False
    try:
        if same_process(record):
            signal.pidfd_send_signal(fd, sig)
            return True
    except ProcessLookupError:
        pass
    finally:
        os.close(fd)
    return False


def append_bounded(path, data, limit=1024 * 1024):
    try:
        size = path.stat().st_size
    except FileNotFoundError:
        size = 0
    if size and size + len(data) > limit:
        path.replace(path.with_name(path.name + '.1'))
    with path.open('ab') as output:
        output.write(data)


class Children:
    def __init__(self, run, control=None):
        self.run = run
        self.control = control
        self.items = {}
        self.records = {}
        self.pumps = {}
        self.log_errors = {}

    def start(self, name, command, env, logs=None):
        log = (logs or self.run) / (name + '.log')
        command = list(map(str, command))
        proc = subprocess.Popen(command, cwd=self.run, env=env, stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                start_new_session=True)
        self.items[name] = proc
        self.records[name] = dict(proc_identity(proc.pid) or {'pid': proc.pid},
                                  command=command, started_host_ns=time.time_ns())
        def pump():
            # Drain even after a storage error so a producer cannot deadlock on stderr.
            output = None
            size = 0
            try:
                with proc.stdout:
                    while True:
                        data = os.read(proc.stdout.fileno(), 64 * 1024)
                        if not data:
                            break
                        if name not in self.log_errors:
                            try:
                                if output and size + len(data) > 4 * 1024 * 1024:
                                    output.close()
                                    output = None
                                    log.replace(log.with_name(log.name + '.1'))
                                    size = 0
                                if output is None:
                                    output = log.open('ab', buffering=0)
                                    size = output.tell()
                                output.write(data)
                                size += len(data)
                            except OSError as exc:
                                self.log_errors[name] = str(exc)
            except OSError as exc:
                self.log_errors[name] = str(exc)
            finally:
                if output is not None:
                    output.close()

        pump_thread = threading.Thread(target=pump, name=f'{name}-log', daemon=True)
        self.pumps[name] = pump_thread
        pump_thread.start()
        self.save()
        return proc

    def save(self):
        if self.control is not None:
            write_json(self.control, {'run': str(self.run), 'processes': self.records})
        write_json(self.run / 'processes.json', self.records)

    def stop(self, name, timeout, *, interrupt_ok=False):
        proc = self.items.get(name)
        if proc is None:
            return
        running = proc.poll() is None
        if running:
            proc.send_signal(signal.SIGINT)
        forced = False
        try:
            proc.wait(timeout)
        except subprocess.TimeoutExpired:
            forced = True
            proc.kill()
            proc.wait()
        self.pumps[name].join()
        self.records[name].update(returncode=proc.returncode, forced=forced,
                                  exited_host_ns=time.time_ns())
        self.save()
        allowed = (0, -signal.SIGINT, 130) if running and interrupt_ok else (0,)
        if forced or proc.returncode not in allowed:
            raise RuntimeError(f'{name} 退出码 {proc.returncode}，forced={forced}')

    def require_alive(self, names):
        for name in names:
            rc = self.items[name].poll()
            if rc is not None:
                raise RuntimeError(f'{name} 提前退出（{rc}），详见 {name}.log')
        if self.log_errors:
            raise OSError(f'写运行日志失败 {self.log_errors}')
