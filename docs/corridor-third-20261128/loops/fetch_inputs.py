"""Copy only the explicitly named third-run records from the verified SSH alias."""
import argparse
import hashlib
import json
import shlex
import subprocess
import tarfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIELD = '/data/robot/maps/fast_livo2/runs/corridor03_20261128T072245Z_tzunG9'
RUN = '20261128T072648Z_c05694da93'
FULL = 'full_z_5R7iGB'
DATA_SUBDIR = Path('data/corridor_runs/20261128_third_mapping')


def checkout_paths(start):
    """Find either the checkout containing the report or the workspace's adjacent algorithm."""
    for parent in start.parents:
        for candidate in (parent, parent/'livo-pgo-dev'):
            if (candidate/'backend/frame_io.py').is_file() and (candidate/'tools/evaluate_z_geometry.py').is_file():
                return candidate, parent/DATA_SUBDIR
    raise FileNotFoundError(f'Cannot locate the algorithm checkout above {start}')


ALGORITHM_ROOT, DEFAULT_DATA = checkout_paths(HERE)


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def collect(alias, data=DEFAULT_DATA):
    plan = json.loads((data / 'pgo/plan.json').read_text())
    ssh = ['ssh', '-o', 'StrictHostKeyChecking=yes', '-o', 'BatchMode=yes', alias]
    identity = subprocess.check_output(ssh + ['id -un; hostname; uname -m'], text=True).splitlines()
    if identity != ['orangepi', 'orangepi5ultra', 'aarch64']:
        raise RuntimeError('SSH target differs from the verified target')
    ids = sorted({k for i, j in plan['holdout_pairs_indices'] for k in (i, j-1, j, j+1)})
    files = [f'{RUN}/pgo/{name}' for name in (
        'graph.jsonl', 'issued.csv', 'edges.json', 'factors.json', 'summary.json', 'online_summary.json',
        'causal_noise.json', 'trajectory_before.tum', 'trajectory_after.tum', 'map_to_odom.tum')]
    files += [f'{RUN}/keyframes/clouds/{i}.pcd' for i in ids]
    files += [f'{RUN}/keyframes/index.csv', f'{RUN}/keyframes/metadata.txt', f'{RUN}/gravity_events.jsonl',
              f'{FULL}/origin_ns.txt', f'{FULL}/static_windows.txt']
    names = ' '.join(map(shlex.quote, files))
    manifest = subprocess.check_output(ssh + [f'cd {shlex.quote(FIELD)} && sha256sum -- {names}'], text=True)
    expected = {name.strip(): digest for digest, name in (row.split(maxsplit=1) for row in manifest.splitlines())}
    if set(expected) != set(files):
        raise RuntimeError('incomplete source manifest')
    dest = HERE/'raw'
    with subprocess.Popen(ssh+[f'cd {shlex.quote(FIELD)} && tar -cf - -- {names}'], stdout=subprocess.PIPE) as proc:
        with tarfile.open(fileobj=proc.stdout, mode='r|') as archive:
            for member in archive:
                if not member.isfile() or member.name not in expected:
                    raise ValueError(member.name)
                path = dest/member.name
                path.parent.mkdir(parents=True, exist_ok=True)
                content = archive.extractfile(member).read()
                if hashlib.sha256(content).hexdigest() != expected[member.name]:
                    raise ValueError('hash mismatch '+member.name)
                path.write_bytes(content)
        if proc.wait():
            raise RuntimeError('source transfer failed')
    if not all(sha(dest/name) == digest for name, digest in expected.items()):
        raise RuntimeError('destination readback failed')
    (HERE/'input_sha256.json').write_text(json.dumps(expected, indent=2)+'\n', encoding='utf8')
    print(len(files), 'files;', len(ids), 'native geometry frames; verified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssh', default='opiss-usb')
    parser.add_argument('--data', type=Path, default=DEFAULT_DATA, help='Third-run directory containing pgo/ and structure/')
    args = parser.parse_args()
    collect(args.ssh, args.data)
