"""Exercise the actual mapper, LIO update, IMU propagation, VIO Jacobian and sync state machine."""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'runtime'))
from run_files import snapshot


LIMIT, OLD_LIMIT = 'lio.max_iterations', 'lio.min_iterations'
EXPOSURE, OLD_EXPOSURE = 'vio.exposure_estimate_en', 'imu.exposure_estimate_en'
ACC, GYR = 'imu.b_acc_cov', 'imu.b_gyr_cov'


def expect(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_numeric(result, limit, exposure, acc, gyr, exposure_cov):
    expect(result['limit_readback'] == result['limit_consumed'] == limit, 'LIO bound')
    expect(result['iterations_executed'] == min(limit, 2) and result['plane_matches'] > 0,
           'planar LIO must converge in two passes unless a bound of one stops it')
    expect(result['exposure_readback'] is exposure, 'canonical exposure readback')
    expect(result['gyr_bias_readback'] == gyr and result['acc_bias_readback'] == acc,
           'bias covariance readback')
    expect(result['photo_HtH_66'] > 0 if exposure else result['photo_HtH_66'] == 0,
           'photometric exposure Jacobian')
    dt2 = result['imu_dt'] ** 2
    for actual, scalar in [(result['imu_Q_gyr_bias'], gyr), (result['imu_Q_acc_bias'], acc)]:
        expect(len(actual) == 3 and all(math.isclose(axis, scalar * dt2, rel_tol=1e-9, abs_tol=1e-15)
                                       for axis in actual), f'bias noise {actual} expected {scalar}*dt²')
    expect(math.isclose(result['imu_Q_66'], exposure_cov * dt2 if exposure else 0, abs_tol=1e-15),
           'IMU exposure process noise')
    expect(result['sync_and_stop'], 'synchronization and shutdown')


def run_suite(fixture, mapping, directory):
    directory.mkdir(parents=True, exist_ok=False)
    formal, manifest = snapshot(directory, ROOT / 'configs')
    base = {key: value for key, value in formal.items()
            if key not in (LIMIT, OLD_LIMIT, EXPOSURE, OLD_EXPOSURE, ACC, GYR)}
    # Keep 1 vs 5 observable after early convergence. Also exercise an explicitly supplied
    # default in conflict with the legacy key (both must count as explicit inputs).
    cases = [
        ('default', {}, (5, True, .0001, .0001)),
        ('legacy_only', {OLD_LIMIT: 1, OLD_EXPOSURE: False}, (1, False, .0001, .0001)),
        ('canonical', {LIMIT: 1, EXPOSURE: False, ACC: .0037, GYR: .00083, 'lio.voxel_size': .75},
         (1, False, .0037, .00083)),
        ('equal_names', {LIMIT: 3, OLD_LIMIT: 3, EXPOSURE: False, OLD_EXPOSURE: False},
         (3, False, .0001, .0001)),
        ('conflicting_limit', {LIMIT: 5, OLD_LIMIT: 1}, ('lio.max_iterations=5', 'lio.min_iterations=1')),
        ('conflicting_exposure', {EXPOSURE: True, OLD_EXPOSURE: False},
         ('vio.exposure_estimate_en=true', 'imu.exposure_estimate_en=false')),
        ('formal', None, (5, False, .0001, .0001)),
    ]
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='177', OMP_NUM_THREADS='1')
    for key in ('FAST_LIVO2_GRAVITY_DIR', 'FAST_LIVO2_SESSION_ID', 'FAST_LIVO2_CONFIG_SHA256',
                'FAST_LIVO2_ALGORITHM_SHA256'):
        environment.pop(key, None)
    results = {}
    for name, changes, expected in cases:
        case_dir = directory / name
        (case_dir / 'Log').mkdir(parents=True)
        parameters = formal if changes is None else dict(base, **changes)
        params_file = directory / 'resolved.yaml' if changes is None else case_dir / 'params.yaml'
        if changes is not None:
            params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}), encoding='utf-8')
        conflict = name.startswith('conflicting')
        proc = subprocess.run([str(mapping if conflict else fixture), '--ros-args', '--params-file',
                               str(params_file)], env=environment, cwd=case_dir, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=30)
        (case_dir / 'process.log').write_text(proc.stdout, encoding='utf-8')
        if conflict:
            expect(proc.returncode == 2 and 'Conflicting explicit parameters' in proc.stdout
                   and all(item in proc.stdout for item in expected), f'{name}: {proc.stdout[-900:]}')
            results[name] = {'exit': proc.returncode, 'conflict': expected}
        else:
            expect(proc.returncode == 0, f'{name}: exit={proc.returncode}\n{proc.stdout[-1200:]}')
            line, = [line for line in proc.stdout.splitlines() if line.startswith('CONTRACT ')]
            result = json.loads(line[len('CONTRACT '):])
            expect(result['vio_root_size'] == result['lio_root_size'] == parameters['lio.voxel_size'],
                   'LIVMapper must pass the configured LiDAR root size to VIO')
            check_numeric(result, *expected, parameters['vio.inv_expo_cov'])
            for old in (OLD_LIMIT, OLD_EXPOSURE):
                if old in parameters:
                    expect(f'Deprecated parameter {old}' in proc.stdout, f'{name} missing deprecation')
            results[name] = result
        print(name, json.dumps(results[name], sort_keys=True), flush=True)
    report = {'resolved_sha256': manifest['resolved_sha256'], 'cases': results}
    (directory / 'contract_results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, default=ROOT / 'build/fast_livo/contract_regression')
    parser.add_argument('--mapping', type=Path, default=ROOT / 'install/fast_livo/lib/fast_livo/fastlivo_mapping')
    parser.add_argument('--output', type=Path, help='New directory for case YAML, separate logs and measured results')
    args = parser.parse_args()
    if args.output:
        run_suite(args.fixture.resolve(strict=True), args.mapping.resolve(strict=True), args.output.resolve())
    else:
        with tempfile.TemporaryDirectory(prefix='fast-livo-contract-') as tmp:
            run_suite(args.fixture.resolve(strict=True), args.mapping.resolve(strict=True), Path(tmp) / 'results')


if __name__ == '__main__':
    main()
