#!/usr/bin/env python3
"""Run explicit thread configurations; retain real OS settings and failed runs."""
import argparse
import csv
import hashlib
import json
import math
import platform
import statistics
import subprocess
from pathlib import Path


def percentile(values, percent):
    return sorted(values)[max(0, math.ceil(len(values)*percent/100)-1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--config', action='append', required=True, help='label=threads.conf')
    parser.add_argument('--output', type=Path, required=True, help='New directory (must not exist)')
    parser.add_argument('--samples', type=int, default=10000)
    parser.add_argument('--timeout', type=float, default=.00055)
    parser.add_argument('--period-us', type=int, default=1000)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--robot', choices=['cartesian', 'arm6'], default='arm6')
    parser.add_argument('--scenario', choices=['smooth', 'exact', 'jump', 'limits', 'singular', 'unreachable'], default='smooth')
    parser.add_argument('--mode', type=int, choices=range(5), default=0)
    args = parser.parse_args()
    if args.samples <= 0 or args.rounds <= 0 or args.period_us < 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Samples, rounds and timeout must be positive; period must be nonnegative')
    configurations = {}
    for value in args.config:
        if '=' not in value:
            parser.error('Expected --config label=path')
        label, path = value.split('=', 1)
        if not label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in label) or label in configurations:
            parser.error('Configuration labels must be unique simple filenames')
        try:
            configurations[label] = Path(path).read_text()
        except OSError as error:
            parser.error(str(error))
    executable = args.executable.resolve(strict=True)
    if args.output.exists():
        parser.error('Output directory already exists; choose a new directory to preserve evidence')
    args.output.mkdir(parents=True, exist_ok=False)
    for label, content in configurations.items():
        (args.output/(label+'.conf')).write_text(content)
    environment = dict(kernel=platform.release(), machine=platform.machine(), executable=str(executable),
                       executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                       samples=args.samples, timeout=args.timeout, period_us=args.period_us,
                       robot=args.robot, scenario=args.scenario, mode=args.mode)
    (args.output/'environment.json').write_text(json.dumps(environment, indent=2)+'\n')
    records = []
    input_reference = None
    failed = False
    for repeat in range(args.rounds):
        order = list(configurations)
        if repeat % 2:
            order.reverse()
        for label in order:
            output = args.output/f'round{repeat+1}_{label}.csv'
            command = [str(executable), str(output), str(args.samples), str(args.timeout), str(args.mode),
                       args.scenario, args.robot, str(args.period_us), str(args.output/(label+'.conf'))]
            record = dict(round=repeat+1, config=label, command=command)
            # This is an experiment watchdog, not an IK or real-time deadline.
            watchdog = max(180, args.samples*max(args.period_us/1e6, args.timeout)+60)
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=watchdog)
                (output.with_suffix('.log')).write_text(result.stdout+result.stderr)
                record['return_code'] = result.returncode
                if result.returncode:
                    record.update(status='unavailable' if result.returncode == 77 else 'failed', error=result.stderr.strip())
                    failed = True
                else:
                    with output.open() as stream:
                        times, success, deadline_misses, position, rotation = [], 0, 0, 0., 0.
                        inputs = hashlib.sha256()
                        for row in csv.DictReader(stream):
                            times.append(float(row['us']))
                            success += int(row['rc']) >= 0
                            deadline_misses += float(row['release_to_finish_us']) > 1000
                            if int(row['rc']) >= 0:
                                position = max(position, float(row['position_error']))
                                rotation = max(rotation, float(row['rotation_error']))
                            inputs.update((','.join(row[key] for key in row if key.startswith(('seed', 'target')))+'\n').encode())
                    if len(times) != args.samples or not all(math.isfinite(t) and t >= 0 for t in times):
                        raise ValueError('Invalid sample count or timing')
                    digest = inputs.hexdigest()
                    if input_reference is None:
                        input_reference = digest
                    if digest != input_reference:
                        raise ValueError('Input sequences differ across configurations')
                    settings = json.loads(Path(str(output)+'.threads.json').read_text())
                    if not settings['caller_restored']:
                        raise ValueError('Caller configuration was not restored')
                    record.update(status='measured', samples=len(times), success=success, mean_us=statistics.mean(times),
                                  p50_us=percentile(times, 50), p95_us=percentile(times, 95), p99_us=percentile(times, 99),
                                  p999_us=percentile(times, 99.9), max_us=max(times), over_budget=sum(t > args.timeout*1e6 for t in times),
                                  over_1ms=sum(t > 1000 for t in times), release_deadline_misses=deadline_misses,
                                  max_position_error=position, max_rotation_error=rotation, inputs_sha256=digest, settings=settings)
            except (subprocess.TimeoutExpired, ValueError, OSError) as error:
                record.update(status='failed', error=str(error))
                failed = True
            records.append(record)
            (args.output/'summary.json').write_text(json.dumps(records, indent=2)+'\n')
            print(label, repeat+1, record['status'], 'success=', record.get('success'), 'max_us=', record.get('max_us'), flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
