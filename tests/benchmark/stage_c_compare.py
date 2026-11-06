#!/usr/bin/env python3
"""Compare stage C ablations on identical near-target and independent random seeds."""
import argparse
import csv
import json
import statistics
import subprocess
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--solver', action='append', required=True, help='label=/path/to/stage_c_probe')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--samples', type=int, default=100)
p.add_argument('--timeout', type=float, default=.00055)
p.add_argument('--urdf', default='tests/robot.urdf')
p.add_argument('--analyze-only', action='store_true', help='Summarize existing CSVs without rerunning solvers')
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
summary = []
measurements = {}
for label, executable in (entry.split('=', 1) for entry in args.solver):
    path = args.output / (label + '.csv')
    if not args.analyze_only:
        subprocess.run([executable, args.urdf, str(path), str(args.samples), str(args.timeout)], check=True, timeout=240)
    groups = {}
    for row in csv.DictReader(path.open()):
        key = tuple(int(row[k]) for k in ['backend', 'scenario', 'mode'])
        groups.setdefault(key, []).append(row)
    measurements[label] = groups
    for (backend, scenario, mode), rows in groups.items():
        assert len(rows) == args.samples
        times = sorted(float(r['us']) for r in rows)
        good = [r for r in rows if int(r['rc']) >= 0]
        evals = [int(r['last_optimize_evaluations']) for r in good if int(r['last_optimize_evaluations']) >= 0]
        record = dict(solver=label, backend='nlopt' if backend == 0 else 'dual',
                      scenario='independent_seed' if scenario else 'near', mode=mode,
                      samples=len(rows), success=len(good), timeout_seconds=args.timeout,
                      mean_us=statistics.mean(times), max_us=times[-1], p99_us=times[max(0,(len(times)*99+99)//100-1)],
                      over_budget=sum(t>args.timeout*1e6 for t in times), over_1ms=sum(t>1000 for t in times),
                      mean_candidates=statistics.mean(int(r['candidates']) for r in rows),
                      mean_best_score=statistics.mean(float(r['best_score']) for r in good) if good else None,
                      mean_last_optimize_evaluations_success=statistics.mean(evals) if evals else None)
        summary.append(record)
        print(json.dumps(record), flush=True)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

# Only compare quality/evaluations on the same cases solved by both versions.
# A mean over all successes alone can hide different failure populations.
paired = []
labels = list(measurements)
for before, after in zip(labels, labels[1:]):
    for key, old_rows in measurements[before].items():
        new_rows = measurements[after][key]
        assert len(old_rows) == len(new_rows)
        both = []
        for old, new in zip(old_rows, new_rows):
            assert old['case'] == new['case']
            if int(old['rc']) >= 0 and int(new['rc']) >= 0:
                both.append((old, new))
        record = dict(before=before, after=after, backend='nlopt' if key[0] == 0 else 'dual',
                      scenario='independent_seed' if key[1] else 'near', mode=key[2], both_solved=len(both))
        if key[0] == 0 and both:
            for name, index in [('before', 0), ('after', 1)]:
                record[name+'_mean_evaluations'] = statistics.mean(int(pair[index]['last_optimize_evaluations']) for pair in both)
        elif key[0] == 1 and key[2] > 0:
            improved = worse = 0
            for old, new in both:
                a, b = float(old['best_score']), float(new['best_score'])
                tolerance = 1e-12 + 1e-4*max(abs(a), abs(b))
                benefit = a-b if key[2] == 1 else b-a
                improved += benefit > tolerance
                worse += benefit < -tolerance
            record.update(improved=improved, worse=worse, equivalent=len(both)-improved-worse)
        paired.append(record)
(args.output/'paired-summary.json').write_text(json.dumps(paired,indent=2)+'\n')
