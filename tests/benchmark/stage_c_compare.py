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
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
summary = []
for label, executable in (entry.split('=', 1) for entry in args.solver):
    path = args.output / (label + '.csv')
    subprocess.run([executable, args.urdf, str(path), str(args.samples), str(args.timeout)], check=True, timeout=240)
    groups = {}
    for row in csv.DictReader(path.open()):
        key = tuple(int(row[k]) for k in ['backend', 'scenario', 'mode'])
        groups.setdefault(key, []).append(row)
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
