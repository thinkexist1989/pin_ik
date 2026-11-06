#!/usr/bin/env python3
"""Exercise configuration rejection and permission reporting of a stage D build."""
import argparse
import json
import os
import resource
import subprocess
import tempfile
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--executable', type=Path, required=True)
args = p.parse_args()
executable = args.executable.resolve(strict=True)
results = []
with tempfile.TemporaryDirectory(prefix='pin-ik-config-tests-') as directory:
    root = Path(directory)
    cpu = min(os.sched_getaffinity(0))
    cases = {'bad-policy': 'caller - keep -\nnewton - keep 1\nnlopt - keep -\n'}
    if not Path('/sys/devices/system/cpu/cpu1023').exists():
        cases['filtered-mask'] = f'caller - keep -\nnewton {cpu},1023 keep -\nnlopt - keep -\n'
    for name, content in cases.items():
        config = root/(name+'.conf')
        config.write_text(content)
        output = root/(name+'.csv')
        run = subprocess.run([str(executable), str(output), '20', '.00055', '0', 'smooth', 'arm6', '0', str(config)],
                             capture_output=True, text=True, timeout=30)
        assert run.returncode == 1 and not output.exists(), (name, run.returncode, run.stderr)
        if name == 'filtered-mask':
            assert 'readback' in run.stderr
        results.append(dict(case=name, status='passed', error=run.stderr.strip()))

    # CAP_SYS_NICE (Linux capability 23) bypasses RLIMIT_RTPRIO. Do not pretend
    # that reducing the limit forces EPERM when this capability is effective.
    capabilities = int(next(line.split()[1] for line in Path('/proc/self/status').read_text().splitlines()
                            if line.startswith('CapEff:')), 16)
    if capabilities & (1 << 23):
        results.append(dict(case='permission-matrix', status='skipped', reason='CAP_SYS_NICE is effective'))
    else:
        (root/'fifo.conf').write_text('caller - keep -\nnewton - fifo 1\nnlopt - fifo 1\n')
        (root/'default.conf').write_text('caller - keep -\nnewton - keep -\nnlopt - keep -\n')

        def deny_realtime():
            os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
            resource.setrlimit(resource.RLIMIT_RTPRIO, (0, 0))

        command = ['python3', str(Path(__file__).with_name('stage_d_compare.py')), '--executable', str(executable),
                   '--config', f'denied={root}/fifo.conf', '--config', f'baseline={root}/default.conf',
                   '--samples', '20', '--rounds', '1', '--period-us', '0', '--output', str(root/'results')]
        run = subprocess.run(command, capture_output=True, text=True, preexec_fn=deny_realtime, timeout=30)
        rows = json.loads((root/'results/summary.json').read_text())
        assert run.returncode == 1, run.stderr
        assert rows[0]['status'] == 'unavailable' and rows[0]['return_code'] == 77
        assert rows[1]['status'] == 'measured' and rows[1]['settings']['caller_restored']
        results.append(dict(case='permission-matrix', status='passed'))
print(json.dumps(results, indent=2))
