from pathlib import Path
import subprocess, time
root=Path('/tmp/pin-ik-stage-c')

def run(label, command):
    print(time.strftime('%H:%M:%S'), 'START', label, flush=True)
    with (root/(label+'.log')).open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    print(time.strftime('%H:%M:%S'), 'DONE', label, flush=True)

versions=['b','c1','c2','c']
for v in versions:
    run('arm6-probe-build-'+v, ['cmake','--build',str(root/('bench-'+v)),'--target','stage_c_probe','-j2'])
for name, samples, timeout, order in [('ablation-arm6-wide',100,'.02',versions),('ablation-arm6-short-1',500,'.00055',versions),('ablation-arm6-short-2',500,'.00055',list(reversed(versions)))]:
    cmd=['python3','tests/benchmark/stage_c_compare.py','--samples',str(samples),'--timeout',timeout,'--urdf','tests/benchmark/arm6.urdf','--output',str(root/name)]
    for v in order: cmd += ['--solver',f'{v}={root}/bench-{v}/stage_c_probe']
    run(name, cmd)
paths={v:str(root/('bench-'+v)/'ik_benchmark') for v in versions}
paths['a']='/tmp/pin-ik-stage-a/bench-a/ik_benchmark'
paths['kdl']='/tmp/pin-ik-stage-a/bench-kdl/ik_benchmark'
for suite, order, repeat in [('performance',['b','c1','c2','c','a','kdl'],1),('performance',['kdl','a','c','c2','c1','b'],2),('periodic',['b','c','a','kdl'],1),('periodic',['kdl','a','c','b'],2)]:
    name=f'{suite}-{repeat}'
    cmd=['python3','tests/benchmark/compare.py','--suite',suite,'--output',str(root/name)]
    for v in order: cmd += ['--solver',f'{v}={paths[v]}']
    run(name,cmd)
