from pathlib import Path
import subprocess, time
root=Path('/tmp/pin-ik-stage-c')
loads=[]
try:
    for _ in range(6):
        loads.append(subprocess.Popen(['python3','-c','x=1\nwhile True:\n x=(x*1664525+1013904223)&0xffffffff']))
    print('Six SCHED_OTHER integer-loop processes started', flush=True)
    paths={'b':root/'bench-b/ik_benchmark','c':root/'bench-c/ik_benchmark','a':Path('/tmp/pin-ik-stage-a/bench-a/ik_benchmark')}
    for suite in ['performance','periodic']:
        print(time.strftime('%H:%M:%S'), 'START',suite,flush=True)
        command=['python3','tests/benchmark/compare.py','--suite',suite,'--output',str(root/('load-'+suite))]
        for label,path in paths.items(): command += ['--solver',f'{label}={path}']
        with (root/('load-'+suite+'.log')).open('w') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        print(time.strftime('%H:%M:%S'),'DONE',suite,flush=True)
finally:
    for p in loads: p.terminate()
    for p in loads: p.wait()
    print('All six load processes reaped',flush=True)
