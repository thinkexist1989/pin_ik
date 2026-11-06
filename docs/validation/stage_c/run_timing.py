from pathlib import Path
import subprocess
root=Path('/tmp/pin-ik-stage-c')
loads=[]
try:
    for _ in range(6):
        loads.append(subprocess.Popen(['python3','-c','x=1\nwhile True:\n x=(x*1664525+1013904223)&0xffffffff']))
    for v in ['b','c']:
        cmd=[str(root/f'bench-{v}/timing_probe'),str(root/f'timing-{v}.csv'),'10000','.00055','0','smooth','arm6','1000']
        print('START',v,flush=True)
        subprocess.run(cmd,check=True)
finally:
    for p in loads: p.terminate()
    for p in loads: p.wait()
    print('All six load processes reaped',flush=True)
