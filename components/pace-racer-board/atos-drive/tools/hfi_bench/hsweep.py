import serial, time, re, sys, cmath, math
# hsweep.py <th_align|-> <reps> <div> <smed> Vh...
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
num=r"(-?[\d.]+(?:e-?\d+)?)"
def hrot(vh,ms,div):
    cmd(f'hrot {vh} {ms} {div}',0.2); time.sleep(ms/1000+0.3); rows=[]
    for ln in cmd('capdump',1.2).splitlines():
        m=re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
        if m: rows.append([float(x) for x in m.groups()])
    return [complex(*r[:2]) for r in rows]
th,reps,div,smed=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]); vs=[float(x) for x in sys.argv[5:]]
cmd('sq 1',0.3); cmd(f'smed {smed}',0.3)
if th!='-': cmd(f'th {th}',0.2); cmd('id 3',0.6); cmd('id 0',0.4)
print(re.search(r"elec=\S+",cmd('enc',0.3)).group(0), f'div {div} smed {smed}')
f=lambda z: f'{abs(z):.4f}<{math.degrees(cmath.phase(z)):+6.1f}'
for v in vs:
    for r in range(reps):
        R=hrot(v,800,div); s=cmd('s',0.3); st=re.search(r"rej=\S+.*?max\|A\|=\S+",s)
        out=f'Vh {v:.3f} ab: P {f(R[0])} N {f(R[1])} N/P {abs(R[1])/abs(R[0]):.3f}'
        if len(R)>=4 and abs(R[2])>0: out+=f' | fresh: P {f(R[2])} N {f(R[3])} N/P {abs(R[3])/abs(R[2]):.3f} 2th {(math.degrees(cmath.phase(R[3])+cmath.phase(R[2]))+180)%360:.0f}'
        print(out, f'[{st.group(0) if st else "?"}]', flush=True)
print(re.search(r"elec=\S+",cmd('enc',0.3)).group(0)); cmd('stop',0.4); cmd('smed 1',0.2); sp.close()
