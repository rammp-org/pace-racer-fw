import serial, time, re, sys, numpy as np
# hrt_log.py <every> <n> <out.npy> cmd... ; runs cmds, arms cap mode 5, waits, dumps, prints error stats
every,n,out=int(sys.argv[1]),int(sys.argv[2]),sys.argv[3]; cmds=sys.argv[4:]
import os; MODE=int(os.environ.get('CAPMODE','5'))
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wait=0.4):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.2)
post=[]
for c in cmds:
    if c.startswith('POST:'): post.append(c[5:]); continue
    w=0.4
    if '@' in c: c,w=c.rsplit('@',1); w=float(w)
    print(cmd(c,w).strip().splitlines()[-1] if cmd else '')
cmd(f'cap {every} {n} {MODE}',0.2); time.sleep(every*n/20000*(3 if MODE==8 else 1)/(3 if MODE==8 else 1)+0.5)
cmd('stop',0.6)  # never dump while driving: console output stalls the sampler ISR (2026-10-23 trips)
num=r"(-?[\d.]+(?:e-?\d+)?)"; rows=[]
sp.write(b'capdump\n'); t=time.time(); b=b''
while time.time()-t<5+n/100:
    b+=sp.read(65536)
    if b'#capend' in b: break
for ln in b.decode(errors='replace').splitlines():
    m=re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
    if m: rows.append([float(x) for x in m.groups()])
for c in post: print(cmd(c,0.4).strip().splitlines()[-1])
sp.close(); a=np.array(rows); np.save(out,a)
enc,meas,hat=a[:,0],a[:,1],a[:,2]
w180=lambda d: (d+90)%180-90
em,eh=w180(meas-enc),w180(hat-enc)
print(f'n={len(a)} enc span {np.ptp(np.unwrap(np.radians(enc)))*57.3:.0f} deg | meas-enc mean {em.mean():+.1f} sd {em.std():.1f} p95 {np.percentile(abs(em-em.mean()),95):.1f} | hat-enc mean {eh.mean():+.1f} sd {eh.std():.1f} p95 {np.percentile(abs(eh-eh.mean()),95):.1f} max {abs(eh-eh.mean()).max():.1f} (deg elec, mod 180)')
