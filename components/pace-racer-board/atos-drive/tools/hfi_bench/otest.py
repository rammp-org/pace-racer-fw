import serial, time, re, sys, numpy as np
# otest.py <out.npy> <every> <rows> <mode> cmd@wait ... ; 'ARM' arms the capture (mode 5 or 9) in sequence.
# Always stops the drive before capdump (console output while driving trips the DRV), then prints
# the hrt status line and hat-enc / meas-enc statistics. Mode 9 rows: [enc, meas, hat], [ifa, ifb, w_hat].
out,every,rows,mode=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]); cmds=sys.argv[5:]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wait=0.4):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.2)
for c in cmds:
    w=0.4
    if '@' in c: c,w=c.rsplit('@',1); w=float(w)
    if c=='ARM': c=f'cap {every} {rows} {mode}'
    r=[l for l in cmd(c,w).splitlines() if l.startswith(('#','!'))]
    print('>',c,'|',(r or [''])[-1][:150],flush=True)
cmd('stop',0.6)
hrt=[l for l in cmd('hrt',0.6).splitlines() if l.startswith('#hrt')]
print(hrt)
num=r"(-?[\d.]+(?:e-?\d+)?)"; data=[]
sp.write(b'capdump\n'); t=time.time(); b=b''
while time.time()-t<5+rows/100:
    b+=sp.read(65536)
    if b'#capend' in b: break
for ln in b.decode(errors='replace').splitlines():
    m=re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
    if m: data.append([float(x) for x in m.groups()])
print([l for l in cmd('s',0.5).splitlines() if l.startswith(('#stats','!'))])
sp.close(); a=np.array(data); np.save(out,a)
if mode==9: a=a[:len(a)//2*2].reshape(-1,6)
if len(a) and mode in (5,9):
    enc,meas,hat=a[:,0],a[:,1],a[:,2]
    he=(hat-enc+180)%360-180; me=(meas-enc+90)%180-90
    dt=every/20000
    span=np.degrees(np.ptp(np.unwrap(np.radians(enc))))/2
    trip=np.where(abs(he)>60)[0]
    print(f'n={len(a)} {len(a)*dt:.2f}s rotor span {span:.0f} deg mech | hat-enc mean {he.mean():+.1f} sd {he.std():.1f} max {abs(he).max():.1f}'
          f' | meas-enc(mod180) mean {me.mean():+.1f} sd {me.std():.1f} | first >60 at {trip[0]*dt if len(trip) else -1:.3f}s')
