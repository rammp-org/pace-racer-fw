import serial, time, re, sys, numpy as np
# xtest.py <out> <every> <n> cmd@wait ... : setup cmds before 'ARM', capture armed at ARM, stop before dump
out,every,n=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]); cmds=sys.argv[4:]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wait=0.4):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.2)
for c in cmds:
    w=0.4
    if '@' in c: c,w=c.rsplit('@',1); w=float(w)
    if c=='ARM': c=f'cap {every} {n} 8'
    r=[l for l in cmd(c,w).splitlines() if l.startswith(('#','!'))]
    print('>',c,'|',(r or [''])[-1][:150],flush=True)
cmd('stop',0.6)
print([l for l in cmd('hrt',0.6).splitlines() if l.startswith('#hrt')])
num=r"(-?[\d.]+(?:e-?\d+)?)"; rows=[]
sp.write(b'capdump\n'); t=time.time(); b=b''
while time.time()-t<5+n/100:
    b+=sp.read(65536)
    if b'#capend' in b: break
for ln in b.decode(errors='replace').splitlines():
    m=re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
    if m: rows.append([float(x) for x in m.groups()])
print([l for l in cmd('s',0.5).splitlines() if l.startswith(('#stats','!'))])
sp.close(); a=np.array(rows); np.save(out,a)
if len(a):
    enc,hat=np.radians(a[:,0]),np.radians(a[:,2]); t=np.arange(len(a))*every/20000
    err=np.degrees(np.angle(np.exp(1j*(hat-enc))))
    mech=np.degrees(np.unwrap(enc))/2
    for i in range(0,len(a),max(1,len(a)//15)):
        print(f'  t {t[i]:5.2f}s rotor {mech[i]-mech[0]:7.1f} deg mech  hat-enc {err[i]:+6.1f} deg')
