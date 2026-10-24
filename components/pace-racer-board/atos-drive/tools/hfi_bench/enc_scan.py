import serial, time, re, sys, numpy as np, cmath, math, json
vh=float(sys.argv[1]); ia=float(sys.argv[2]); ms=int(sys.argv[3]); step=float(sys.argv[4]); out=sys.argv[5]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
num=r"(-?[\d.]+(?:e-?\d+)?)"
def capdump():
    rows=[]
    for ln in cmd('capdump',1.5).splitlines():
        m=re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
        if m: rows.append([float(x) for x in m.groups()])
    return rows
def enc():
    t=cmd('enc',0.3); m=re.search(r"acc=(-?[\d.]+)deg.*elec=(-?[\d.]+)deg",t)
    return float(m.group(1)), float(m.group(2))
cmd('sq 1',0.3); cmd('dtc 0',0.3)
res=[]
for A in np.arange(0,360,step):
    cmd(f'th {A}',0.2); cmd(f'id {ia}',0.6); cmd('id 0',0.4)
    acc0,e0=enc()
    cmd(f'hrot {vh} {ms} 8',0.2); time.sleep(ms/1000+0.3)
    rows=capdump(); acc1,e1=enc()
    if len(rows)<4: print('short',A); continue
    P=complex(*rows[2][:2]); N=complex(*rows[3][:2])
    res.append(dict(A=float(A),e0=e0,e1=e1,acc0=acc0,acc1=acc1,P=[P.real,P.imag],N=[N.real,N.imag]))
    print(f'th {A:5.1f} enc {e0:6.1f}->{e1:6.1f} |P| {abs(P):.3f} argP {math.degrees(cmath.phase(P)):+6.1f} |N| {abs(N):.4f} argN {math.degrees(cmath.phase(N)):+6.1f}',flush=True)
cmd('id 0',0.2); cmd('stop',0.5)
json.dump(res,open(out,'w'))
th=np.radians([r['e0'] for r in res]); N=np.array([complex(*r['N']) for r in res]); P=np.array([complex(*r['P']) for r in res])
M=np.c_[np.ones_like(th),np.exp(2j*th),np.exp(-2j*th)]
c,*_=np.linalg.lstsq(M,N,rcond=None)
print(f'fit N = Ns + Nr e^(j2θ) + Nm e^(-j2θ): |Ns| {abs(c[0]):.4f} |Nr| {abs(c[1]):.4f} |Nm| {abs(c[2]):.4f}  resid rms {np.sqrt(np.mean(abs(N-M@c)**2)):.4f}  |P| mean {abs(P).mean():.3f}')
for k in (1,2,4,6):
    Mk=np.c_[np.ones_like(th),np.exp(1j*k*th),np.exp(-1j*k*th)]; ck,*_=np.linalg.lstsq(Mk,N,rcond=None)
    print(f'  harmonic {k}: |+| {abs(ck[1]):.4f} |-| {abs(ck[2]):.4f}')
sp.close()
