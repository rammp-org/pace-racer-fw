import serial, time, re, sys, numpy as np, cmath, math
vh=sys.argv[1]; bias=sys.argv[2]; ms=sys.argv[3]; pins=[float(x) for x in sys.argv[4:]] or [0,60,120]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
def capdump():
    txt=cmd('capdump',1.5); rows=[]
    for ln in txt.splitlines():
        m=re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)
        if m: rows.append([float(x) for x in m.groups()])
    return np.array(rows)
cmd('sq 1',0.3); print(cmd('dtc 0',0.4).strip()[-60:])
res=[]
for A in pins:
    cmd(f'th {A}',0.3); cmd('id 2',0.7)
    if float(bias)==0: cmd('id 0',0.4)   # align with 2 A, then release before measuring
    else: cmd(f'id {bias}',0.2)
    cmd(f'hrot {vh} {ms}',0.3); time.sleep(float(ms)/1000+0.4)
    rows=capdump()
    if len(rows)<2: print('short',rows); continue
    P=complex(rows[0,0],rows[0,1]); N=complex(rows[1,0],rows[1,1]); n=rows[0,2]
    th2=math.degrees(cmath.phase(N)+cmath.phase(P)+math.pi)%360
    print(f'pin {A:5.1f}: |P| {abs(P):.3f} A argP {math.degrees(cmath.phase(P)):+.1f}  |N| {abs(N):.4f} argN {math.degrees(cmath.phase(N)):+.1f}  |N|/|P| {abs(N)/abs(P):.3f}  2θ {th2:.1f} -> θ {th2/2:.1f} (expect {A%180:.0f})  n={int(n)}')
    res.append((A,P,N))
cmd('id 0',0.3)
if len(res)>=3:
    # N(A) = Ns + Nr * exp(j 2A) : least squares in complex
    A=np.array([r[0] for r in res]); N=np.array([r[2] for r in res]); P=np.array([r[1] for r in res])
    E=np.exp(1j*2*np.radians(A)); M=np.c_[np.ones_like(E),E]
    c=np.linalg.lstsq(M,N,rcond=None)[0]; Ns,Nr=c
    print(f'fit: stationary |Ns| {abs(Ns):.4f} arg {math.degrees(cmath.phase(Ns)):+.1f} | rotor |Nr| {abs(Nr):.4f} arg {math.degrees(cmath.phase(Nr)):+.1f} | residual {np.abs(N-M@c).mean():.4f} | saliency |Nr|/|P| {abs(Nr)/np.abs(P).mean():.3f}')
    for (a,p,nn) in res:
        th2=math.degrees(cmath.phase(nn-Ns)+cmath.phase(p)+math.pi)%360
        print(f'   pin {a:5.1f}: rotor-only 2θ {th2:.1f} -> θ {th2/2:.1f} (expect {a%180:.0f})')
print(cmd('r',0.6).strip()[-120:]); print(cmd('s',0.6).strip()[-200:]); sp.close()
