import serial, time, re, sys, numpy as np
vh=sys.argv[1] if len(sys.argv)>1 else '0.08'; bias=sys.argv[2] if len(sys.argv)>2 else '2'; A=sys.argv[3] if len(sys.argv)>3 else '0'
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.3); print(cmd('dtc 0',0.3).strip()[-50:]); cmd(f'th {A}',0.3); cmd(f'id {bias}',0.6)
sp.write(f'hrot {vh} 1500\ncap 1 400 1\n'.encode()); time.sleep(0.5)
txt=cmd("capdump",3.0); rows=[]
# drop the settle (first ~40 rows)

for ln in txt.splitlines():
    m=re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)
    if m: rows.append([float(x) for x in m.groups()])
time.sleep(1.2); cmd('id 0',0.3); sp.close()
print(txt[:300]) if len(rows)<10 else None
r=np.array(rows); ia=r[:,0]; ib=r[:,1]; va=r[:,2]; n=len(r); print('rows',n)
print('va:', ' '.join('%+.3f'%x for x in va[:24]))
print('ia:', ' '.join('%+.2f'%x for x in ia[:24]))
print('ib:', ' '.join('%+.2f'%x for x in ib[:24]))
i=ia-ia.mean()+1j*(ib-ib.mean())
# find the injection phase index from va pattern: va = Vh*cos(phi), phi = 90*k
vac=va-va.mean(); k=np.arange(n)
best=None
for off in range(4):
    ph=np.radians(90*((k+off)%4)); c=np.dot(vac,np.cos(ph))
    if best is None or c>best[0]: best=(c,off)
off=best[1]; ph=np.radians(90*((k+off)%4)); e=np.exp(1j*ph)
P=np.mean(i*np.conj(e)); N=np.mean(i*e)
print(f'phase offset {off}; |P| {abs(P):.3f} argP {np.degrees(np.angle(P)):+.1f} | |N| {abs(N):.3f} argN {np.degrees(np.angle(N)):+.1f} | |N|/|P| {abs(N)/abs(P):.3f} | 2θ {(np.degrees(np.angle(N)+np.angle(P))+180)%360:.1f}')
# spectrum of the complex current: look at +fs/4 and -fs/4 bins and others
F=np.fft.fft(i)/n; fr=np.fft.fftfreq(n,1/20000)
order=np.argsort(-np.abs(F))[:8]
print('top complex-spectrum bins (Hz: amp A):', ', '.join('%+.0f:%.3f'%(fr[j],abs(F[j])) for j in order))
print('i p-p alpha %.2f beta %.2f; mean alpha %.2f beta %.2f'%(ia.max()-ia.min(), ib.max()-ib.min(), ia.mean(), ib.mean()))
