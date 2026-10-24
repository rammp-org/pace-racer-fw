import serial, time, re, sys, numpy as np
vh=sys.argv[1] if len(sys.argv)>1 else '0.1'; bias=sys.argv[2] if len(sys.argv)>2 else '0'
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.3); print(cmd('th 0',0.4).strip()[-80:]); print(cmd(f'hfi {vh} {bias}',0.6).strip()[-120:]); time.sleep(0.3)
print(cmd('cap 1 240 1',0.5).strip()[-120:]); time.sleep(0.5)
txt=cmd('capdump',3.0); rows=[]
for ln in txt.splitlines():
    m=re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)
    if m: rows.append([float(x) for x in m.groups()])
print(cmd('hfi 0',0.4).strip()[-60:]); print(cmd('s',0.8).strip()[-200:]); sp.close()
r=np.array(rows); ia=r[:,0]; ib=r[:,1]; va=r[:,2]; n=len(r); print('rows',n)
print('va  :', ' '.join('%+.3f'%x for x in va[:40]))
print('ia  :', ' '.join('%+.2f'%x for x in ia[:40]))
print('dia :', ' '.join('%+.2f'%x for x in np.diff(ia)[:40]))
# correlate dia(k) with va(k-lag) for lags 0..4
dia=np.diff(ia); vinj=va-va.mean()
for lag in range(0,5):
    x=vinj[:n-1-lag] if lag>0 else vinj[:n-1]; y=dia[lag:] if lag>0 else dia
    m=min(len(x),len(y)); c=np.corrcoef(x[:m],y[:m])[0,1]; g=np.dot(x[:m],y[:m])/np.dot(x[:m],x[:m])
    print(f'lag {lag}: corr(dia(k), va(k-{lag})) = {c:+.3f}, gain {g:+.2f} A/V -> Ts/L: L={50e-6/max(abs(g),1e-9)*1e6:.1f} uH')
print('ia mean %.2f  p-p %.2f'%(ia.mean(), ia.max()-ia.min()))
