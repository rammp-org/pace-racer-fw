import serial, time, re, sys, numpy as np
amps=sys.argv[1]; rpm=sys.argv[2]; ramp=sys.argv[3]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(16384)
    return b.decode(errors='replace')
cmd('sq 1',0.3); cmd('ho 9999 9000 1',0.3); cmd('vl 6',0.3)
print(cmd(f'run {amps} {rpm} {ramp}',0.3).strip()[-100:]); time.sleep(float(ramp)+1.5)
# stop -> unload -> coast; then immediately re-engage HOLD at 0 A and capture 100 ms at 20 kHz
sp.write(b'stop\n'); time.sleep(0.35); sp.write(b'th 0\ncap 1 2000 1\n'); time.sleep(0.5)
txt=cmd('capdump',8.0); print(cmd('s',0.6).strip()[-160:]); print(cmd('r',0.5).strip()[-90:]); sp.close()
rows=[[float(x) for x in m.groups()] for ln in txt.splitlines() for m in [re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)] if m]
r=np.array(rows); print('rows',len(r))
if len(r)>100:
    ia=r[:,0]; ib=r[:,1]; va=r[:,2]; n=len(r)
    print('va: mean %.3f p-p %.3f std %.3f | ia p-p %.2f std %.2f'%(va.mean(),va.max()-va.min(),va.std(),ia.max()-ia.min(),ia.std()))
    F=np.fft.rfft(va-va.mean()); fr=np.fft.rfftfreq(n,1/20000); k=np.argmax(np.abs(F[1:]))+1
    print('dominant va component: %.0f Hz, amplitude %.3f V'%(fr[k], 2*abs(F[k])/n))
    print('va every 2.5 ms:', ' '.join('%+.2f'%x for x in va[::50]))
