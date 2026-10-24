import serial, time, re, sys, numpy as np
amps=sys.argv[1]; rpm=sys.argv[2]; ramp=sys.argv[3] if len(sys.argv)>3 else '8'
pp=7
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(16384)
    return b.decode(errors='replace')
cmd('sq 1',0.3); cmd('ho 9999 9000 1',0.3); cmd('vl 6',0.3)
print(cmd(f'run {amps} {rpm} {ramp}',0.5).strip()[-120:])
time.sleep(float(ramp)+1.5)
st=cmd('',0.1)
cmd('cap 1 1500 1',0.3); time.sleep(0.4); txt=cmd('capdump',6.0)
print(cmd('stop',1.0).strip()[-80:]); print(cmd('s',0.8).strip()[-200:]); print(cmd('r',0.5).strip()[-100:]); sp.close()
rows=[[float(x) for x in m.groups()] for ln in txt.splitlines() for m in [re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)] if m]
r=np.array(rows); ia=r[:,0]; ib=r[:,1]; va=r[:,2]; n=len(r); t=np.arange(n)/20000
fe=float(rpm)*pp/60
# fit sinusoids at fe to ia, ib, va
M=np.c_[np.cos(2*np.pi*fe*t),np.sin(2*np.pi*fe*t),np.ones(n)]
def fit(y):
    c=np.linalg.lstsq(M,y,rcond=None)[0]; return np.hypot(c[0],c[1]), np.degrees(np.arctan2(-c[1],c[0])), (y-M@c).std()
Ai,phi_i,ri=fit(ia); Av,phi_v,rv=fit(va); Ab,phi_b,rb=fit(ib)
ph=(phi_v-phi_i+180)%360-180
Ri=0.036*Ai
print(f'rows {n}, fe {fe:.1f} Hz: |ia| {Ai:.2f} A (resid {ri:.2f}), |ib| {Ab:.2f} A, |va| {Av:.3f} V (resid {rv:.3f}); v leads i by {ph:+.1f} deg')
print(f'  if the rotor follows with i on its d-axis: E = |va| sin(phase) = {Av*abs(np.sin(np.radians(ph))):.3f} V, R_eff*i = {Av*np.cos(np.radians(ph)):.3f} V (R*i expected {Ri:.3f} V + dead-time)')
we=2*np.pi*fe; print(f'  -> lambda = E/we = {Av*abs(np.sin(np.radians(ph)))/we*1e3:.3f} mWb ; wL*i = {we*3e-6*Ai:.4f} V (negligible)')
print('  va first cycle:', ' '.join('%+.2f'%x for x in va[:int(20000/fe):max(1,int(20000/fe/16))]))
print('  ia first cycle:', ' '.join('%+.2f'%x for x in ia[:int(20000/fe):max(1,int(20000/fe/16))]))
