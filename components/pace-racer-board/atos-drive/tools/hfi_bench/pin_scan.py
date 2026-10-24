import serial, time, re, sys, numpy as np
vh=sys.argv[1]; bias=sys.argv[2]; pins=[float(x) for x in sys.argv[3:]] or [0,60,120]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
def capdump():
    txt=cmd('capdump',3.0); rows=[]
    for ln in txt.splitlines():
        m=re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)
        if m: rows.append([float(x) for x in m.groups()])
    return np.array(rows)
cmd('sq 1',0.3); print(cmd('hfig 1000 20000 0 1 2',0.4).strip()[-90:]); print(cmd('dtc 0.13 0.3',0.4).strip()[-60:])
for A in pins:
    cmd(f'th {A}',0.3); cmd(f'id {bias}',0.6)       # pin the rotor d-axis at A with the bias
    cmd(f'hscan {vh} {bias}',0.3)
    t0=time.time()
    while time.time()-t0<15:
        time.sleep(0.5); st=cmd('hfi',0.4); m=re.search(r'scan_left=(\d+)',st)
        if m and int(m.group(1))==0: break
    rows=capdump()
    if len(rows)<30: print('short capdump',len(rows)); continue
    lagcode=np.floor(rows[:,0]/1000).astype(int); angdeg=rows[:,0]-1000*lagcode; ang=np.radians(angdeg); yd=rows[:,1]; yq=rows[:,2]
    M=np.c_[np.sin(2*ang),np.cos(2*ang),np.ones_like(ang)]
    cq=np.linalg.lstsq(M,yq,rcond=None)[0]; cd=np.linalg.lstsq(M,yd,rcond=None)[0]
    ampq=np.hypot(cq[0],cq[1]); ampd=np.hypot(cd[0],cd[1]); resq=(yq-M@cq).std()
    th_q=0.5*np.degrees(np.arctan2(cq[1],-cq[0]))%180
    th_d=0.5*np.degrees(np.arctan2(cd[1],cd[0]))%180
    # 6th-harmonic (inverter) content for reference
    M6=np.c_[np.sin(6*ang),np.cos(6*ang)]; c6=np.linalg.lstsq(M6,yq-M@cq,rcond=None)[0]
    print(f'pin {A:5.1f}: yd mean {cd[2]:.3f} 2θamp {ampd:.3f} (min at {(th_d+90)%180:.1f}/max at {th_d:.1f}) | yq 2θamp {ampq:.3f} off {cq[2]:+.3f} resid {resq:.3f} 6θamp {np.hypot(*c6):.3f} -> d-axis est {th_q:.1f} (expect {A%180:.0f}) | lags {np.bincount(lagcode,minlength=4)[1:].tolist()}')
    print('   yd:', ' '.join('%+.2f'%x for x in yd)); print('   yq:', ' '.join('%+.2f'%x for x in yq))
cmd('id 0',0.3); print(cmd('r',0.6).strip()[-120:]); print(cmd('s',0.6).strip()[-200:]); sp.close()
