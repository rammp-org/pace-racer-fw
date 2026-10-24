import serial, time, re, sys, numpy as np
args=sys.argv[1:]  # pairs "vh bias"
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
    return np.array(rows), txt
cmd('sq 1',0.3); print(cmd('hfig 1000 20000 0 1 2',0.5).strip()[-120:]); print(cmd('dtc '+(sys.argv[1] if False else '0.13')+' 0.3',0.5).strip()[-80:])
for i in range(0,len(args),2):
    vh,bias=args[i],args[i+1]
    print(cmd(f'hscan {vh} {bias}',0.8).strip()[-200:])
    time.sleep(2.6)
    for _ in range(10):
        st=cmd('hfi',0.5); m=re.search(r'scan_left=(\d+)',st)
        if m and int(m.group(1))==0: break
        time.sleep(0.5)
    rows,txt=capdump()
    if len(rows)<30: print('capdump short:',txt[:300]); continue
    lagcode=np.floor(rows[:,0]/1000).astype(int); angdeg=rows[:,0]-1000*lagcode; ang=np.radians(angdeg); yd=rows[:,1]; yq=rows[:,2]
    # fit yq = A sin(2a) + B cos(2a) + C ; yd = D + E sin(2a) + F cos(2a)
    M=np.c_[np.sin(2*ang),np.cos(2*ang),np.ones_like(ang)]
    cq,_,_,_=np.linalg.lstsq(M,yq,rcond=None); cd,_,_,_=np.linalg.lstsq(M,yd,rcond=None)
    ampq=np.hypot(cq[0],cq[1]); ampd=np.hypot(cd[0],cd[1])
    # yq ~ K sin(2(theta_r - a)) = K( sin2θr cos2a - cos2θr sin2a ) -> cq[1]=K sin2θr, cq[0]=-K cos2θr
    th_r=0.5*np.degrees(np.arctan2(cq[1],-cq[0]))%180
    resq=yq-M@cq
    print(f'  Vh={vh} bias={bias}: yd mean {cd[2]:.4f} A/tick (-> L_d~{float(vh)*50e-6/max(cd[2],1e-6)*1e6:.2f} uH if lag right), yd 2θ amp {ampd:.4f}; yq 2θ amp {ampq:.4f} offset {cq[2]:.4f} resid rms {resq.std():.4f} | saliency (yd amp/yd mean) {ampd/max(abs(cd[2]),1e-9):.3f} | rotor d-axis est {th_r:.1f} deg (mod 180)')
    print('  angle: '+' '.join('%3.0f'%x for x in angdeg)); print('  lag  : '+' '.join('%3d'%x for x in lagcode))
    print('  yd   : '+' '.join('%+.3f'%x for x in yd))
    print('  yq   : '+' '.join('%+.3f'%x for x in yq))
print(cmd('r',0.8).strip()[-200:]); print(cmd('s',0.8).strip()[-260:])
sp.close()
