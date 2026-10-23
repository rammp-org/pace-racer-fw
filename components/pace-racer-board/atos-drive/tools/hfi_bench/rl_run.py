import serial, time, re, socket, struct, numpy as np, sys
levels=[float(x) for x in sys.argv[1:]] or [0.1,0.2,0.3]
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
def cmd(c,wait=1.0):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wait: b+=sp.read(8192)
    return b.decode(errors='replace')
def capdump():
    txt=cmd('capdump',4.0); rows=[]
    for ln in txt.splitlines():
        m=re.match(r"^@\d+,(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?),(-?[\d.]+(?:e-?\d+)?)\s*$", ln)
        if m: rows.append([float(x) for x in m.groups()])
    return np.array(rows), txt
cmd('sq 1',0.3); print(cmd('e 1',0.8).strip()[-200:])
s=socket.create_connection(("192.168.1.101",5555),timeout=20, source_address=("192.168.1.50",0)); f=s.makefile("rb")
def w(c): s.sendall((c+"\n").encode()); time.sleep(0.12)
def q(c): w(c); return f.readline().decode().strip()
def block():
    assert f.read(1)==b"#"; n=int(f.read(1)); ln=int(f.read(n)); d=f.read(ln); f.readline(); return d
results=[]
for i,v in enumerate(levels):
    if i==len(levels)-1:
        w('*CLS'); w(':STOP'); w(':TIM:MODE MAIN'); w(':CHAN1:SCAL 0.05'); w(':CHAN1:OFFS -0.1'); w(':TIM:SCAL 200e-6'); w(':TIM:OFFS 600e-6'); w(':ACQ:MDEP 100k')
        w(':TRIG:EDGE:SOUR CHAN1'); w(':TRIG:EDGE:SLOP POS'); w(':TRIG:EDGE:LEV 0.01'); w(':TRIG:SWE SING'); w(':SING'); time.sleep(1.0); print('scope armed',q(':TRIG:STAT?'))
    r=cmd(f'rl {v} 10',1.5); print(r.strip()[-160:])
    rows,txt=capdump()
    if len(rows)==0: print('capdump raw:',txt[:400]); break
    ia=rows[:,0]; va=rows[:,2]; n=len(ia); on=va>0.5*v
    if on.sum()<5: print('  no on-samples; va range',va.min(),va.max()); continue
    i_ss=ia[on][-40:].mean() if on.sum()>60 else ia[on].mean()
    k0=np.argmax(on); k63=k0+np.argmax(ia[k0:]>0.632*i_ss)
    results.append((v,i_ss,(k63-k0)*50.0))
    print(f'  v={v:.2f} V rows={n} on={on.sum()} I_ss={i_ss:.2f} A t63~{(k63-k0)*50} us first on-samples: {np.round(ia[k0:k0+8],2).tolist()}')
    time.sleep(0.5)
print(cmd('s',1.0).strip()[-300:]); sp.close()
st=q(':TRIG:STAT?'); print('scope',st)
if st=='STOP':
    time.sleep(0.5); w(':STOP'); w(':WAV:SOUR CHAN1'); w(':WAV:MODE RAW'); w(':WAV:FORM WORD')
    pre=q(':WAV:PRE?').split(','); pts=int(pre[2]); xinc,xorig,xref=map(float,pre[4:7]); yinc,yorig,yref=map(float,pre[7:10])
    codes=[]; got=0
    while got<pts:
        nn=min(250000,pts-got); w(f':WAV:STAR {got+1}'); w(f':WAV:STOP {got+nn}'); w(':WAV:DATA?'); d=block(); c=struct.unpack('<%dH'%(len(d)//2),d); codes.extend(c); got+=len(c)
    a=(np.array(codes,float)-yorig-yref)*yinc*100; t=(xorig+(np.arange(len(a))-xref)*xinc)*1e6
    np.savez(f'/tmp/hfi/rl_{levels[-1]}.npz',t=t,a=a); print('scope pts',got,'dt %.3f us'%(xinc*1e6),'peak %.2f A'%a.max(),'err',q(':SYST:ERR?'))
    bs=int(50/(xinc*1e6)); m=len(a)//bs; env=a[:m*bs].reshape(m,bs).mean(1); tt=t[:m*bs:bs]
    sel=(tt>-200)&(tt<1200); print('  50us-bin mean (A):', ' '.join('%.1f'%x for x in env[sel]))
    k=np.argmax(a>1.0); seg=a[k-int(2/(xinc*1e6)):k+int(60/(xinc*1e6)):max(1,int(0.5/(xinc*1e6)))]; print('  first edge 0.5us:', ' '.join('%.1f'%x for x in seg))
w(':RUN'); s.close()
if len(results)>=2:
    V=np.array([r[0] for r in results]); I=np.array([r[1] for r in results]); p=np.polyfit(I,V,1)
    print(f'R fit (V/I slope over {len(V)} levels): {p[0]*1000:.1f} mohm, offset {p[1]*1000:.0f} mV (dead-time error)')
