import socket, struct, time, sys, numpy as np, serial, re
# faultcap.py <lev_A> <tdiv_s> <wait_s> <tag> cmd... : arm phase-A single shot at |I|>lev, run console cmds, read RAW
lev,tdiv,wait,tag=float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),sys.argv[4]; cmds=sys.argv[5:]
s=socket.create_connection(("192.168.1.101",5555),timeout=20,source_address=("192.168.1.50",0)); f=s.makefile("rb")
def w(c): s.sendall((c+"\n").encode()); time.sleep(0.12)
def q(c): w(c); return f.readline().decode().strip()
def block():
    assert f.read(1)==b"#"; n=int(f.read(1)); ln=int(f.read(n)); d=f.read(ln); f.readline(); return d
vdiv=max(0.02,lev/100/3)
w(':STOP'); w(':TIM:MODE MAIN'); w(':CHAN1:UNIT VOLT'); w(':CHAN1:PROB 1'); w(f':CHAN1:SCAL {vdiv}'); w(':CHAN1:OFFS 0')
w(f':TIM:SCAL {tdiv}'); w(f':TIM:OFFS {3*tdiv}'); w(':ACQ:MDEP 1M')
w(':TRIG:MODE EDGE'); w(':TRIG:EDGE:SOUR CHAN1'); w(':TRIG:EDGE:SLOP POS'); w(f':TRIG:EDGE:LEV {lev/100}'); w(':TRIG:SWE SING'); w(':SING'); time.sleep(1.2)
print('armed',q(':TRIG:STAT?'),'srat',q(':ACQ:SRAT?'),flush=True)
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.read(65536)
def cmd(c,wt=0.4):
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<wt: b+=sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1',0.2)
for c in cmds:
    wt=0.4
    if '@' in c: c,wt=c.rsplit('@',1); wt=float(wt)
    print('>',c,(cmd(c,wt).strip().splitlines() or [''])[-1][:100],flush=True)
t0=time.time(); st=''
while time.time()-t0<wait:
    time.sleep(0.5); st=q(':TRIG:STAT?')
    if st=='STOP': print('triggered after %.1f s'%(time.time()-t0)); break
print('board:',[l for l in cmd('s',0.6).splitlines() if l.startswith(('#stats','!'))])
cmd('stop',0.5)
if st=='STOP':
    w(':STOP'); time.sleep(0.3); w(':WAV:SOUR CHAN1'); w(':WAV:MODE RAW'); w(':WAV:FORM WORD')
    pre=q(':WAV:PRE?').split(','); pts=int(pre[2]); xinc,xorig,xref=map(float,pre[4:7]); yinc,yorig,yref=map(float,pre[7:10])
    codes=[]; got=0
    while got<pts:
        n=min(250000,pts-got); w(f':WAV:STAR {got+1}'); w(f':WAV:STOP {got+n}'); w(':WAV:DATA?')
        d=block(); c=struct.unpack('<%dH'%(len(d)//2),d); codes.extend(c); got+=len(c)
    a=(np.array(codes,float)-yorig-yref)*yinc*100; t=(xorig+(np.arange(len(a))-xref)*xinc)*1e6
    np.savez(f'/tmp/hfi/{tag}.npz',t=t,a=a)
    print('span %.0f us dt %.3f us peak %.1f A min %.1f A'%(t[-1]-t[0],xinc*1e6,a.max(),a.min()))
else: print('NO TRIGGER, state',st)
w(':RUN'); s.close(); sp.close()
