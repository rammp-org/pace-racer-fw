#!/usr/bin/env python3
"""Arm scope single-shot on phase A, reset the board (or just wait), fetch RAW, print pulses + bootlog.
usage: bootcap.py [--vdiv 0.2] [--tdiv 20e-6] [--toffs 60e-6] [--lev 0.03] [--mdep 100k] [--noreset] [--tag name]"""
import socket, struct, time, subprocess, argparse, sys, numpy as np, glob
ap=argparse.ArgumentParser(); ap.add_argument('--vdiv',type=float,default=0.2); ap.add_argument('--tdiv',type=float,default=20e-6)
ap.add_argument('--toffs',type=float,default=60e-6); ap.add_argument('--lev',type=float,default=0.03); ap.add_argument('--mdep',default='100k')
ap.add_argument('--noreset',action='store_true'); ap.add_argument('--tag',default='cap'); ap.add_argument('--wait',type=float,default=25)
ap.add_argument('--nobootlog',action='store_true'); ap.add_argument('--cmd',default='')
A=ap.parse_args()
ATOS='/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/components/pace-racer-board/atos-drive'
s=socket.create_connection(("192.168.1.101",5555),timeout=20, source_address=("192.168.1.50",0)); f=s.makefile("rb")
def w(c): s.sendall((c+"\n").encode()); time.sleep(0.12)
def q(c): w(c); return f.readline().decode().strip()
def block():
    assert f.read(1)==b"#"; n=int(f.read(1)); ln=int(f.read(n)); d=f.read(ln); f.readline(); return d
w(':STOP'); w(':TIM:MODE MAIN'); w(':CHAN1:UNIT VOLT'); w(':CHAN1:PROB 1'); w(f':CHAN1:SCAL {A.vdiv}'); w(f':CHAN1:OFFS {-2*A.vdiv}')
w(f':TIM:SCAL {A.tdiv}'); w(f':TIM:OFFS {A.toffs}'); w(f':ACQ:MDEP {A.mdep}')
w(':TRIG:MODE EDGE'); w(':TRIG:EDGE:SOUR CHAN1'); w(':TRIG:EDGE:SLOP POS'); w(f':TRIG:EDGE:LEV {A.lev}'); w(':TRIG:SWE SING'); w(':SING'); time.sleep(1.2)
print('armed',q(':TRIG:STAT?'),'srat',q(':ACQ:SRAT?'),'err',q(':SYST:ERR?'), flush=True)
if A.cmd:
    import serial
    sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); sp.write((A.cmd+'\n').encode()); time.sleep(0.3); print('sent:',A.cmd, sp.read(4000).decode(errors='replace')[-600:]); sp.close()
elif not A.noreset:
    r=subprocess.run(['/tmp/hfi/venv/bin/python','tools/usb_reset.py'],cwd=ATOS,env={'PYTHONPATH':'managed_components/espp__ota/python','PATH':'/usr/bin:/bin'},capture_output=True,text=True); print(r.stdout.strip()[-60:], r.stderr.strip()[-120:], flush=True)
t0=time.time(); st=''
while time.time()-t0<A.wait:
    time.sleep(0.5); st=q(':TRIG:STAT?')
    if st=='STOP': print('triggered after %.1f s'%(time.time()-t0)); break
else: print('NO TRIGGER in %.0f s (state %s)'%(A.wait,st))
if st=='STOP':
    time.sleep(0.8); w(':STOP'); time.sleep(0.3)
    w(':WAV:SOUR CHAN1'); w(':WAV:MODE RAW'); w(':WAV:FORM WORD')
    pre=q(':WAV:PRE?').split(','); pts=int(pre[2]); xinc,xorig,xref=map(float,pre[4:7]); yinc,yorig,yref=map(float,pre[7:10])
    codes=[]; got=0
    while got<pts:
        n=min(250000,pts-got); w(f':WAV:STAR {got+1}'); w(f':WAV:STOP {got+n}'); w(':WAV:DATA?')
        d=block(); c=struct.unpack('<%dH'%(len(d)//2),d); codes.extend(c); got+=len(c)
    print('pts',got,'err',q(':SYST:ERR?'))
    a=(np.array(codes,float)-yorig-yref)*yinc*100; t=(xorig+(np.arange(len(a))-xref)*xinc)*1e6
    np.savez(f'/tmp/hfi/{A.tag}.npz',t=t,a=a)
    print('span %.1f us dt %.3f us peak %.1f A min %.1f A'%(t[-1]-t[0],xinc*1e6,a.max(),a.min()))
    idx=np.flatnonzero(np.abs(a)>2)
    if len(idx):
        gap=int(1.0/(xinc*1e6))
        for g in np.split(idx, np.flatnonzero(np.diff(idx)>gap)+1)[:25]:
            print('  |I|>2A: t=%+9.2f us  dur %7.2f us  peak %6.1f A'%(t[g[0]],(g[-1]-g[0]+1)*xinc*1e6,a[g][np.argmax(np.abs(a[g]))]))
w(':RUN'); s.close()
if not A.nobootlog:
    import serial
    for _ in range(20):
        if glob.glob('/dev/cu.usbmodem2101'): break
        time.sleep(0.5)
    time.sleep(1.0)
    sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2); buf=b''
    def cmd(c,wait=1.2):
        global buf; sp.write((c+'\n').encode()); t=time.time()
        while time.time()-t<wait: buf+=sp.read(4096)
    cmd('sq 1',0.4); cmd('bootlog',1.5); cmd('r'); cmd('s'); cmd('mi')
    sp.close(); print(buf.decode(errors='replace')[-2200:])
