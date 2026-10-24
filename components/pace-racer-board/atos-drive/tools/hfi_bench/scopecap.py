import socket, struct, time, sys, numpy as np, serial
# scopecap.py <lev_A> <tdiv_s> <tag> cmd@wait... : peak-detect single shot on phase A (Micsig clamp, CH1) covering a whole
# console sequence (e.g. a 10 s xrun at tdiv 1 s): trigger on +lev (rising), trigger point 1 div from the left. Prints the
# record's peak/min and the largest 1 ms envelope rises (a duty freeze shows as a multi-period ramp). Saves /tmp/hfi/<tag>.npz.
lev, tdiv, tag = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]; cmds = sys.argv[4:]
s = socket.create_connection(("192.168.1.101", 5555), timeout=30, source_address=("192.168.1.50", 0)); f = s.makefile("rb")
def w(c): s.sendall((c + "\n").encode()); time.sleep(0.12)
def q(c): w(c); return f.readline().decode().strip()
def block():
    assert f.read(1) == b"#"; n = int(f.read(1)); ln = int(f.read(n)); d = f.read(ln); f.readline(); return d
w('*CLS'); w(':STOP'); w(':CHAN1:UNIT VOLT'); w(':CHAN1:PROB 1'); w(':CHAN1:SCAL 0.03'); w(':CHAN1:OFFS 0')
w(f':TIM:SCAL {tdiv}'); w(':TIM:MODE MAIN'); w(f':TIM:OFFS {4*tdiv}'); w(':ACQ:MDEP 100k'); w(':ACQ:TYPE PEAK')
w(':TRIG:MODE EDGE'); w(':TRIG:EDGE:SOUR CHAN1'); w(':TRIG:EDGE:SLOP POS'); w(f':TRIG:EDGE:LEV {lev/100}'); w(':TRIG:SWE SING'); w(':SING')
time.sleep(max(1.2, 1.2 * tdiv))
w('*CLS'); print('armed', q(':TRIG:STAT?'), 'srat', q(':ACQ:SRAT?'), flush=True)
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wt=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wt: b += sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1', 0.2)
for c in cmds:
    wt = 0.4
    if '@' in c: c, wt = c.rsplit('@', 1); wt = float(wt)
    r = [l for l in cmd(c, wt).splitlines() if l.startswith(('#', '!'))]
    print('>', c, '|', (r or [''])[-1][:140], flush=True)
cmd('stop', 0.6)
print('board:', [l for l in cmd('hrt', 0.5).splitlines() if l.startswith(('#hrt', '!'))])
print('board:', [l for l in cmd('s', 0.6).splitlines() if l.startswith(('#stats', '!'))])
t0 = time.time(); st = ''
while time.time() - t0 < 10 * tdiv:
    st = q(':TRIG:STAT?')
    if st == 'STOP': break
    time.sleep(0.5)
if st == 'STOP':
    w(':STOP'); time.sleep(0.3); w(':WAV:SOUR CHAN1'); w(':WAV:MODE RAW'); w(':WAV:FORM WORD')
    pre = q(':WAV:PRE?').split(','); pts = int(pre[2]); xinc, xorig, xref = map(float, pre[4:7]); yinc, yorig, yref = map(float, pre[7:10])
    codes = []; got = 0
    while got < pts:
        n = min(250000, pts - got); w(f':WAV:STAR {got+1}'); w(f':WAV:STOP {got+n}'); w(':WAV:DATA?')
        d = block(); c = struct.unpack('<%dH' % (len(d) // 2), d); codes.extend(c); got += len(c)
    a = (np.array(codes, float) - yorig - yref) * yinc * 100; t = xorig + (np.arange(len(a)) - xref) * xinc
    np.savez(f'/tmp/hfi/{tag}.npz', t=t, a=a)
    i = np.argmax(abs(a)); k = max(1, int(round(1e-3 / xinc)))
    env = abs(a[:len(a) // k * k]).reshape(-1, k).max(1)
    rise = np.diff(env)
    top = np.argsort(rise)[-3:][::-1]
    print(f'RESULT {tag}: t {t[0]:+.2f}..{t[-1]:+.2f} s, {len(a)} pts, dt {xinc*1e6:.2f} us | peak {a.max():+.1f} A min {a.min():+.1f} A '
          f'(|max| at t={t[i]:+.3f} s) | 1 ms envelope p50 {np.percentile(env,50):.1f} p99 {np.percentile(env,99):.1f} A | '
          f'largest 1 ms rises: ' + ', '.join(f'{rise[j]:+.1f} A @ {t[j*k]:+.2f} s' for j in top))
else:
    print('NO TRIGGER, state', st)
w(':ACQ:TYPE NORM'); w(':RUN'); s.close(); sp.close()
