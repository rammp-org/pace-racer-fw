import serial, time, re, sys, numpy as np
# xshd.py <out.npy> <every> <run_s> cmd@wait ... ; 'ARM' arms a mode-9 capture (3000 rows).
# Like otest.py (stops the drive before capdump), plus: guard result, hat-enc and |If| stats over the
# driven window, encoder speed ripple. Mode 9 rows: [enc, meas, hat], [ifa, ifb, w_hat].
out, every, run_s = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]); cmds = sys.argv[4:]
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wait=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wait: b += sp.read(8192)
    return b.decode(errors='replace')
cmd('sq 1', 0.2)
for c in cmds:
    w = 0.4
    if '@' in c: c, w = c.rsplit('@', 1); w = float(w)
    if c == 'ARM': c = f'cap {every} 3000 9'
    r = [l for l in cmd(c, w).splitlines() if l.startswith(('#', '!'))]
    print('>', c, '|', (r or [''])[-1][:150], flush=True)
cmd('stop', 0.6)
hrt = [l for l in cmd('hrt', 0.6).splitlines() if l.startswith('#hrt')]
print(hrt)
gerr = re.search(r'last_trip_err=(-?[\d.]+)', ' '.join(hrt)); gerr = float(gerr.group(1)) if gerr else None
em = re.search(r'errmax=(-?[\d.]+)', ' '.join(hrt)); em = float(em.group(1)) if em else None
num = r"(-?[\d.]+(?:e-?\d+)?)"; data = []
sp.write(b'capdump\n'); t = time.time(); b = b''
while time.time() - t < 35:
    b += sp.read(65536)
    if b'#capend' in b: break
for ln in b.decode(errors='replace').splitlines():
    m = re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
    if m: data.append([float(x) for x in m.groups()])
st = [l for l in cmd('s', 0.5).splitlines() if l.startswith(('#stats', '!'))]
print(st)
sp.close(); a = np.array(data); np.save(out, a)
a = a[:len(a) // 2 * 2].reshape(-1, 6)
dt = every / 20000
n_run = min(len(a), int(run_s / dt))
a = a[:n_run]
live = np.nonzero(np.diff(a[:, 2]))[0]
if len(live): a = a[:live[-1] + 2]
enc, meas, hat, fa, fb, wh = a.T
he = (hat - enc + 180) % 360 - 180
im = np.hypot(fa, fb)
encu = np.unwrap(np.radians(enc)); we = np.gradient(encu, dt)
span = np.degrees(encu[-1] - encu[0]) / 2
tr = np.where(abs(he) > 60)[0]
print(f'RESULT {out}: {len(a)*dt:.2f}s of {run_s:.1f}s, travel {span:+.0f} deg mech | hat-enc mean {he.mean():+.1f} sd {he.std():.1f} max {abs(he).max():.1f}'
      f' | |I| p50 {np.percentile(im,50):.2f} p95 {np.percentile(im,95):.2f} max {im.max():.2f} | w_enc mean {we.mean():+.1f} sd {we.std():.1f} min {we.min():+.1f} rad/s e'
      f' | first>60 {tr[0]*dt if len(tr) else -1:.2f}s | GUARD {"TRIP %.0f" % gerr if gerr else "ok"} errmax {em}')
