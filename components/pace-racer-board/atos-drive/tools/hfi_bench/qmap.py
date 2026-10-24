import serial, time, re, sys, numpy as np
# qmap.py <out.npy> <Vh> "<ns_re ns_im off>" <th_deg> [iq_A] : passive HFI angle error vs rotor-frame current quadrant at one
# detent (hfi-higher-vh, 2026-10-23). Aligns the rotor near th_deg (HOLD id 3 A, release), starts hrt observing only
# (hrtg drive 0, smooth-hfi-drive observer set), then holds encoder-frame (id, iq) steps with eiq/hid (|iq| below the
# ~1.1 A running friction, so no runaway) while a mode-9 capture runs; stops before capdump. Per step: true-frame id/iq,
# meas-enc circular mean and sd (2-theta domain), rotor travel. env HN overrides the hrtn line.
import os
out, vh, cal, th0 = sys.argv[1], float(sys.argv[2]), sys.argv[3], float(sys.argv[4]); iq = float(sys.argv[5]) if len(sys.argv) > 5 else 0.9
HN = os.environ.get('HN', 'hrtn -0.0046 0.0412 -0.0484 0.0493')
STEPS = [(0, 0)] + [(d, iq) for d in (0, -0.5, -1.0, 0.5)] + [(0, 0)] + [(d, -iq) for d in (0, -0.5, -1.0, 0.5)] + [(0, 0)]
HOLD_S, EV = 0.7, 100
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wait=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wait: b += sp.read(8192)
    return b.decode(errors='replace')
def last(r): r = [l for l in r.splitlines() if l.startswith(('#', '!'))]; return (r or [''])[-1][:140]
cmd('sq 1', 0.2)
for c, w in [('stop', 0.6), (f'th {th0}', 0.2), ('id 3', 0.7), ('id 0', 0.5), ('hguard 60', 0.2), ('hrtg 600 100000 0.01 0 0.2 20', 0.2),
             ('hrtm 4000 8 0 4000 1.5 4000', 0.2), ('hrtk -3600 -3500', 0.2), ('hrtc 0 0 0 6', 0.2), (HN, 0.2), ('cff 0 0', 0.2),
             ('xsl 0', 0.2), (f'hrt {vh} {cal}', 0.9)]:
    print('>', c, '|', last(cmd(c, w)), flush=True)
print('>', 'enc', '|', [l for l in cmd('enc', 0.3).splitlines() if 'elec' in l][:1])
cmd(f'cap {EV} 3000 9', 0.05)
t0 = time.time(); marks = []
for k, (d, q) in enumerate(STEPS):
    marks.append(time.time() - t0)
    r = cmd(f'eiq {q}', 0.05) if k == 0 or q != STEPS[k - 1][1] else ''
    cmd(f'hid {d}', HOLD_S)
marks.append(time.time() - t0)
cmd('stop', 0.6)
print([l for l in cmd('hrt', 0.6).splitlines() if l.startswith('#hrt')])
num = r"(-?[\d.]+(?:e-?\d+)?)"; data = []
sp.write(b'capdump\n'); t = time.time(); b = b''
while time.time() - t < 35:
    b += sp.read(65536)
    if b'#capend' in b: break
for ln in b.decode(errors='replace').splitlines():
    m = re.match(rf"^@\d+,{num},{num},{num}\s*$", ln)
    if m: data.append([float(x) for x in m.groups()])
print([l for l in cmd('s', 0.5).splitlines() if l.startswith(('#stats', '!'))])
sp.close(); a = np.array(data); np.save(out, a); np.save(out.replace('.npy', '_marks.npy'), np.array(marks))
a = a[:len(a) // 2 * 2].reshape(-1, 6); dt = EV / 20000; tt = np.arange(len(a)) * dt
enc, meas, hat, fa, fb, wh = a.T
ir = (fa + 1j * fb) * np.exp(-1j * np.radians(enc))
for k, (d, q) in enumerate(STEPS):
    s = (tt > marks[k] + 0.25) & (tt < marks[k + 1] - 0.02)
    if s.sum() < 5: continue
    z = np.exp(2j * np.radians(meas[s] - enc[s])); mu = np.degrees(np.angle(z.mean())) / 2
    sd = np.degrees(np.sqrt(-2 * np.log(max(abs(z.mean()), 1e-9)))) / 2
    print(f'step id* {d:+.1f} iq* {q:+.1f}: true id {ir[s].real.mean():+.2f} iq {ir[s].imag.mean():+.2f} | enc {enc[s].mean():6.1f} '
          f'(span {np.ptp(enc[s]):4.1f}) | meas-enc mean {mu:+6.1f} csd {sd:5.1f} | hat-enc {((hat[s]-enc[s]+180)%360-180).mean():+6.1f}')
