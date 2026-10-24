import serial, time, re, sys, numpy as np, json
# pol_scan.py <Vh> <ns_re> <ns_im> <off> '<hpol args>' <step_deg> <out.json> [runs] [align_A] [approach] ['<pre cmds;..>']
# approach 1: align from both sides ('th X-40', 'id A', 'th X', 'id 0' and from X+40) = more distinct detent conditions
# Polarity test vs the encoder: align the rotor with 'th X', 'id <align>', 'id 0' at
# 360/step positions, start hrt with no seed (branch = whatever atan2 gives), run
# 'hpol Vp ms psi' <runs> times (a correct decision flips the branch, so the next run
# tests the other end) and check every decision against the encoder. Also reports
# the psi that best separates the k angles (offline) and the margin distribution.
vh, nr, ni, off = (float(x) for x in sys.argv[1:5])
hargs, step, out = sys.argv[5], float(sys.argv[6]), sys.argv[7]
runs = int(sys.argv[8]) if len(sys.argv) > 8 else 2
ialign = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
approach = int(sys.argv[10]) if len(sys.argv) > 10 else 0
pre = [c for c in sys.argv[11].split(';') if c] if len(sys.argv) > 11 else []
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wait=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wait: b += sp.read(8192)
    return b.decode(errors='replace')
def enc():
    for _ in range(3):
        t = cmd('enc', 0.3); m = re.search(r"acc=(-?[\d.]+)deg.*elec=(-?[\d.]+)deg", t)
        if m: return float(m.group(1)), float(m.group(2))
    return float('nan'), float('nan')
num = r"([+-]?[\d.]+)"
keys = ['flip', 'margin', 'snr', 'Y', 'K', 'k', 'kdeg', 'sel', 'i1', 'imax', 'move', 'th0', 'hat', 'enc', 'err']
pat = (rf"#hpol flip={num} margin={num}% snr={num} Y={num}% K={num}% k={num}%@{num}deg sel={num} I1={num} A "
       rf"imax={num} A move={num} deg th0={num} hat={num} enc={num} err={num}")
cmd('sq 1', 0.3); cmd('dtc 0', 0.3); cmd('hrt 0', 0.3); print(cmd('hseed none', 0.3).strip())
for c in pre: print(cmd(c, 0.3).strip())
res = []
conds = [(X, d) for X in np.arange(0, 360, step) for d in ((-1, 1) if approach else (0,))]
for X, d in conds:
    cmd('hrt 0', 0.2)
    if d: cmd(f'th {X - 40 * d}', 0.2); cmd(f'id {ialign}', 0.4)
    cmd(f'th {X}', 0.2); cmd(f'id {ialign}', 0.5); cmd('id 0', 0.4)
    acc0, e0 = enc()
    cmd(f'hrt {vh} {nr} {ni} {off}', 0.5)
    for run in range(runs):
        t = cmd(f'hpol {hargs}', 1.6)
        m = re.search(pat, t)
        if not m: print('no result at', X, t.strip()[-200:]); continue
        r = dict(zip(keys, (float(v) for v in m.groups())))
        r.update(X=float(X), app=d, run=run, e0=e0, acc0=acc0)
        r['south'] = bool(abs((r['th0'] - r['enc'] + 180) % 360 - 180) > 90)  # theta_hat started on the south end
        r['ok'] = abs(r['err']) < 90
        res.append(r)
        print(f"X {X:5.1f}{'+-'[d < 0] if d else ' '} enc {r['enc']:6.1f} th0 {r['th0']:6.1f} {'S' if r['south'] else 'N'} flip={int(r['flip'])} "
              f"{'OK ' if r['ok'] else 'BAD'} margin {r['margin']:+7.3f}% snr {r['snr']:6.1f} Y {r['Y']:+6.3f} K {r['K']:+6.3f} k {r['k']:.3f}%@{r['kdeg']:+6.1f} "
              f"I1 {r['i1']:.2f} imax {r['imax']:.2f} move {r['move']:.2f} err {r['err']:+6.1f}", flush=True)
    acc1, _ = enc()
    if res: res[-1]['acc1'] = acc1
    cmd('hrt 0', 0.2)
cmd('hrt 0', 0.2); cmd('id 0', 0.2); cmd('stop', 0.5); cmd('hseed enc', 0.3)
json.dump(res, open(out, 'w'))
if res:
    south = np.array([r['south'] for r in res]); ok = np.array([r['ok'] for r in res])
    kv = np.array([r['k'] * np.exp(1j * np.radians(r['kdeg'])) for r in res]) * np.where(south, -1, 1)
    best = np.degrees(np.angle(np.mean(kv / abs(kv))))
    sm = np.array([r['margin'] for r in res]) * np.where(south, -1, 1)  # positive = correct side
    for nm in ('Y', 'K'):
        v = np.array([r[nm] for r in res]) * np.where(south, -1, 1)
        print(f"{nm}: correct side {np.sum(v > 0)}/{len(v)}, signed % min {v.min():+.3f} p5 {np.percentile(v,5):+.3f} median {np.median(v):+.3f}")
    mv = np.array([r['move'] for r in res]); snr = np.array([abs(r['snr']) for r in res])
    print(f"firmware decision correct {ok.sum()}/{len(ok)} ({100*ok.mean():.1f} %), south starts {south.sum()}")
    print(f"signed margin % (positive = correct): min {sm.min():+.3f} p5 {np.percentile(sm,5):+.3f} p25 {np.percentile(sm,25):+.3f} "
          f"median {np.median(sm):+.3f} max {sm.max():+.3f}; |snr| min {snr.min():.1f} median {np.median(snr):.1f}")
    print(f"k north-referred: mean angle {best:+.1f} deg, angle spread sd {np.degrees(np.std(np.angle(kv * np.exp(-1j*np.radians(best))))):.1f} deg, "
          f"|k| median {np.median(abs(kv)):.3f} %; with psi={best:.1f}: {(np.real(kv*np.exp(-1j*np.radians(best)))>0).sum()}/{len(kv)} correct")
    det = np.round(np.array([r['enc'] for r in res]) / 60.0) % 6  # cogging detents are ~60 deg elec apart
    for k in sorted(set(det)):
        sel_ = det == k
        print(f"  detent ~{np.mean([r['enc'] for r, q in zip(res, sel_) if q]):6.1f} deg: correct {ok[sel_].sum()}/{sel_.sum()}, "
              f"signed margin median {np.median(sm[sel_]):+.3f} min {sm[sel_].min():+.3f}")
    acc = np.array([r.get('acc1', np.nan) - r['acc0'] for r in res])
    print(f"rotor acc change per condition (mech deg, all runs): max |.| {np.nanmax(abs(acc)):.2f}")
    print(f"rotor move during tests: max {mv.max():.2f} median {np.median(mv):.2f} deg elec; I1 median {np.median([r['i1'] for r in res]):.2f} A, "
          f"imax max {max(r['imax'] for r in res):.2f} A")
sp.close()
