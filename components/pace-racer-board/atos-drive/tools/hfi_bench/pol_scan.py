import serial, time, re, sys, numpy as np, json
# pol_scan.py <Vh> <ns_re> <ns_im> <off> <A> <ms> <reps> <step_deg> <out.json> [align_A]
# Polarity test vs the encoder: align the rotor with 'th X', 'id <align>', 'id 0' at
# 360/step positions, start hrt with no seed (branch = whatever atan2 gives), run
# 'hpol A ms reps', and check the decided theta_hat against the encoder. The three
# candidate metrics (|P|, N rotor, both) are scored offline from dP and dN.
vh, nr, ni, off = (float(x) for x in sys.argv[1:5])
amps, ms, reps, step, out = float(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), sys.argv[9]
ialign = float(sys.argv[10]) if len(sys.argv) > 10 else 3.0
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wait=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wait: b += sp.read(8192)
    return b.decode(errors='replace')
def enc():
    t = cmd('enc', 0.3); m = re.search(r"acc=(-?[\d.]+)deg.*elec=(-?[\d.]+)deg", t)
    return float(m.group(1)), float(m.group(2))
num = r"([+-]?[\d.]+)"
keys = ['flip', 'margin', 'snr', 'metric', 'dP', 'dN', 'P', 'idp', 'idn', 'move', 'th0', 'hat', 'enc', 'err']
pat = (rf"#hpol flip={num} margin={num}% snr={num} metric={num} dP={num} dN={num} \|P\|={num} "
       rf"id={num}/{num} A move={num} deg th0={num} hat={num} enc={num} err={num}")
cmd('sq 1', 0.3); cmd('dtc 0', 0.3); cmd('hrt 0', 0.3); print(cmd('hseed none', 0.3).strip())
res = []
for X in np.arange(0, 360, step):
    cmd('hrt 0', 0.2); cmd(f'th {X}', 0.2); cmd(f'id {ialign}', 0.6); cmd('id 0', 0.4)
    acc0, e0 = enc()
    cmd(f'hrt {vh} {nr} {ni} {off}', 0.5)
    t = cmd(f'hpol {amps} {ms} {reps}', 4 * reps * (ms + 10) / 1000 + 0.8)
    acc1, e1 = enc()
    cmd('hrt 0', 0.2)
    m = re.search(pat, t)
    if not m: print('no result at', X, t.strip()[-200:]); continue
    r = dict(zip(keys, (float(v) for v in m.groups())))
    r.update(X=float(X), e0=e0, e1=e1, acc0=acc0, acc1=acc1)
    need = abs((r['th0'] - e0 + 180) % 360 - 180) > 90  # theta_hat started on the south end
    r['need'] = bool(need)
    r['ok'] = abs(r['err']) < 90
    res.append(r)
    print(f"X {X:5.1f} enc {e0:6.1f} th0 {r['th0']:6.1f} need={int(need)} flip={int(r['flip'])} "
          f"{'OK ' if r['ok'] else 'BAD'} margin {r['margin']:+6.2f}% snr {r['snr']:6.1f} dP {r['dP']:+.5f} "
          f"dN {r['dN']:+.5f} id {r['idp']:+.2f}/{r['idn']:+.2f} move {r['move']:.2f} dacc {acc1-acc0:+.2f} err {r['err']:+6.1f}", flush=True)
cmd('hrt 0', 0.2); cmd('id 0', 0.2); cmd('stop', 0.5); cmd('hseed enc', 0.3)
json.dump(res, open(out, 'w'))
if res:
    need = np.array([r['need'] for r in res])
    sgn = 1 - 2 * need  # +1 when theta_hat started on north: the correct metric sign
    P = np.array([r['P'] for r in res])
    for name, v in (('|P|', np.array([r['dP'] for r in res])), ('Nrot', np.array([r['dN'] for r in res])),
                    ('both', np.array([r['dP'] + r['dN'] for r in res]))):
        mg = 100 * v * sgn / P  # signed margin: positive = correct
        print(f"{name:5s} correct {np.sum(mg > 0)}/{len(mg)}  margin % min {mg.min():+.2f} p10 {np.percentile(mg,10):+.2f} "
              f"median {np.median(mg):+.2f} max {mg.max():+.2f}")
    ok = np.array([r['ok'] for r in res]); mv = np.array([r['move'] for r in res])
    snr = np.array([abs(r['snr']) for r in res])
    print(f"firmware decision correct {ok.sum()}/{len(ok)} ({100*ok.mean():.0f} %), flips needed {need.sum()}, "
          f"|snr| min {snr.min():.1f} median {np.median(snr):.1f}, move max {mv.max():.2f} median {np.median(mv):.2f} deg elec")
sp.close()
