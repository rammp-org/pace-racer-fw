import serial, time, re, sys
# pol_xiq.py <Vh> <ns_re> <ns_im> <off> <iq_A> <hold_s> <X_deg>...
# Encoder-free start: for each rotor position X, align ('th X', 'id 3', 'id 0'),
# 'hseed pol', start hrt (the polarity test runs by itself at the first estimate),
# then 'xiq <iq>' with hguard 60 for hold_s. Reports the hpol decision, whether the
# guard tripped (hrt switched off / last_trip_err changed) and rotor movement.
vh, nr, ni, off, iq, hold = (float(x) for x in sys.argv[1:7]); xs = [float(x) for x in sys.argv[7:]]
sp = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.2); sp.read(65536)
def cmd(c, wait=0.4):
    sp.write((c + '\n').encode()); t = time.time(); b = b''
    while time.time() - t < wait: b += sp.read(8192)
    return b.decode(errors='replace')
def enc():
    t = cmd('enc', 0.3); m = re.search(r"acc=(-?[\d.]+)deg.*elec=(-?[\d.]+)deg", t)
    return float(m.group(1)), float(m.group(2))
def hrt():
    t = cmd('hrt', 0.3); m = re.search(r"#hrt vh=([\d.]+).*last_trip_err=(-?[\d.]+)", t)
    return float(m.group(1)), float(m.group(2))
line = lambda t, p: next((l for l in t.splitlines() if l.startswith(p)), t.strip()[-160:])
cmd('sq 1', 0.3); cmd('dtc 0', 0.3); cmd('hguard 60', 0.3); print(line(cmd('hseed pol', 0.3), '#'))
trips = 0
for X in xs:
    cmd('hrt 0', 0.2); cmd(f'th {X}', 0.2); cmd('id 3', 0.6); cmd('id 0', 0.4)
    acc0, e0 = enc(); _, lt0 = hrt()
    cmd(f'hrt {vh} {nr} {ni} {off}', 0.8)        # seeds, runs hpol (~0.3 s), marks seeded
    pol = line(cmd('hpol', 0.3), ('#hpol', '!'))
    r = line(cmd(f'xiq {iq}', hold), ('#', '!'))
    v1, lt1 = hrt(); acc1, e1 = enc()
    cmd('stop', 0.6)
    trip = v1 == 0.0 or lt1 != lt0
    trips += trip
    print(f"X {X:5.1f} enc {e0:6.1f} | {pol}\n        xiq: {r} | after {hold:.1f}s hrt vh={v1:.3f} "
          f"{'TRIP' if trip else 'no trip'} (last_trip_err {lt1:+.1f}) rotor moved {acc1-acc0:+.2f} deg mech", flush=True)
cmd('hrt 0', 0.2); cmd('stop', 0.5); cmd('hseed enc', 0.3)
print(f"guard trips {trips}/{len(xs)}")
print(line(cmd('s', 0.5), '#stats'))
sp.close()
