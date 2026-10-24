import numpy as np, sys
# hvh_replay.py file.npy... : open-loop replay of the hrt observer (smooth-hfi-drive set) on mode-9 captures (8 ms rows,
# meas held between rows) with the hfi-higher-vh additions: hrtz model gate (ge deg, tau ms) and hrtb zero-current windows.
# The windows are SYNTHETIC: inside a window the model sees iq = 0 and, after settle, the measurement is the encoder plus
# N(0, wsd) deg (zero-current scans at 0.20/0.25 V: 1-3 deg rms); outside, the captured measurement (cout x i0 confidence).
# Open loop: rotor motion and currents are the captured closed-loop ones (no feedback through the drive).
def run(f, kp=600, ki=100000, ktj=4000, bj=8, coul=4000, ca=-3600, cb=-3500, i0=1.5, emax=20, stic=4000,
        ge=0, tau=20, per=0, wid=3, settle=1.5, boost=3, cout=0.5, wsd=3.0, sub=32, every=160, seed=1):
    rng = np.random.default_rng(seed)
    a = np.load(f); a = a[:len(a)//2*2].reshape(-1, 6); live = np.nonzero(np.diff(a[:, 2]))[0]; a = a[:live[-1]+2]
    enc, meas, hat, fa, fb, w = a.T; dt0 = every/20000; dt = dt0/sub
    th = np.radians(hat[0]); wh = w[0]; em = np.radians(emax); elp = 0.0; out = np.zeros(len(a)); t = 0.0
    for i in range(len(a)):
        m2 = fa[i]**2 + fb[i]**2
        for _ in range(sub):
            ph = (t*1e3) % per if per > 0 else 1e9
            inwin = ph < wid; clean = inwin and ph >= settle
            c, s = np.cos(th), np.sin(th); iq = 0.0 if inwin else fb[i]*c - fa[i]*s
            conf = 1/(1 + (0 if inwin else m2)/i0**2)
            mz = np.radians(enc[i] + rng.normal(0, wsd)) if clean else np.radians(meas[i])
            er = np.clip(((2*mz - 2*th + np.pi) % (2*np.pi) - np.pi)/2, -em, em)
            e = er*boost if clean else er*conf*(cout if per > 0 else 1)
            acog = ca*np.cos(6*th) + cb*np.sin(6*th); ad = ktj*iq + acog
            if stic > 0: ad -= np.clip(ad, -stic, stic)*max(0, 1 - abs(wh)*0.2)
            elp += (er - elp)*min(1, dt/(tau*1e-3))
            if ge > 0 and ad*elp < 0: ad *= max(0, 1 - abs(elp)/np.radians(ge))
            wh += (ki*e + ad - bj*wh - coul*np.clip(wh*0.2, -1, 1))*dt
            th += (wh + kp*e)*dt; t += dt
        out[i] = np.degrees(th)
    return (out - enc + 180) % 360 - 180
if __name__ == '__main__':
    cfgs = [('captured', None), ('replay base', {}), ('hrtz 5 20', dict(ge=5)), ('hrtb 20 3 1.5 3 0.5', dict(per=20)),
            ('hrtb + hrtz', dict(per=20, ge=5)), ('hrtb 10 3 1.5 3 0.5', dict(per=10))]
    for f in sys.argv[1:]:
        a = np.load(f); a = a[:len(a)//2*2].reshape(-1, 6); live = np.nonzero(np.diff(a[:, 2]))[0]; a = a[:live[-1]+2]
        cap = (a[:, 2] - a[:, 0] + 180) % 360 - 180
        for name, kw in cfgs:
            he = cap if kw is None else run(f, **kw); tr = np.where(abs(he) > 60)[0]
            print(f'{f.split("/")[-1]:22s} {name:22s} mean {he.mean():+6.1f} sd {he.std():5.1f} max {abs(he).max():5.1f} '
                  f'first>60 {tr[0]*0.008 if len(tr) else -1:5.2f}s  last-0.3s mean {he[-38:].mean():+6.1f}')
