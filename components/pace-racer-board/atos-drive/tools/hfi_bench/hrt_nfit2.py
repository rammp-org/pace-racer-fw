import numpy as np, sys
# hrt_nfit2.py "<ns_re ns_im off>" train.npy... [-- test.npy...] : hrt_nfit.py's w-domain fit of the drive-current saliency
# (w/|P| = e^{j2th} A (1 + b|If|^2) + K If^2) on mode-8 captures taken at any Vh/cal. Prints the 'hrtn kr ki br bi' line and
# per-file angle error (deg elec, mean/sd/p98) raw vs corrected.
args = sys.argv[1:]; ns_re, ns_im, offd = map(float, args[0].split()); ns = ns_re + 1j * ns_im; off = np.radians(offd)
files = args[1:]; test = []
if '--' in files: i = files.index('--'); files, test = files[:i], files[i + 1:]
def load(f):
    a = np.load(f); a = a[:len(a) // 3 * 3].reshape(-1, 3, 3)
    d = dict(th=np.radians(a[:, 0, 0]), N=a[:, 0, 1] + 1j * a[:, 0, 2], P=a[:, 1, 0] + 1j * a[:, 1, 1], If=a[:, 2, 0] + 1j * a[:, 2, 1])
    m = abs(d['P']) > 0.5 * np.median(abs(d['P'])) if len(d['P']) else []
    return {k: v[m] for k, v in d.items()}
D = {f: load(f) for f in files + test}
def wn(d): return (d['N'] - ns) * (d['P'] / abs(d['P'])) * np.exp(1j * (np.pi - off))
U = []; Y = []
for f in files:
    d = D[f]; m2 = abs(d['If']) ** 2; e2 = np.exp(2j * d['th'])
    U.append(np.c_[e2, e2 * m2, d['If'] ** 2]); Y.append(wn(d))
U = np.vstack(U); Y = np.concatenate(Y); (A, B, K), *_ = np.linalg.lstsq(U, Y, rcond=None); b = B / A
print(f'|P| median {np.median(np.concatenate([abs(D[f]["P"]) for f in files])):.2f} A | A {A:.4f} |A| {abs(A):.4f} arg {np.degrees(np.angle(A)):+.1f} deg')
print(f'hrtn {K.real:.4f} {K.imag:.4f} {b.real:.4f} {b.imag:.4f}')
def err(d, kk, bb, a0):
    m2 = abs(d['If']) ** 2; w = wn(d) - kk * d['If'] ** 2
    return np.degrees(np.angle(np.exp(1j * (np.angle(w) - np.angle(1 + bb * m2) - a0 - 2 * d['th'])))) / 2
for f, d in D.items():
    e0 = err(d, 0, 0, 0); e = err(d, K, b, 0)
    print(f'{f.split("/")[-1]:24s} n {len(e):4d} |If| p50 {np.median(abs(d["If"])):.2f} p95 {np.percentile(abs(d["If"]),95):.2f} | raw {e0.mean():+6.1f}/{e0.std():5.1f}/{np.percentile(abs(e0),98):4.0f}'
          f'  corrected {e.mean():+6.1f}/{e.std():5.1f}/{np.percentile(abs(e),98):4.0f}{"  (test)" if f in test else ""}')
