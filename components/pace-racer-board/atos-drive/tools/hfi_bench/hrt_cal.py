import json, sys, numpy as np
r=json.load(open(sys.argv[1])); th=np.radians([x['e0'] for x in r]); N=np.array([complex(*x['N']) for x in r]); P=np.array([complex(*x['P']) for x in r])
c,*_=np.linalg.lstsq(np.c_[np.ones_like(th),np.exp(2j*th)],N,rcond=None)
est2=np.angle((N-c[0])*P/abs(P))+np.pi; off=np.angle(np.mean(np.exp(1j*(est2-2*th))))
print(f'{c[0].real:.4f} {c[0].imag:.4f} {np.degrees(off):.1f}')
