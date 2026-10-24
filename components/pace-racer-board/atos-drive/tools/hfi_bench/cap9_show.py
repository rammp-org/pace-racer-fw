import numpy as np, sys
f=sys.argv[1]; every=int(sys.argv[2]); step=int(sys.argv[3]) if len(sys.argv)>3 else 20
rng=[int(x) for x in sys.argv[4].split(':')] if len(sys.argv)>4 else None
a=np.load(f); a=a[:len(a)//2*2].reshape(-1,6)
enc,meas,hat,fa,fb,w=a.T; dt=every/20000
th=np.radians(enc); iq=fb*np.cos(th)-fa*np.sin(th); idd=fa*np.cos(th)+fb*np.sin(th)
h=np.radians(hat); iqh=fb*np.cos(h)-fa*np.sin(h)
me=(meas-enc+90)%180-90; he=(hat-enc+180)%360-180; ue=np.degrees(np.unwrap(th)); v=np.gradient(np.unwrap(th))/dt
idx=range(rng[0],rng[1],step) if rng else range(0,len(a),step)
for i in idx: print(i, f't {i*dt:.3f} pos {ue[i]-ue[0]:7.0f} v {v[i]:6.0f} w_hat {w[i]:6.0f} iq_true {iq[i]:5.2f} id_true {idd[i]:5.2f} iq_hat {iqh[i]:5.2f} meas-enc {me[i]:6.1f} hat-enc {he[i]:6.1f}')
