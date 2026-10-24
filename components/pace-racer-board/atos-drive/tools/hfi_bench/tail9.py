import numpy as np, sys
# tail9.py file.npy [t0 t1 step]: mode-9 rows (8 ms) with true-frame id/iq, meas-enc, hat-enc, w_hat, w_enc
a=np.load(sys.argv[1]); a=a[:len(a)//2*2].reshape(-1,6); dt=float(sys.argv[5]) if len(sys.argv)>5 else 0.008
live=np.nonzero(np.diff(a[:,2]))[0]; a=a[:live[-1]+2]
enc,meas,hat,fa,fb,wh=a.T; me=(meas-enc+90)%180-90; he=(hat-enc+180)%360-180; ir=(fa+1j*fb)*np.exp(-1j*np.radians(enc))
we=np.gradient(np.unwrap(np.radians(enc)),dt); n=len(a)
t0=float(sys.argv[2]) if len(sys.argv)>2 else (n-40)*dt; t1=float(sys.argv[3]) if len(sys.argv)>3 else n*dt; st=int(sys.argv[4]) if len(sys.argv)>4 else 1
for i in range(int(t0/dt),min(n,int(t1/dt)),st):
    print(f't {i*dt:5.2f} enc {enc[i]:6.1f} me {me[i]:+6.1f} he {he[i]:+6.1f} id {ir[i].real:+5.2f} iq {ir[i].imag:+5.2f} wh {wh[i]:+6.1f} we {we[i]:+6.1f}')
