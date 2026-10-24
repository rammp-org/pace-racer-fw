import numpy as np, sys
# offline replay of the hrt observer on a mode-9 capture (meas held between 1 kHz rows, 20 substeps)
def run(f, kp=300, ki=30000, ktj=0, bj=0, coul=0, ca=0, cb=0, l3=0, emax=20, comp=0.0, sub=20, every=20):
    a=np.load(f); a=a[:len(a)//2*2].reshape(-1,6)
    enc,meas,hat,fa,fb,w=a.T
    dt=every/20000/sub; th=np.radians(hat[0]); wh=0; dh=0; em=np.radians(emax)
    out=np.zeros(len(a))
    for i in range(len(a)):
        c,s=np.cos(th),np.sin(th); iq=fb[i]*c-fa[i]*s
        two=2*np.radians(meas[i]) - 2*np.radians(comp*abs(iq))
        acog=ca*np.cos(6*th)+cb*np.sin(6*th)
        for _ in range(sub):
            e2=(two-2*th+np.pi)%(2*np.pi)-np.pi; e=np.clip(e2/2,-em,em)
            dh+=l3*e*dt
            wh+=(ki*e+ktj*iq-bj*wh-coul*np.clip(wh*0.2,-1,1)+acog+dh)*dt
            th+=(wh+kp*e)*dt
        out[i]=np.degrees(th)
    he=(out-enc+180)%360-180
    return he
if __name__=='__main__':
    f=sys.argv[1]
    for args in [dict(),dict(kp=600,ki=100000),dict(ktj=4000,bj=8,coul=1300,ca=-3000,cb=-3100),
                 dict(kp=600,ki=100000,ktj=4000,bj=8,coul=1300,ca=-3000,cb=-3100),
                 dict(kp=600,ki=100000,ktj=4000,bj=8,coul=1300,ca=-3000,cb=-3100,l3=1e6),
                 dict(kp=1000,ki=250000,ktj=4000,bj=8,coul=1300,ca=-3000,cb=-3100)]:
        he=run(f,**args); t=np.where(abs(he)>60)[0]
        print(f'{str(args):95s} mean {he.mean():+6.1f} sd {he.std():5.1f} max {abs(he).max():5.1f} first>60 {t[0] if len(t) else -1}')
