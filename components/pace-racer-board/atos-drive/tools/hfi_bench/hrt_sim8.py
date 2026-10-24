import numpy as np, sys
# hrt_sim8.py: offline replay of the hrt observer on mode-8 captures, with/without the hrtn correction
sys.path.insert(0,'/tmp/hfi/bt')
ns=-0.0176-0.0427j; off=np.radians(149.2); K=-0.0046+0.0412j; b=-0.0484+0.0493j
def load(f):
    a=np.load(f); a=a[:len(a)//3*3].reshape(-1,3,3)
    d=dict(th=np.radians(a[:,0,0]), N=a[:,0,1]+1j*a[:,0,2], P=a[:,1,0]+1j*a[:,1,1], If=a[:,2,0]+1j*a[:,2,1])
    m=abs(d['P'])>1.5
    return {k:v[m] for k,v in d.items()}
def meas2(d,corr=True):
    w=(d['N']-ns)*(d['P']/abs(d['P']))*np.exp(1j*(np.pi-off))
    if not corr: return np.angle(w)
    m2=abs(d['If'])**2
    return np.angle(w-K*d['If']**2)-np.angle(1+b*m2)
def run(d,every=20,kp=300,ki=30000,ktj=4000,bj=8,coul=1300,ca=-3000,cb=-3100,emax=20,corr=True,sub=20):
    two=meas2(d,corr); dt=every/20000/sub; th=d['th'][0]; wh=0; em=np.radians(emax); out=[]
    for i in range(len(two)):
        c,s=np.cos(th),np.sin(th); iq=d['If'][i].imag*c-d['If'][i].real*s
        acog=ca*np.cos(6*th)+cb*np.sin(6*th)
        for _ in range(sub):
            e=np.clip(((two[i]-2*th+np.pi)%(2*np.pi)-np.pi)/2,-em,em)
            wh+=(ki*e+ktj*iq-bj*wh-coul*np.clip(wh*0.2,-1,1)+acog)*dt; th+=(wh+kp*e)*dt
        out.append(th)
    he=np.degrees(np.angle(np.exp(1j*(np.array(out)-d['th']))))
    return he
for f,ev in [('/tmp/hfi/bt/M8_brk1.npy',20),('/tmp/hfi/bt/M8_brk2.npy',20),('/tmp/hfi/bt/M8_erun.npy',20),('/tmp/hfi/m8_e30.npy',20),('/tmp/hfi/m8_if.npy',20)]:
    d=load(f)
    for corr in (False,True):
        for g in [dict(),dict(kp=150,ki=10000),dict(kp=600,ki=100000)]:
            he=run(d,every=ev,corr=corr,**g); t=np.where(abs(he)>60)[0]
            print(f'{f.split("/")[-1]:14s} corr {int(corr)} {str(g):28s} mean {he.mean():+6.1f} sd {he.std():5.1f} max {abs(he).max():5.1f} first>60 {t[0] if len(t) else -1}')
