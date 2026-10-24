import numpy as np
def load(f):
    a=np.load(f'/tmp/hfi/{f}.npy'); a=a[:len(a)//3*3].reshape(-1,3,3)
    th=np.radians(a[:,0,0]); N=a[:,0,1]+1j*a[:,0,2]; P=a[:,1,0]+1j*a[:,1,1]; iq=a[:,1,2]; If=a[:,2,0]+1j*a[:,2,1]; idd=a[:,2,2]
    return dict(th=th,N=N,P=P,iq=iq,If=If,id=idd)
phis=np.array([0,2*np.pi/3,4*np.pi/3])
def relay_R2(d,soft=0.0):
    P,N,If=d['P'],d['N'],d['If']
    A=np.abs(P[:,None]*np.exp(-1j*phis)+np.conj(N)[:,None]*np.exp(1j*phis))       # per-phase injection amplitude
    I=np.real(If[:,None]*np.exp(-1j*phis))                                           # per-phase fundamental
    c=I/np.maximum(A,1e-3); R=np.sqrt(np.clip(1-c**2,0,None))
    return (R*np.exp(2j*phis)).sum(1)/3, R.mean(1)
def feats(d,model):
    th=d['th']; P=d['P']; cols=[np.ones_like(th),np.exp(2j*th)]
    if 'harm' in model: cols+= [np.exp(-2j*th),np.exp(4j*th),np.exp(-4j*th),np.exp(6j*th),np.exp(-6j*th)]
    if 'relay' in model:
        R2,R0=relay_R2(d); u=P/abs(P)
        cols+= [np.conj(u)**2*R2, np.conj(u)**2*np.conj(R2), R2, np.conj(R2)]
    if 'ifund' in model:
        If=d['If']; cols+= [If**2, np.conj(If)**2, abs(If)**2*np.ones_like(th)]
    return np.c_[cols].T if False else np.array(cols).T
def ang_err(d,c,model):
    # estimate theta from N with all non-rotor terms removed, compare to encoder (mod 180)
    M=feats(d,model); Nr_only=d['N']-(M@c - M[:,1]*c[1])
    est2=np.angle(Nr_only/c[1]); err=np.degrees(np.angle(np.exp(1j*(est2-2*d['th']))))/2
    return err
D={k:load(k) for k in ('m8_e30','m8_em30','m8_if')}
for model in ('base','harm','relay','harm+relay','harm+relay+ifund'):
    tr=D['m8_e30']; M=feats(tr,model); c,*_=np.linalg.lstsq(M,tr['N'],rcond=None)
    out=[]
    for k,d in D.items():
        Md=feats(d,model); r=d['N']-Md@c; e=ang_err(d,c,model)
        out.append(f'{k}: resid {np.sqrt(np.mean(abs(r)**2)):.3f} θerr sd {e.std():5.1f} mean {e.mean():+5.1f}')
    print(f'{model:18s} |Nr| {abs(c[1]):.3f} | '+' | '.join(out))
