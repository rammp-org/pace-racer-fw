import numpy as np
# hrt_nfit.py: fit the drive-current saliency model on mode-8 captures (N, P, If, encoder); paths below are 2026-10-23 bench data
ns=-0.0176-0.0427j; off=np.radians(149.2)
def load(f):
    a=np.load(f); a=a[:len(a)//3*3].reshape(-1,3,3)
    d=dict(th=np.radians(a[:,0,0]), N=a[:,0,1]+1j*a[:,0,2], P=a[:,1,0]+1j*a[:,1,1], If=a[:,2,0]+1j*a[:,2,1])
    m=abs(d['P'])>1.5          # hrt running
    return {k:v[m] for k,v in d.items()}
F={'static':'/tmp/hfi/bt/M8_static.npy','brk1':'/tmp/hfi/bt/M8_brk1.npy','brk2':'/tmp/hfi/bt/M8_brk2.npy','erun':'/tmp/hfi/bt/M8_erun.npy',
   'e30':'/tmp/hfi/m8_e30.npy','em30':'/tmp/hfi/m8_em30.npy','if':'/tmp/hfi/m8_if.npy'}
D={k:load(v) for k,v in F.items()}
D['static']={k:v[:600] for k,v in D['static'].items()}
for k,d in D.items(): print(k,len(d['th']),'|If| med %.2f'%np.median(abs(d['If'])))
def feats(d,names):
    If=d['If']; m=abs(If); u=np.exp(2j*d['th'])*np.conj(d['P']/abs(d['P']))*np.exp(-1j*(np.pi-off))
    lib={'If2':If**2,'cIf2':np.conj(If)**2,'m2':m**2+0j,'If2n':If**2/np.maximum(m,0.3),'cIf2n':np.conj(If)**2/np.maximum(m,0.3),
         'If2m2':If**2*m**2,'u_m2':u*m**2,'u_m':u*m, 'If2P':If**2*np.conj(d['P']/abs(d['P'])), 'cIf2P':np.conj(If)**2*np.conj(d['P']/abs(d['P'])),
         'If2Pn':If**2*np.conj(d['P']/abs(d['P']))/np.maximum(m,0.3)}
    return u, np.array([lib[n] for n in names]).T if names else np.zeros((len(If),0))
def angerr(d,c,names):
    u,X=feats(d,names); Nc=d['N']-ns-(X@c[1:] if names else 0)
    w=Nc*d['P']*np.exp(1j*(np.pi-off)); e=np.degrees(np.angle(w*np.exp(-2j*d['th'])))/2
    return e
train=['static','brk1','brk2','erun','e30','em30']
for names in ([],['If2'],['cIf2'],['If2','cIf2'],['If2','cIf2','m2'],['If2P'],['If2P','cIf2P'],['If2Pn'],['u_m2'],['u_m2','If2P'],['u_m2','If2P','cIf2P'],['u_m','u_m2','If2P','cIf2P','m2']):
    U=[];Y=[]
    for k in train:
        u,X=feats(D[k],names); U.append(np.c_[u,X]); Y.append(D[k]['N']-ns)
    U=np.vstack(U);Y=np.concatenate(Y); c,*_=np.linalg.lstsq(U,Y,rcond=None)
    s=[]
    for k,d in D.items():
        e=angerr(d,c,names); s.append(f'{k} {e.mean():+5.1f}/{e.std():4.1f}/{np.percentile(abs(e),98):4.0f}')
    print(f'{"+".join(names) or "base":28s}', ' | '.join(s))

print('--- w-domain fit: Wn = (N-ns) Phat e^{j(pi-off)} = e^{j2th}(A + B m^2) + K If^2')
def wn(d): return (d['N']-ns)*(d['P']/abs(d['P']))*np.exp(1j*(np.pi-off))
U=[];Y=[]
for k in train:
    d=D[k]; m2=abs(d['If'])**2; e2=np.exp(2j*d['th'])
    U.append(np.c_[e2,e2*m2,d['If']**2]); Y.append(wn(d))
U=np.vstack(U);Y=np.concatenate(Y); c,*_=np.linalg.lstsq(U,Y,rcond=None)
A,B,K=c; b=B/A
print('A',np.round(A,4),'|A|',round(abs(A),4),'arg A deg',round(np.degrees(np.angle(A)),1),' b=B/A',np.round(b,4),' K',np.round(K,4))
def corr(d,kk=K,bb=b,a0=np.angle(A)):
    m2=abs(d['If'])**2; w=wn(d)-kk*d['If']**2
    two=np.angle(w)-np.angle(1+bb*m2)-a0
    return np.degrees(np.angle(np.exp(1j*(two-2*d['th']))))/2
for k,d in D.items():
    e=corr(d); e0=corr(d,0,0,0)
    print(f'{k:7s} raw {e0.mean():+6.1f}/{e0.std():5.1f}  corrected {e.mean():+6.1f}/{e.std():5.1f} p98 {np.percentile(abs(e),98):4.0f}')
# around the flip in brk1: true rotor-frame current and errors
for k in ('brk1','brk2'):
    d=D[k]; ir=d['If']*np.exp(-1j*d['th']); e=corr(d); e0=corr(d,0,0,0)
    big=np.where(abs(ir.real)>1.0)[0]
    print(k,'samples with |true id|>1 A:',len(big),' raw err mean %+.1f sd %.1f | corrected mean %+.1f sd %.1f'%(e0[big].mean(),e0[big].std(),e[big].mean(),e[big].std()) if len(big) else '')
