# pol_analyze.py <scan.json> [move_max_deg_elec] [enc_ofs_fix_deg]: polarity accuracy vs the encoder,
# stator-fixed Y fit (hpolcal c1 s1 c3 s3 = the 'stator' numbers) and cross-validation.
import json,sys,numpy as np
r=json.load(open(sys.argv[1])); mvmax=float(sys.argv[2]) if len(sys.argv)>2 else 4.0
ofs_fix=float(sys.argv[3]) if len(sys.argv)>3 else 0.0
for x in r:
    x['enc']=(x['enc']+ofs_fix)%360
    x['south']=abs((x['th0']-x['enc']+180)%360-180)>90
v=[x for x in r if x['move']<mvmax]
print(f"{len(r)} decisions, {len(v)} with move < {mvmax} deg elec ({mvmax/2:.0f} deg mech)")
s=np.array([-1 if x['south'] else 1 for x in v]); Y=np.array([x['Y'] for x in v]); th=np.radians([x['th0'] for x in v])
det=np.round(np.array([x['enc'] for x in v])/60)%6; app=np.array([x['app'] for x in v])
print(f"uncalibrated Y (ysign +1): {np.sum(np.sign(Y)==s)}/{len(s)} = {100*np.mean(np.sign(Y)==s):.1f} %")
def A(th,h=(1,3)): return np.c_[tuple([f(k*th) for k in h for f in (np.cos,np.sin)])]
def fit(i):  # Y = m*s + stator(th0)
    M=np.c_[s[i],A(th[i])]; c,*_=np.linalg.lstsq(M,Y[i],rcond=None); return c
c=fit(np.ones(len(s),bool)); res=Y-np.c_[s,A(th)]@c
print(f"full fit: magnet {c[0]:+.2f} %, stator {np.round(c[1:],2)}, resid sd {res.std():.2f}")
for name,groups in (('leave-one-detent-out',det),('approach split',app)):
    ok=np.zeros(len(s),bool); mg=np.zeros(len(s))
    for g in set(groups):
        te=groups==g; cc=fit(~te); corr=Y[te]-A(th[te])@cc[1:]
        mg[te]=corr*s[te]; ok[te]=np.sign(corr)==s[te]
    print(f"{name}: {ok.sum()}/{len(ok)} = {100*ok.mean():.1f} %; signed margin p5 {np.percentile(mg,5):+.2f} median {np.median(mg):+.2f}")
    for g in sorted(set(groups)): print(f"    {g}: {ok[groups==g].sum()}/{(groups==g).sum()}")
