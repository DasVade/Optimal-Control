"""Render demonstrations from preserved traces only; never run controllers."""
from pathlib import Path
import os, json, hashlib
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from control_portfolio.cartpole import Controller, wrap
OUT=ROOT/'docs/figures/robotics'; OUT.mkdir(exist_ok=True)
verification={}
def load(path):
    p=ROOT/'outputs'/path/'trajectory.npz'
    verification[str(path)]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
    d=dict(np.load(p)); assert all(np.isfinite(v).all() for v in d.values())
    return d
def save(fig, update, frames, name, fps=10):
    FuncAnimation(fig,update,frames=frames,interval=1000/fps).save(OUT/name,writer=PillowWriter(fps=fps))
    plt.close(fig)

traces=[load('quadrotor_20261023/'+n) for n in ['lqr_ic1','mpc_n10_ic1']]
assert np.array_equal(traces[0]['time'],traces[1]['time'])
assert np.array_equal(traces[0]['state'][0],traces[1]['state'][0])
assert all(np.max(d['applied'])<=5.5+1e-9 and np.min(d['applied'])>=-1e-9 for d in traces)
fig,axes=plt.subplots(1,2,figsize=(8,3.5)); artists=[]
for ax,label in zip(axes,['Clipped LQR','Constrained MPC N=10']):
    ax.set(xlim=(-.6,1.4),ylim=(-.6,1.4),aspect='equal',xlabel='y (m)',ylabel='z (m)',title=label)
    ax.plot(0,0,'k*'); trail,=ax.plot([],[],lw=1); body,=ax.plot([],[],'o-',lw=3)
    artists.append((trail,body))
title=fig.suptitle(''); fig.tight_layout(rect=(0,0,1,.9))
def quad(i):
    for d,(trail,body) in zip(traces,artists):
        x=d['state'][i]; v=.25*np.array([np.cos(x[2]),np.sin(x[2])])
        trail.set_data(d['state'][:i+1,0],d['state'][:i+1,1]); body.set_data([x[0]-v[0],x[0]+v[0]],[x[1]-v[1],x[1]+v[1]])
    title.set_text(f'Same nonlinear plant, synchronized t={traces[0]["time"][i]:.1f} s')
save(fig,quad,range(0,501,5),'quadrotor_lqr_mpc.gif')

d=load('verified_20261023/cartpole/swing_30deg'); t=d['time']; x=d['state']; controller=Controller()
modes=np.array([controller.command(s)[2]=='lqr' for s in x]); switches=np.flatnonzero(modes[1:]!=modes[:-1])+1
within=(abs(x[:,0])<=.05)&(abs(wrap(x[:,1]-np.pi))<=.05)&(abs(x[:,2])<=.1)&(abs(x[:,3])<=.1)
valid=np.flatnonzero(np.logical_and.accumulate(within[::-1])[::-1] & (t<=t[-1]-1))
assert len(valid) and len(switches)==1 and modes[-1]
verification['cartpole']={'settling_s':float(t[valid[0]]),'switch_s':t[switches].tolist(),'final_state':x[-1].tolist(),'max_abs_cart_m':float(abs(x[:,0]).max()),'reference_bound_exceedance_fraction':float(np.mean(abs(x[:,0])>2))}
fig,(ax,err)=plt.subplots(2,1,figsize=(6,4.5),gridspec_kw={'height_ratios':[2,1]})
ax.set(xlim=(float(x[:,0].min())-1.2,float(x[:,0].max())+1.2),ylim=(-1.2,1.2),aspect='equal',xlabel='Cart position (m)')
ax.axhline(0,color='gray',lw=.5); pole,=ax.plot([],[],'o-',lw=3); title=ax.set_title('')
err.plot(t,wrap(x[:,1]-np.pi),label='Wrapped upright error'); cursor=err.axvline(0,color='black')
err.set(xlabel='Time (s)',ylabel='Angle error (rad)'); err.legend(fontsize=8); fig.tight_layout()
def cart(i):
    p,theta=x[i,:2]; pole.set_data([p,p+np.sin(theta)],[0,-np.cos(theta)])
    title.set_text(f'Cart-pole | {"LQR" if modes[i] else "Energy shaping"} | t={t[i]:.1f} s')
    cursor.set_xdata([t[i],t[i]])
save(fig,cart,range(0,len(t),10),'cartpole_hybrid.gif')

plans=[load('minsnap_20261023/duration'+s) for s in ['1.0','2.0']]
assert np.allclose(plans[0]['state'],plans[1]['state'],atol=1e-8)
w=np.array([[-3,-4],[0,0],[2,3],[4,0],[8,-4]])
fig,ax=plt.subplots(figsize=(6,3.6)); ax.plot(plans[0]['state'][:,0],plans[0]['state'][:,1],color='gray',lw=1)
ax.scatter(w[:,0],w[:,1],c='black',s=18,label='Waypoints'); dots=[]
for label,marker in [('4 s plan','o'),('8 s plan','x')]:
    dot,=ax.plot([],[],marker,ms=9,label=label); dots.append(dot)
ax.set(xlabel='y (m)',ylabel='z (m)',aspect='equal'); ax.legend(fontsize=8); title=ax.set_title('Planned motion only — no flight dynamics | t=0.0 s',fontsize=10); fig.tight_layout()
def snap(k):
    now=k/10
    for d,dot in zip(plans,dots):
        p=[np.interp(min(now,d['time'][-1]),d['time'],d['state'][:,j]) for j in range(2)]
        dot.set_data([p[0]],[p[1]])
    title.set_text(f'Planned motion only — no flight dynamics | t={now:.1f} s')
save(fig,snap,range(81),'minimum_snap_duration.gif')
(ROOT/'docs/results/robotics/demo_verification.json').write_text(json.dumps(verification,indent=2)+'\n')
print(json.dumps(verification['cartpole'],indent=2))
