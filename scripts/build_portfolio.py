"""Export curated portfolio plots and numeric summaries from saved experiments only."""
import argparse
from pathlib import Path
import json
import os
import shutil

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'outputs/.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',type=Path,required=True)
args=parser.parse_args();run=args.run
data=json.loads((run/'results.json').read_text());env=json.loads((run/'environment.json').read_text())
figures=ROOT/'docs/figures';figures.mkdir(parents=True,exist_ok=True)
results=ROOT/'docs/results';results.mkdir(parents=True,exist_ok=True)
for name in ['results.json','config.json','environment.json','cartpole_metrics.csv']:shutil.copy2(run/name,results/name)
rows={r['name']:r for r in data['cartpole']}
def load(name):
    with np.load(run/'cartpole'/name/'trajectory.npz') as saved:return {k:saved[k] for k in saved.files}
def angle(x):return (x+np.pi)%(2*np.pi)-np.pi

fig,axes=plt.subplots(2,2,figsize=(10,6),sharex=True)
for r in [.1,1.,10.]:
    s=load(f'r{r}_ic0');t=s['time'];x=s['state'];label=f'R={r:g}'
    axes[0,0].plot(t,x[:,0],label=label);axes[0,1].plot(t,angle(x[:,1]-np.pi),label=label)
    axes[1,0].plot(t,x[:,2],label=label);axes[1,1].plot(t,s['input'],label=label)
for ax,title,ylabel in zip(axes.flat,['Cart position','Upright angle error','Cart velocity','Applied force'],['m','rad','m/s','N']):
    ax.set(title=title,ylabel=ylabel);ax.grid(alpha=.2);ax.legend(fontsize=8)
for ax in axes[1]:ax.set_xlabel('Simulated time (s)')
fig.suptitle('Nonlinear cart-pole | continuous LQR | matched initial state [0.1, pi+0.1, 0, 0]')
fig.tight_layout();fig.savefig(figures/'lqr_weights.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(10,6))
for cap in [2.,10.,180.]:
    s=load(f'limit{cap}_ic1');axes[0,0].plot(s['time'],s['state'][:,0],label=f'{cap:g} N clip')
    axes[0,1].plot(s['time'],s['input'],label=f'{cap:g} N clip')
for name in ['basin_1.2_lqr','basin_1.2_hybrid']:
    s=load(name);axes[1,0].plot(s['time'],angle(s['state'][:,1]-np.pi),label=name.split('_')[-1])
for name in ['swing_30deg','swing_exact_down']:
    s=load(name);axes[1,1].plot(s['time'],angle(s['state'][:,1]-np.pi),label=name.replace('swing_',''))
titles=['Larger initial offset: position','Larger initial offset: applied force','Initial upright offset 1.2 rad','Energy shaping from near/exact rest below']
for ax,title,ylabel in zip(axes.flat,titles,['m','N','rad','rad']):
    ax.set(title=title,xlabel='Simulated time (s)',ylabel=ylabel);ax.grid(alpha=.2);ax.legend(fontsize=8)
axes[0,0].axhline(2,ls=':',c='gray');axes[0,0].axhline(-2,ls=':',c='gray')
fig.suptitle('Failures are retained | clipping is not constrained optimization; cart bound is diagnostic only')
fig.tight_layout();fig.savefig(figures/'failures.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(10,4))
for condition in ['nominal','heading_offset','speed_mismatch']:
    folder=run/'unicycle'/f'{condition}_seed101';s=np.load(folder/'trajectory.npz');row=json.loads((folder/'result.json').read_text())
    axes[0].plot(s['state'][:,0],s['state'][:,1],label=condition.replace('_',' '))
    axes[1].plot(s['time'],np.linalg.norm(s['state'][:,:2]-s['desired'],axis=1),label=condition.replace('_',' '))
o=row['obstacle'];axes[0].add_patch(plt.Circle((o['y'],o['z']),o['radius'],color='gray',alpha=.4))
axes[0].set(xlabel='y (m)',ylabel='z (m)',title='Unicycle feedforward | seed 101');axes[0].axis('equal')
axes[1].set(xlabel='Time (s)',ylabel='Position error (m)',title='No feedback correction')
for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(figures/'unicycle.png',dpi=150);plt.close(fig)

s=load('swing_30deg');x=s['state'];t=s['time'];fig,ax=plt.subplots(figsize=(6,3))
lo=float(x[:,0].min()-1.5);hi=float(x[:,0].max()+1.5)
ax.set(xlim=(lo,hi),ylim=(-1.4,1.4),xlabel='Cart position (m)',ylabel='Height (m)')
ax.set_aspect('equal');ax.axhline(0,c='gray');ax.axvline(-2,c='gray',ls=':');ax.axvline(2,c='gray',ls=':')
cart,=ax.plot([],[],'s',ms=12,c='#245682');pole,=ax.plot([],[],'o-',lw=3,c='#b55924')
label=ax.text(.02,.9,'',transform=ax.transAxes,fontsize=9)
indices=np.linspace(0,len(t)-1,40,dtype=int)
def frame(i):
    j=indices[i];p=x[j,0];theta=x[j,1]
    cart.set_data([p],[0]);pole.set_data([p,p+np.sin(theta)],[0,-np.cos(theta)])
    label.set_text(f'Simulation t={t[j]:.1f} s | energy shaping + LQR')
    return cart,pole,label
fig.tight_layout();FuncAnimation(fig,frame,frames=len(indices),blit=False).save(figures/'swingup.gif',writer=PillowWriter(fps=10),dpi=85)
plt.close(fig)

summary={'cartpole_cases':len(data['cartpole']),'unicycle_cases':len(data['unicycle']),
         'r_weight_ic0':{str(r):rows[f'r{r}_ic0']['metrics'] for r in [.1,1.,10.]},
         'stopped_cases':[r['name'] for r in data['cartpole'] if r['metrics']['stopped']],
         'unsettled_cases':[r['name'] for r in data['cartpole'] if r['metrics']['settling_time_s'] is None],
         'unicycle':{condition:{key:float(np.mean([r[key] for r in data['unicycle'] if r['condition']==condition]))
                      for key in ['position_rmse_m','final_position_error_m','sampled_min_clearance_m']}
                      for condition in ['nominal','heading_offset','speed_mismatch']}}
write=lambda path,value:path.write_text(json.dumps(value,indent=2),encoding='utf-8')
write(results/'summary.json',summary)
print(json.dumps(summary,indent=2))
