"""Build curated portfolio figures and small metrics from saved, real experiments.

Does not optimize, simulate, or alter existing experiment outputs.
"""
import os
import argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter

parser=argparse.ArgumentParser()
parser.add_argument('--main-only',action='store_true',help='Render only quadrotor/minimum-snap figures and quadrotor animation; do not curate metrics.')
parser.add_argument('--quadrotor-run',type=Path)
parser.add_argument('--minsnap-run',type=Path)
parser.add_argument('--figure-output',type=Path,default=ROOT/'docs/figures/robotics')
args=parser.parse_args()
OUT=ROOT/'outputs';FIG=args.figure_output;DATA=ROOT/'docs/results/robotics'
FIG.mkdir(parents=True,exist_ok=True);DATA.mkdir(parents=True,exist_ok=True)
sources={name:OUT/f'{name}_20261023' for name in ['quadrotor','minsnap','ilqr','acrobot','acrobot_folded','supporting','osc','robotics_benchmark','ilqr_benchmark','acrobot_benchmark']}
if args.quadrotor_run:sources['quadrotor']=args.quadrotor_run
if args.minsnap_run:sources['minsnap']=args.minsnap_run
if not args.main_only:
    for name,source in sources.items():
        for file in ['summary.json','config.json','environment.json']:
            if (source/file).exists():shutil.copyfile(source/file,DATA/f'{name}_{file}')
verification=OUT/'verification_20261023/verification.json'
if verification.exists() and not args.main_only:shutil.copyfile(verification,DATA/'verification.json')

def trace(module,name):return np.load(sources[module]/name/'trajectory.npz')
def finish(fig,name):fig.tight_layout();fig.savefig(FIG/name,dpi=165);plt.close(fig)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})


def export_quadrotor_animation():
    # Saved nonlinear MPC rollout, with the model's pitch convention.
    d=trace('quadrotor','mpc_n10_ic1');fig,ax=plt.subplots(figsize=(5,4));x=d['state'];t=d['time']
    ax.set(xlim=(-.6,1.4),ylim=(-.6,1.4),xlabel='y (m)',ylabel='z (m)',aspect='equal')
    ax.plot(0,0,'k*',label='Hover target');trail,=ax.plot([],[],lw=1);body,=ax.plot([],[],'o-',lw=3);ax.legend(loc='lower right')
    title=ax.set_title('')
    def frame(i):
        state=x[i];offset=.25*np.array([np.cos(state[2]),np.sin(state[2])])
        body.set_data([state[0]-offset[0],state[0]+offset[0]],[state[1]-offset[1],state[1]+offset[1]])
        trail.set_data(x[:i+1,0],x[:i+1,1]);title.set_text(f'Constrained MPC | t={t[i]:.1f} s')
        return body,trail,title
    animation=FuncAnimation(fig,frame,frames=range(0,len(t),5),interval=100)
    animation.save(FIG/'quadrotor.gif',writer=PillowWriter(fps=10));plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(10,6))
for name,label in [('lqr_ic1','Clipped LQR'),('mpc_n5_ic1','MPC: 4 steps'),('mpc_n10_ic1','MPC: 9 steps'),('mpc_n20_ic1','MPC: 19 steps')]:
    d=trace('quadrotor',name);t=d['time'];x=d['state'];u=d['applied']
    axes[0,0].plot(x[:,0],x[:,1],label=label)
    axes[0,1].plot(t,np.linalg.norm(x[:,:2],axis=1),label=label)
    axes[1,0].plot(t,x[:,2],label=label)
    axes[1,1].plot(t[:-1],np.sum((u-4.905)**2,axis=1),label=label)
axes[0,0].plot(0,0,'k*');axes[0,0].set(xlabel='y (m)',ylabel='z (m)',title='Matched nonlinear plant trajectories');axes[0,0].legend(fontsize=8)
axes[0,1].set(xlabel='Time (s)',ylabel='Position error norm (m)')
axes[1,0].set(xlabel='Time (s)',ylabel='Pitch (rad)')
axes[1,1].set(xlabel='Time (s)',ylabel='Squared input deviation\nrelative to hover (N^2)')
fig.suptitle('Hover recovery: x0=[0.5,0.5,0,1,1,0], motors 0-5.5 N',fontsize=12)
finish(fig,'quadrotor_comparison.png')

fig,axes=plt.subplots(1,3,figsize=(11,3.5))
for scale in [.5,1.,2.]:
    d=trace('minsnap',f'duration{scale}');x=d['state'];t=d['time']
    axes[0].plot(x[:,0],x[:,1],label=f'{4*scale:g} s')
    axes[1].plot(t,np.linalg.norm(d['derivative1'],axis=1),label=f'{4*scale:g} s')
    axes[2].plot(t,np.linalg.norm(d['derivative4'],axis=1),label=f'{4*scale:g} s')
w=np.array([[-3,-4],[0,0],[2,3],[4,0],[8,-4]])
axes[0].scatter(w[:,0],w[:,1],c='black',s=15,zorder=10)
axes[0].set(xlabel='y (m)',ylabel='z (m)',title='Same waypoint geometry')
axes[1].set(xlabel='Time (s)',ylabel='Speed (m/s)',title='Free endpoint derivatives')
axes[2].set(xlabel='Time (s)',ylabel='Snap norm (m/s^4)',title='Uniform duration scaling')
axes[1].legend();finish(fig,'minimum_snap.png')
if args.main_only:
    export_quadrotor_animation()
    print(f'Rendered main figures from saved traces in {FIG}; metrics unchanged.')
    raise SystemExit(0)

fig,axes=plt.subplots(1,3,figsize=(11,3.5))
for name,label in [('lqr_r0.1','Clipped LQR, R=0.1 I'),('mpc_r0.1','MPC, R=0.1 I')]:
    d=trace('quadrotor',name);axes[0].plot(d['time'],np.linalg.norm(d['state'][:,:2],axis=1),label=label)
for method in ['lqr','mpc']:
    d=trace('quadrotor',f'{method}_mismatch_seed0');axes[1].plot(d['time'],np.linalg.norm(d['state'][:,:2],axis=1),label=method.upper())
axes[0].set(xlabel='Time (s)',ylabel='Position error (m)',title='Aggressive LQR + clipping fails');axes[0].legend(fontsize=8)
axes[1].set(xlabel='Time (s)',ylabel='Position error (m)',title='10% heavier plant, seed 0');axes[1].legend()
d=trace('quadrotor','lqr_r0.1');axes[2].plot(d['time'][:-1],d['command'][:,0],label='Commanded motor 1');axes[2].plot(d['time'][:-1],d['applied'][:,0],label='Applied motor 1')
axes[2].set(xlabel='Time (s)',ylabel='Thrust (N)',title='Clipping is not optimization');axes[2].legend(fontsize=8)
finish(fig,'quadrotor_failures.png')

fig,axes=plt.subplots(1,3,figsize=(11,3.5))
summary=json.loads((sources['ilqr']/'summary.json').read_text())
for row in summary['results']:
    axes[0].semilogy(row['metrics']['accepted_cost_history'],label=row['name'])
d=trace('ilqr','seed0');t=d['time'];x=d['state'];u=d['input']
axes[0].set(xlabel='Accepted iteration',ylabel='Quadratic objective',title='Iteration limit: not converged');axes[0].legend(fontsize=8)
axes[1].plot(t,x[:,3],label='Pendulum');axes[1].plot(t,x[:,2],label='Quadrotor');axes[1].axhline(np.pi,c='grey',ls=':');axes[1].set(xlabel='Time (s)',ylabel='Angle (rad)');axes[1].legend(fontsize=8)
axes[2].plot(t[:-1],u);axes[2].axhline(0,c='black',ls=':');axes[2].set(xlabel='Time (s)',ylabel='Thrust (N)',title='Unconstrained inputs\nNegative thrust occurs')
finish(fig,'ilqr.png')

fig,axes=plt.subplots(1,2,figsize=(9,3.7))
from control_portfolio.acrobot import end_effector
from matplotlib.patches import Rectangle
for i,initialization in enumerate(['acrobot','acrobot_folded']):
    d=trace(initialization,'seed0')
    for key,label in [('spline_state','Collocation spline (infeasible)'),('independent_state','Independent input replay')]:
        points=end_effector(d[key]);axes[i].plot(points[:,0],points[:,1],label=label)
    for center in [-1.5,1.5]:axes[i].add_patch(Rectangle((center-1,-.5),2,1,color='grey',alpha=.3))
    axes[i].set(xlabel='x (m)',ylabel='z (m)',title=f'{initialization}: failed trial');axes[i].axis('equal');axes[i].legend(fontsize=7)
finish(fig,'acrobot_failures.png')

export_quadrotor_animation()
print('Generated figures and curated metrics in docs/',flush=True)
