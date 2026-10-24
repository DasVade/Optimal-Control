"""Plot saved WSL biped states and export actual model kinematics; no simulation."""
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
from pydrake.all import MultibodyPlant,Parser,RigidTransform

parser=argparse.ArgumentParser()
parser.add_argument('--input',type=Path,default=ROOT/'outputs/osc_wsl_5s_20261023')
parser.add_argument('--prefix',default='osc_wsl')
args=parser.parse_args()
folder=args.input;figures=ROOT/'docs/figures/robotics'
target=ROOT/'docs/results/robotics';target.mkdir(exist_ok=True)
for label in dict.fromkeys(['osc_wsl_benchmark_v2_20261023','osc_wsl_5s_20261023',folder.name]):
    for name in ['environment.json','multibody_checks.json','simulation_metrics.json']:
        shutil.copyfile(ROOT/'outputs'/label/name,target/f'{label}_{name}')
d=np.load(folder/'trajectory.npz');t=d['time'];x=d['state']
log=json.loads((folder/'solver_log.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(11,3.5))
axes[0].plot(t,x[:,0]);axes[0].set(xlabel='Time (s)',ylabel='Forward position (m)')
axes[1].plot(t,x[:,1]);axes[1].set(xlabel='Time (s)',ylabel='Base height (m)')
axes[2].plot(t,x[:,2]);axes[2].set(xlabel='Time (s)',ylabel='Base pitch (rad)')
for r in log:
    if r['fallback']:
        for ax in axes:ax.axvline(r['time_s'],color='red',ls=':',label='QP fallback')
axes[2].legend();fig.suptitle(f'Supplied biped + new OSC adapter: {t[-1]:g} s execution, stability unverified')
fig.tight_layout();fig.savefig(figures/f'{args.prefix}_states.png',dpi=150);plt.close(fig)
plant=MultibodyPlant(0.);Parser(plant).AddModels(str(ROOT/'course_sources/hw6/planar_walker.urdf'))
plant.WeldFrames(plant.world_frame(),plant.GetBodyByName('base').body_frame(),RigidTransform.Identity());plant.Finalize()
context=plant.CreateDefaultContext();points=[]
for state in x[::5]:
    plant.SetPositionsAndVelocities(context,state)
    limbs=[]
    for side in ['left','right']:
        leg=[]
        for name,offset in [(side+'_upper_leg',[0,0,0]),(side+'_lower_leg',[0,0,0]),(side+'_lower_leg',[0,0,-.5])]:
            leg.append(plant.CalcPointsPositions(context,plant.GetBodyByName(name).body_frame(),np.array(offset),plant.world_frame()).ravel()[[0,2]])
        limbs.append(leg)
    points.append(limbs)
points=np.array(points);fig,ax=plt.subplots(figsize=(7,3.5));ax.set(xlim=(min(-.5,float(x[:,0].min())-.5),float(x[:,0].max())+.7),ylim=(-.1,1.4),aspect='equal',xlabel='World x (m)',ylabel='World z (m)')
ax.axhline(0,c='grey');left,=ax.plot([],[],'o-',lw=3,label='Left leg');right,=ax.plot([],[],'o-',lw=3,label='Right leg');ax.legend(loc='upper left');title=ax.set_title('')
def update(i):
    left.set_data(points[i,0,:,0],points[i,0,:,1]);right.set_data(points[i,1,:,0],points[i,1,:,1]);title.set_text(f'Actual Drake states | t={t[::5][i]:.2f} s')
ani=FuncAnimation(fig,update,frames=len(points),interval=50)
ani.save(figures/f'{args.prefix}.gif',writer=PillowWriter(fps=20));plt.close(fig)
print('Exported saved-state plots, model-kinematics GIF and small metrics.')
