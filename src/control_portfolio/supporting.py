"""Small adapters around completed HW1/HW2 source, preserving their provenance."""
import importlib.util
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module;spec.loader.exec_module(module);return module


def run_supporting(folder):
    from .robotics_experiments import save
    source=Path(__file__).resolve().parents[2]/'course_sources'
    grid=load('grid_world',source/'hw2/grid_world.py')
    vi=load('course_value_iteration',source/'hw2/value_iteration.py')
    world=grid.GridWorld();rows=[]
    for gamma in [.9,.1]:
        value,policy=vi.value_iteration(world,1e-8,gamma,False)
        bellman=np.array([min(sum(p*(cost+gamma*value.ravel()[next_state]) for p,next_state,cost in world.P[s][a]) for a in range(4)) for s in range(100)])
        residual=float(np.max(np.abs(value.ravel()-bellman)))
        np.savez_compressed(folder/f'grid_gamma{gamma}.npz',value=value,policy=policy)
        fig,ax=plt.subplots(figsize=(5,4));image=ax.imshow(value);fig.colorbar(image,ax=ax,label='Discounted cost')
        y,x=np.indices(value.shape);dr=np.array([-1,0,1,0]);dc=np.array([0,1,0,-1]);p=policy.astype(int)
        ax.quiver(x,y,dc[p],-dr[p],scale=25);ax.set(xlabel='Column',ylabel='Row',title=f'GridWorld policy, gamma={gamma}')
        fig.tight_layout();fig.savefig(folder/f'grid_gamma{gamma}.png',dpi=150);plt.close(fig)
        rows.append(dict(gamma=gamma,bellman_residual=residual))
    portrait=load('course_phase_portrait',source/'hw1/phase_portrait_p5c.py')
    fig=portrait.plot_phase_portrait();fig.savefig(folder/'phase_portrait.png',dpi=150);plt.close(fig)
    eq=np.array(portrait.equilibria_points());res=max(np.linalg.norm(portrait.f(*point)) for point in eq)
    rows.append(dict(module='HW1',equilibria=eq.tolist(),equilibrium_residual=float(res),
                     note='Original comment has a cubic algebra typo; returned equilibria satisfy the actual dynamics.'))
    save(folder/'supporting_metrics.json',rows)
    return rows
