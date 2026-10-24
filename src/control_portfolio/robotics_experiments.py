"""Experiment dispatch; immutable output folders are managed by the existing CLI."""
import csv
import json
import time
from pathlib import Path
import numpy as np


def save(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')


def export(folder,name,trajectory,metrics,settings,logs=None):
    destination=folder/name; destination.mkdir(parents=True)
    np.savez_compressed(destination/'trajectory.npz',**trajectory)
    save(destination/'result.json',dict(settings=settings,metrics=metrics))
    if logs is not None:save(destination/'solver_log.json',logs)
    state=trajectory['state'];t=trajectory['time']
    np.savetxt(destination/'state.csv',np.column_stack([t,state]),delimiter=',',
               header='time_s,'+','.join(f'x{i}' for i in range(state.shape[1])),comments='')
    for key in ('command','applied','input'):
        if key in trajectory:
            value=np.asarray(trajectory[key]);value=value[:,None] if value.ndim==1 else value
            np.savetxt(destination/f'{key}.csv',np.column_stack([t[:len(value)],value]),delimiter=',',
                header='time_s,'+','.join(f'u{i}' for i in range(value.shape[1])),comments='')
    return dict(name=name,settings=settings,metrics=metrics)


def run(task,folder,config):
    rows=[];start=time.perf_counter()
    if task in ('quadrotor','robotics-benchmark'):
        from .quadrotor import Controller,simulate
        cfg=config['quadrotor']; cases=[]
        def add(name,method,initial,horizon=10,r=1.,mass=1.,disturbance=0.,seed=None):
            cases.append(dict(name=name,method=method,initial=initial,horizon=horizon,r=r,mass=mass,disturbance=disturbance,seed=seed))
        if task=='robotics-benchmark':add('short_mpc','mpc',cfg['initial_states'][1])
        else:
            for i,x in enumerate(cfg['initial_states']):
                add(f'lqr_ic{i}','lqr',x)
                for n in cfg['horizon_states']:add(f'mpc_n{n}_ic{i}','mpc',x,n)
            for r in cfg['r_scales']:
                for method in ['lqr','mpc']:add(f'{method}_r{r}',method,cfg['initial_states'][1],r=r)
            for seed in cfg['seeds']:
                force=float(np.random.default_rng(seed).uniform(*cfg['disturbance_range_N']))
                for method in ['lqr','mpc']:
                    add(f'{method}_disturb_seed{seed}',method,cfg['initial_states'][1],disturbance=force,seed=seed)
                    add(f'{method}_mismatch_seed{seed}',method,cfg['initial_states'][1],mass=cfg['mismatch_mass_kg'],disturbance=force,seed=seed)
        for case in cases:
            controller=Controller(case['method'],case['horizon'],cfg['prediction_dt_s'],case['r'])
            trajectory,metrics,logs=simulate(controller,case['initial'],1. if task=='robotics-benchmark' else cfg['duration_s'],
                cfg['control_dt_s'],cfg['integration_dt_s'],case['mass'],case['disturbance'])
            rows.append(export(folder,case['name'],trajectory,metrics,case,logs))
            print(case['name'],metrics['position_rmse_m'],metrics['success'],flush=True)
    elif task=='minsnap':
        from .minsnap import solve,evaluate
        cfg=config['minsnap']
        for scale in cfg['duration_scales']:
            solution=solve(cfg['waypoints'],[scale]*(len(cfg['waypoints'])-1),cfg['terms'])
            t=np.linspace(0,sum(solution['durations']),1001)
            trajectory=dict(time=t,state=np.array([evaluate(solution,z) for z in t]),coefficients=solution['coefficients'])
            for order in range(1,5):trajectory[f'derivative{order}']=np.array([evaluate(solution,z,order) for z in t])
            metrics=solution['metrics']
            for order in range(1,5):metrics[f'peak_derivative{order}']=float(np.max(np.linalg.norm(trajectory[f'derivative{order}'],axis=1)))
            rows.append(export(folder,f'duration{scale}',trajectory,metrics,dict(duration_scale=scale,terms=cfg['terms'],endpoint_derivatives='free')))
    elif task in ('ilqr','ilqr-benchmark'):
        from .ilqr import solve
        cfg=config['ilqr']
        for seed in ([0] if task.endswith('benchmark') else cfg['seeds']):
            rng=np.random.default_rng(seed);guess=np.full((cfg['horizon_states']-1,2),1.2*9.81/2)
            if seed:guess+=rng.normal(0,cfg['initial_input_noise_std_N'],guess.shape)
            trajectory,metrics=solve(np.array(cfg['initial_state']),np.array(cfg['goal_state']),guess,cfg['dt_s'],
                3 if task.endswith('benchmark') else cfg['max_iterations'],cfg['tolerance'])
            rows.append(export(folder,f'seed{seed}',trajectory,metrics,dict(seed=seed,**cfg)))
            print('iLQR',seed,metrics['status'],metrics['final_cost'],flush=True)
    elif task in ('acrobot','acrobot-benchmark'):
        from .acrobot import solve
        cfg=config['acrobot']
        for seed in ([0] if task.endswith('benchmark') else cfg['seeds']):
            trajectory,metrics=solve(cfg['nodes'],cfg['dt_s'],seed,3 if task.endswith('benchmark') else cfg['max_iterations'],cfg['box_width_m'],cfg.get('initialization','linear'))
            rows.append(export(folder,f'seed{seed}',trajectory,metrics,dict(seed=seed,**cfg)))
            print('Acrobot',seed,metrics['status'],metrics['validated_swingup'],flush=True)
    elif task=='supporting':
        from .supporting import run_supporting
        rows=run_supporting(folder)
    elif task=='osc':
        from .osc import attempt
        rows=attempt(folder)
    else:raise ValueError(task)
    save(folder/'summary.json',dict(task=task,results=rows,elapsed_seconds=time.perf_counter()-start))
    return rows
