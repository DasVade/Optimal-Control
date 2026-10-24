import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time

import numpy as np
from .cartpole import Controller,Parameters
from .simulation import simulate,metrics
from .unicycle import experiment

ROOT=Path(__file__).resolve().parents[2]


def write_json(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')


def run_suite(output,config):
    nominal=Parameters(**config['nominal_parameters']);rows=[]
    cases=[]
    def add(name,group,offset=(.1,.1,0,0),r=1.,q=None,limit=180.,legacy=False,mode='lqr',plant=nominal,seed=None,absolute=False,dt=None,duration=None):
        x=np.array(offset,dtype=float)
        if not absolute:x[1]+=np.pi
        cases.append(dict(name=name,group=group,initial=x.tolist(),r=r,q=list(q or config['q_diagonal']),limit=limit,legacy=legacy,mode=mode,
                          plant=plant.__dict__,seed=seed,dt=dt or config['dt_s'],duration=duration or config['duration_s']))
    for n,initial in enumerate(config['near_upright_initial_conditions']):
        for r in config['r_values']:add(f'r{r}_ic{n}','r_weight',initial,r=r)
        for qangle in [10.,100.]:add(f'qtheta{qangle}_ic{n}','q_weight',initial,q=(1,qangle,1,10))
        for limit in config['force_limits_N']:add(f'limit{limit}_ic{n}','actuator',initial,limit=limit)
        for legacy in [False,True]:add(f'linearization_legacy{legacy}_ic{n}','linearization',initial,legacy=legacy)
    for angle in [.1,.4,.8,1.2]:
        for mode in ['lqr','hybrid']:add(f'basin_{angle}_{mode}','basin',(0,angle,0,0),mode=mode)
    for angle,label in [(0.,'exact_down'),(np.pi/6,'30deg'),(np.pi/6+2*np.pi,'wrapped_30deg')]:
        add('swing_'+label,'swingup',(0,angle,0,0),mode='hybrid',absolute=True)
    for seed in config['seeds']:
        for r in [0.1,1.,10.]:add(f'disturb_r{r}_seed{seed}','disturbance',r=r,seed=seed)
        for changed in [False,True]:
            p=Parameters(mp=1.25,length=1.15) if changed else nominal
            add(f'mismatch{changed}_seed{seed}','mismatch',plant=p,seed=seed)
    for dt in [.02,.01,.005]:add(f'convergence_dt{dt}','integration',dt=dt)
    add('swing_convergence_dt0.005','integration',(0,np.pi/6,0,0),absolute=True,mode='hybrid',dt=.005)
    for case in cases:
        controller=Controller(nominal,case['q'],case['r'],case['limit'],case['legacy'],case['mode'])
        amplitude=float(np.random.default_rng(case['seed']).uniform(-2,2)) if case['seed'] is not None else 0.
        forcing=lambda t,a=amplitude:a if 5<=t<6 else 0.
        run=simulate(controller,case['initial'],case['duration'],case['dt'],Parameters(**case['plant']),forcing)
        row={**case,'disturbance_force_N':amplitude,'metrics':metrics(run,case['limit']),
             'gain':controller.k.tolist(),'design_closed_loop_eigenvalues_real':controller.closed_loop_eigenvalues.real.tolist()}
        rows.append(row)
        folder=output/'cartpole'/case['name'];folder.mkdir(parents=True)
        np.savez_compressed(folder/'trajectory.npz',**{k:run[k] for k in ['time','state','input','raw_input','disturbance']})
        write_json(folder/'result.json',row)
        print(case['name'],json.dumps(row['metrics']),flush=True)
    uni=[]
    for seed in config['seeds']:
        uconfig=config['unicycle']
        for label,heading,scale in [('nominal',0.,1.),('heading_offset',uconfig['heading_offset_rad'],1.),('speed_mismatch',0.,uconfig['speed_scale_mismatch'])]:
            run=experiment(seed,heading,scale,uconfig['duration_s'],uconfig['integration_max_step_s'],uconfig['rtol'],uconfig['atol'])
            folder=output/'unicycle'/f'{label}_seed{seed}';folder.mkdir(parents=True)
            np.savez_compressed(folder/'trajectory.npz',**{k:run[k] for k in ['time','state','desired','input']})
            row={'condition':label,**run['metrics'],'obstacle':run['obstacle'].__dict__}
            write_json(folder/'result.json',row);uni.append(row);print('unicycle',row,flush=True)
    write_json(output/'results.json',{'config':config,'cartpole':rows,'unicycle':uni})
    flattened=[{'name':r['name'],'group':r['group'],**r['metrics']} for r in rows]
    columns=list(dict.fromkeys(k for row in flattened for k in row))
    with (output/'cartpole_metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(flattened)
    return len(cases),len(uni)


def main():
    parser=argparse.ArgumentParser(description='Audited cart-pole and unicycle coursework experiments')
    parser.add_argument('task',choices=['benchmark','suite'])
    parser.add_argument('--config',type=Path,default=ROOT/'configs/experiments.json')
    parser.add_argument('--output',type=Path,default=None)
    args=parser.parse_args()
    output=args.output or ROOT/'outputs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    if output.exists():raise FileExistsError(f'Refusing to overwrite an existing experiment: {output}')
    output.mkdir(parents=True)
    config=json.loads(args.config.read_text())
    write_json(output/'config.json',config)
    start=time.perf_counter()
    if args.task=='benchmark':
        run=simulate(Controller(mode='lqr'),[.1,np.pi+.1,0,0],2.,.01)
        result={'two_second_simulation_wall_seconds':run['elapsed_seconds'],
                'estimated_60_twenty_second_runs_seconds':run['elapsed_seconds']*10*60,
                'note':'Linear extrapolation; excludes unicycle, plotting, CARE solves and difficult hybrid trajectories.'}
        write_json(output/'benchmark.json',result);print(json.dumps(result,indent=2))
    else:
        counts=run_suite(output,config);print('Completed',counts,flush=True)
    try:commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    except subprocess.CalledProcessError:commit=None
    write_json(output/'environment.json',{'python':platform.python_version(),'platform':platform.platform(),
              'cpu':platform.processor(),'logical_cpu_count':os.cpu_count(),
              'numpy':np.__version__,'elapsed_seconds':time.perf_counter()-start,'base_commit':commit,
              'configuration_sha256':hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'execution':'Local CPU only; no paid jobs or hardware experiments.'})


if __name__=='__main__':main()
