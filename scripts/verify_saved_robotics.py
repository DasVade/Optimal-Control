"""Independent integration checks on saved results; original runs are never edited."""
import importlib.metadata
import json
from pathlib import Path
import numpy as np
from control_portfolio.quadrotor import Controller,simulate
from control_portfolio.ilqr import step
ROOT=Path(__file__).resolve().parents[1]
destination=ROOT/'outputs/verification_20261023'
destination.mkdir(exist_ok=False)
checks=[]
for method in ['lqr','mpc']:
    saved=np.load(ROOT/f'outputs/quadrotor_20261023/{method+"_ic1" if method=="lqr" else "mpc_n10_ic1"}/trajectory.npz')
    fresh,metrics,_=simulate(Controller(method),[.5,.5,0,1,1,0],integration_dt=.0025)
    checks.append(dict(module='quadrotor',method=method,comparison='RK4 integration dt .005 versus .0025 s, same held controller period .02 s',
        max_state_difference=float(np.max(np.abs(fresh['state']-saved['state']))),metrics=metrics))
for seed in [0,1,2]:
    saved=np.load(ROOT/f'outputs/ilqr_20261023/seed{seed}/trajectory.npz')
    xx=[saved['state'][0].copy()]
    for u in saved['input']:
        xx.append(step(step(xx[-1],u,.01),u,.01))
    delta=np.array(xx)-saved['state']
    checks.append(dict(module='ilqr',seed=seed,comparison='Independent open-loop replay: two RK4 .01 s steps per saved .02 s action',
        max_state_difference=float(np.max(np.abs(delta))),state_rmse=float(np.sqrt(np.mean(delta**2))),
        final_state_difference=delta[-1].tolist()))
record=dict(checks=checks,dependencies={name:importlib.metadata.version(name) for name in ['numpy','scipy','matplotlib','pillow','osqp']},
    note='Supplemental verification recorded after the saved optimization runs; original run files unchanged.')
(destination/'verification.json').write_text(json.dumps(record,indent=2))
print(json.dumps(record,indent=2))
