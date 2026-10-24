"""Execute the preserved completed HW4 baseline in Drake; never edit it."""
from contextlib import redirect_stdout
import argparse
import importlib
import io
import json
from pathlib import Path
import sys
import traceback
import numpy as np
from control_portfolio.minsnap import solve,evaluate

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=root/'outputs/drake_minsnap_verification_20261023')
folder=parser.parse_args().output;folder.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(root/'course_sources/hw4_completed'))
original=importlib.import_module('minsnap')
waypoints=np.array([[-3,-4],[0,0],[2,3],[4,0],[8,-4]],float)
durations=np.ones(4);capture=io.StringIO()
record=dict(source='course_sources/hw4_completed/minsnap.py',waypoints=waypoints.tolist(),durations_s=durations.tolist(),terms=14)
try:
    with redirect_stdout(capture):curve=original.minsnap(4,14,waypoints,durations)
    scaled=solve(waypoints,durations)
    t=np.linspace(0,4,1001);pred=curve(t);reference=np.array([evaluate(scaled,time) for time in t])
    continuity=[]
    for knot in [1.,2.,3.]:
        for order in range(1,5):continuity.append(float(np.max(np.abs(curve(knot-1e-9,order)-curve(knot+1e-9,order)))))
    nodes,weights=np.polynomial.legendre.leggauss(14)
    cost=sum(.5*np.sum(weights*np.sum(curve(i+(nodes+1)/2,4)**2,axis=1)) for i in range(4))
    record.update(execution_completed=True,original_solver_stdout=capture.getvalue(),
        original_waypoint_residual=float(np.max(np.abs(curve(np.arange(5))-waypoints))),
        original_continuity_residual_one_sided=float(max(continuity)),original_snap_integral=float(cost),
        scaled_snap_integral=scaled['metrics']['snap_integral'],
        max_position_difference_m=float(np.max(np.abs(pred-reference))),
        note='Baseline solver status, residuals and conditioning must be interpreted together; execution is not automatically numerical validation.')
    np.savez_compressed(folder/'comparison.npz',time=t,original=pred,scaled=reference)
except Exception as error:
    record.update(execution_completed=False,error=f'{type(error).__name__}: {error}',original_solver_stdout=capture.getvalue())
    (folder/'exception.txt').write_text(traceback.format_exc())
(folder/'result.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2))
