"""New acceleration-level OSC QP core; NOT demonstrated walking.

The supplied model/planner require Drake. This module has a portable matrix-level
test, but that does not validate multibody Jacobians or a walking simulation.
"""
import importlib.util
from pathlib import Path
import numpy as np
from scipy import sparse
import osqp


def solve_qp(mass,bias,gravity,actuation,contact_jacobian,contact_bias,tasks,limits,mu=1.,regularization=1e-5):
    """M a + Cv + G = B u + J.T lambda; lambda world x,y,z.

tasks: (J, Jdot*v, desired acceleration, weight). Gravity is on LEFT.
Returns no stale torque on failure: caller must handle a reported failure.
"""
    nv=mass.shape[0];nu=actuation.shape[1];n=nv+nu+3
    h=np.zeros((n,n));g=np.zeros(n);h[:nv,:nv]=2*regularization*np.eye(nv)
    for j,jbias,command,w in tasks:
        h[:nv,:nv]+=2*j.T@w@j;g[:nv]+=-2*j.T@w@(command-jbias)
    dynamics=np.hstack([mass,-actuation,-contact_jacobian.T])
    contact=np.hstack([contact_jacobian,np.zeros((3,nu+3))])
    force=np.zeros((4,n));force[:,-3:]=[[1,0,-mu],[-1,0,-mu],[0,0,-1],[0,1,0]]
    inputs=np.hstack([np.zeros((nu,nv)),np.eye(nu),np.zeros((nu,3))])
    a=sparse.csc_matrix(np.vstack([dynamics,contact,force,inputs]))
    lower=np.r_[-bias-gravity,-contact_bias,[-np.inf]*3,0.,-np.asarray(limits)]
    upper=np.r_[-bias-gravity,-contact_bias,np.zeros(4),limits]
    solver=osqp.OSQP();solver.setup(P=sparse.csc_matrix(h),q=g,A=a,l=lower,u=upper,
        verbose=False,eps_abs=1e-7,eps_rel=1e-7,max_iter=10000,polishing=True)
    result=solver.solve(raise_error=False)
    residual=None if result.x is None else float(max(np.max(lower-a@result.x),np.max(a@result.x-upper),0))
    return result.x,dict(status=result.info.status,success=result.info.status_val==1,
                        constraint_violation=residual)


def attempt(folder):
    from .robotics_experiments import save
    root=Path(__file__).resolve().parents[2];source=root/'course_sources/hw6'
    required=['planar_walker.urdf','footstep_planner.py','osc.py','osc_tracking_objective.py']
    missing=[str(source/name) for name in required if not (source/name).exists()]
    record=dict(module='OSC',source_materials_present=not missing,missing=missing,
                demonstrated_walking=False,portable_qp_only=True)
    if importlib.util.find_spec('pydrake') is None:
        record.update(status='blocked',reason='pydrake is not installed in the active Windows environment. Existing WSL Ubuntu Python 3.10.12 also reports ModuleNotFoundError: pydrake and has no pip. Temporary WSL environment creation was declined; no further environment modifications attempted.')
    else:
        record.update(status='unverified',reason='Drake detected; full multibody wiring, Jacobian checks and walking simulation remain required. Matrix QP alone is insufficient.')
    save(folder/'status.json',record);print(record['status'],record['reason'],flush=True)
    return [record]
