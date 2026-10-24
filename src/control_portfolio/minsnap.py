"""New numerically scaled solver for the completed HW4 formulation.

Ascending powers in tau=t/T. Solve equality-constrained least squares of
quadrature-weighted snap, without squaring its condition number in a Hessian.
Endpoint derivatives are FREE. Interior derivatives 1 through 4 are continuous.
"""
import math
import numpy as np
from scipy.linalg import null_space, block_diag


def basis(tau, order=0, terms=14):
    return np.array([math.factorial(j)/math.factorial(j-order)*tau**(j-order)
                     if j>=order else 0. for j in range(terms)])


def solve(waypoints, durations, terms=14):
    w=np.asarray(waypoints,float); durations=np.asarray(durations,float)
    n=len(durations)
    if len(w)!=n+1 or np.any(durations<=0) or terms<8: raise ValueError('Invalid trajectory dimensions')
    rows=[]; targets=[]
    for i,T in enumerate(durations):
        for tau,point in [(0.,w[i]),(1.,w[i+1])]:
            row=np.zeros(n*terms); row[i*terms:(i+1)*terms]=basis(tau,terms=terms)
            rows.append(row); targets.append(point)
    for i in range(n-1):
        for k in range(1,5):
            row=np.zeros(n*terms)
            row[i*terms:(i+1)*terms]=basis(1.,k,terms)/durations[i]**k
            row[(i+1)*terms:(i+2)*terms]=-basis(0.,k,terms)/durations[i+1]**k
            rows.append(row); targets.append(np.zeros(w.shape[1]))
    a=np.array(rows); b=np.array(targets)
    scale=np.linalg.norm(a,axis=1); a_scaled=a/scale[:,None]; b_scaled=b/scale[:,None]
    particular=np.linalg.lstsq(a_scaled,b_scaled,rcond=None)[0]
    null=null_space(a_scaled)
    # Gauss quadrature exactly integrates degree 2*(terms-5) squared snap.
    nodes,weights=np.polynomial.legendre.leggauss(terms)
    snap=np.array([basis(t,4,terms)*np.sqrt(weight) for t,weight in zip((nodes+1)/2,weights/2)])
    l=block_diag(*[snap/T**3.5 for T in durations])
    reduced=l@null
    correction,_,rank,singular=np.linalg.lstsq(reduced,-l@particular,rcond=None)
    coefficients=particular+null@correction
    residual=a@coefficients-b
    return dict(coefficients=coefficients.reshape(n,terms,-1),durations=durations,
        metrics=dict(snap_integral=float(np.sum((l@coefficients)**2)),
        waypoint_residual=float(np.max(np.abs(residual[:2*n]))),
        continuity_residual=float(np.max(np.abs(residual[2*n:]))),
        reduced_rank=int(rank),reduced_condition=float(singular[0]/singular[-1]),
        projected_gradient_norm=float(np.linalg.norm(reduced.T@(l@coefficients))),
        status='equality-nullspace least squares',endpoint_derivatives='unconstrained'))


def evaluate(solution,t,order=0):
    durations=solution['durations']; knots=np.r_[0,np.cumsum(durations)]
    i=min(np.searchsorted(knots,t,side='right')-1,len(durations)-1); i=max(i,0)
    c=solution['coefficients'][i]
    return basis((t-knots[i])/durations[i],order,len(c))@c/durations[i]**order
