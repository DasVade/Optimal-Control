"""New HW5 iLQR completion. Unconstrained inputs, local optimization only.

The mass matrix is the supplied Lagrangian's, not a rigid quadrotor surrogate.
RK4 rollout and complex-step derivatives of that SAME discrete map are used.
"""
import time
import numpy as np
from .quadrotor import rk4


def dynamics(x,u):
    theta,phi=x[2:4]; phidot=x[7]; m=1.; p=.2; length=1.
    matrix=np.array([[m+p,0,0,p*length*np.cos(phi)],
                     [0,m+p,0,p*length*np.sin(phi)],
                     [0,0,.0625,0],
                     [p*length*np.cos(phi),p*length*np.sin(phi),0,p*length**2]])
    c=np.array([-p*length*np.sin(phi)*phidot**2,p*length*np.cos(phi)*phidot**2,0,0])
    gravity=np.array([0,-(m+p)*9.81,0,-p*9.81*length*np.sin(phi)])
    force=np.array([-np.sin(theta)*sum(u),np.cos(theta)*sum(u),.25*(u[0]-u[1]),0])
    return np.r_[x[4:],np.linalg.solve(matrix,force+gravity-c)]


def step(x,u,dt):return rk4(dynamics,x,u,dt)


def jacobians(x,u,dt):
    eps=1e-25
    a=np.column_stack([np.imag(step(x.astype(complex)+1j*eps*np.eye(8)[j],u,dt))/eps for j in range(8)])
    b=np.column_stack([np.imag(step(x,u.astype(complex)+1j*eps*np.eye(2)[j],dt))/eps for j in range(2)])
    return a,b


def solve(initial,goal,guess,dt=.02,max_iterations=150,tolerance=1e-5):
    start=time.perf_counter(); u=np.array(guess,float); n=len(u)
    q=np.eye(8); r=np.eye(2)*1e-3; qf=np.eye(8)*100
    # Both masses must be supported at hover. Original template omitted pendulum mass.
    hover=np.full(2,1.2*9.81/2)
    def rollout(inputs):
        xx=[np.array(initial,float)]
        for action in inputs:xx.append(step(xx[-1],action,dt))
        return np.array(xx)
    def cost(xx,uu):
        e=xx-goal; v=uu-hover
        return float(.5*dt*(np.einsum('ni,ij,nj',e[:-1],q,e[:-1])+np.einsum('ni,ij,nj',v,r,v))+.5*e[-1]@qf@e[-1])
    x=rollout(u); value=cost(x,u); history=[value]; attempts=[]; rho=1e-5; status='iteration limit'
    for iteration in range(max_iterations):
        derivatives=[jacobians(x[k],u[k],dt) for k in range(n)]
        vx=qf@(x[-1]-goal); vxx=qf.copy(); gains=np.zeros((n,2,8)); feed=np.zeros((n,2))
        try:
            for k in reversed(range(n)):
                a,b=derivatives[k]; qx=dt*q@(x[k]-goal)+a.T@vx
                qu=dt*r@(u[k]-hover)+b.T@vx
                qxx=dt*q+a.T@vxx@a; quu=dt*r+b.T@vxx@b; qux=b.T@vxx@a
                regular=quu+rho*np.eye(2); np.linalg.cholesky(regular)
                d=-np.linalg.solve(regular,qu); gain=-np.linalg.solve(regular,qux)
                feed[k]=d; gains[k]=gain
                vx=qx+gain.T@qu+qux.T@d+gain.T@quu@d
                vxx=qxx+gain.T@qux+qux.T@gain+gain.T@quu@gain; vxx=(vxx+vxx.T)/2
        except np.linalg.LinAlgError:
            rho*=10; attempts.append(dict(iteration=iteration,status='nonpositive backward Hessian',rho=rho))
            if rho>1e8:status='regularization limit';break
            continue
        accepted=False
        for alpha in [1.,.5,.25,.125,.0625,.03125,.015625]:
            xn=[x[0].copy()]; un=[]
            with np.errstate(over='ignore',invalid='ignore'):
                for k in range(n):
                    action=u[k]+alpha*feed[k]+gains[k]@(xn[-1]-x[k]); un.append(action)
                    xn.append(step(xn[-1],action,dt))
                xn=np.array(xn); un=np.array(un); new=cost(xn,un)
            if np.isfinite(new) and new<value:
                improvement=value-new; x,u,value=xn,un,new; history.append(value)
                attempts.append(dict(iteration=iteration,status='accepted',alpha=alpha,rho=rho,cost=value))
                rho=max(1e-8,rho/3); accepted=True
                if improvement<tolerance*(1+abs(value)):status='relative cost tolerance'
                break
        if status=='relative cost tolerance':break
        if not accepted:
            rho*=10; attempts.append(dict(iteration=iteration,status='line search failed',rho=rho))
            if rho>1e8:status='line search/regularization limit';break
    return dict(time=np.arange(n+1)*dt,state=x,input=u),dict(status=status,
        converged=status=='relative cost tolerance',initial_cost=history[0],final_cost=value,
        final_state_error=(x[-1]-goal).tolist(),final_position_error_m=float(np.linalg.norm(x[-1,:2]-goal[:2])),
        final_pendulum_error_rad=float(x[-1,3]-goal[3]),peak_abs_input_N=float(np.max(np.abs(u))),
        minimum_input_N=float(u.min()),wall_seconds=time.perf_counter()-start,
        accepted_cost_history=history,attempts=attempts)
