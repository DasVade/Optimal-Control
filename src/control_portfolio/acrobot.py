"""New HW5 Hermite-Simpson implementation using the supplied URDF parameters.

q0 is absolute, q1 relative; downward zero. Only elbow torque, +/-30 Nm.
Both rod COM inertias about y are .33 kg m^2, plus the 2 kg tip mass.
End-effector avoidance only; final configuration fixed, final velocity free.
"""
import time
import numpy as np
from scipy.optimize import minimize, Bounds
from scipy.interpolate import CubicHermiteSpline
from scipy.integrate import solve_ivp


def dynamics(x,u):
    q0,q1,v0,v1=np.moveaxis(np.asarray(x),-1,0)
    c=np.cos(q1); s=np.sin(q1)
    # M11=I1+I2+m1*r1^2+m2*(l1^2+r2^2+2*l1*r2*c)+mb*(l1^2+l2^2+2*l1*l2*c)
    m11=6.16+5*c; m12=2.58+2.5*c; m22=2.58
    h=2.5*s
    c0=-h*(2*v0*v1+v1*v1); c1=h*v0*v0
    g0=9.81*(3.5*np.sin(q0)+2.5*np.sin(q0+q1)); g1=9.81*2.5*np.sin(q0+q1)
    rhs0=-c0-g0; rhs1=np.asarray(u)-c1-g1
    det=m11*m22-m12*m12
    return np.stack([v0,v1,(m22*rhs0-m12*rhs1)/det,(-m12*rhs0+m11*rhs1)/det],axis=-1)


def end_effector(x):
    q0,q1=np.asarray(x)[...,:2].T
    return np.stack([-np.sin(q0)-np.sin(q0+q1),-np.cos(q0)-np.cos(q0+q1)],axis=-1)


def clearance(x,centers,width=2.,height=1.):
    pos=end_effector(x)
    delta=np.abs(pos[...,None,:]-np.asarray(centers))-np.array([width,height])/2
    return np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(np.max(delta,axis=-1),0)-.1


def defects(x,u,dt):
    f=dynamics(x,u)
    mid=.5*(x[:-1]+x[1:])+dt/8*(f[:-1]-f[1:])
    mdot=1.5/dt*(x[1:]-x[:-1])-.25*(f[:-1]+f[1:])
    return mdot-dynamics(mid,.5*(u[:-1]+u[1:]))


def solve(nodes=21,dt=.175,seed=0,max_iterations=250,width=2.,initialization='linear'):
    start=time.perf_counter(); rng=np.random.default_rng(seed); t=np.arange(nodes)*dt
    centers=[[-1.5,0],[1.5,0]]
    initial=np.zeros(4); target=np.array([np.pi,0.])
    def unpack(z):return z[:nodes*4].reshape(nodes,4),z[nodes*4:]
    def equality(z):
        x,u=unpack(z)
        return np.r_[x[0]-initial,x[-1,:2]-target,defects(x,u,dt).ravel()]
    def inequality(z):
        x,u=unpack(z); f=dynamics(x,u)
        mid=.5*(x[:-1]+x[1:])+dt/8*(f[:-1]-f[1:])
        return np.r_[clearance(x,centers,width).ravel(),clearance(mid,centers,width).ravel()]
    def objective(z):
        u=z[nodes*4:];return dt*(np.sum(u*u)-.5*(u[0]**2+u[-1]**2))
    def gradient(z):
        g=np.zeros_like(z);g[nodes*4:]=2*dt*z[nodes*4:];g[nodes*4]*=.5;g[-1]*=.5;return g
    x=np.zeros((nodes,4));x[:,0]=np.linspace(0,np.pi,nodes)
    if initialization=='folded':
        x[:,1]=-np.pi*np.sin(np.linspace(0,np.pi,nodes))
        x[:,2:]=np.gradient(x[:,:2],dt,axis=0)
    elif initialization!='linear':raise ValueError(initialization)
    x[1:-1,:2]+=rng.normal(0,.3,(nodes-2,2))
    if initialization=='linear':x[:,2]=np.pi/t[-1]
    x[0]=0
    z=np.r_[x.ravel(),rng.normal(0,3,nodes)]
    lower=np.r_[np.tile([-np.pi,-np.pi,-15,-15],nodes),np.full(nodes,-30)]
    upper=-lower
    result=minimize(objective,z,jac=gradient,method='SLSQP',bounds=Bounds(lower,upper),
        constraints=[dict(type='eq',fun=equality),dict(type='ineq',fun=inequality)],
        options=dict(maxiter=max_iterations,ftol=1e-8,disp=False))
    x,u=unpack(result.x); spline=CubicHermiteSpline(t,x,dynamics(x,u))
    dense=np.linspace(0,t[-1],2001); interpolated=spline(dense)
    # Independent adaptive integration of first-order hold, consistent with collocation inputs.
    independent=solve_ivp(lambda time,state:dynamics(state,np.interp(time,t,u)),[0,t[-1]],initial,
                          t_eval=dense,rtol=1e-9,atol=1e-11,max_step=dt/8)
    defect=float(np.max(np.abs(defects(x,u,dt)))); coll_clear=float(np.min(clearance(interpolated,centers,width)))
    actual_clear=float(np.min(clearance(independent.y.T,centers,width)))
    discrepancy=float(np.sqrt(np.mean((independent.y.T-interpolated[:len(independent.t)])**2)))
    metrics=dict(seed=seed,initialization=initialization,solver_success=bool(result.success),status=result.message,iterations=int(result.nit),
        objective_Nm2s=float(result.fun),max_dynamics_defect=defect,
        boundary_residual=float(np.max(np.abs(np.r_[x[0],x[-1,:2]-target]))),
        node_and_midpoint_clearance_m=float(np.min(inequality(result.x))),dense_spline_clearance_m=coll_clear,
        independent_clearance_m=actual_clear,independent_final_configuration_error_rad=float(np.linalg.norm(independent.y[:2,-1]-target)),
        independent_state_rmse=discrepancy,independent_integration_success=bool(independent.success),
        peak_input_Nm=float(np.max(np.abs(u))),peak_node_velocity_rad_s=float(np.max(np.abs(x[:,2:]))),
        feasible_nodes=bool(result.success and defect<1e-5 and np.min(inequality(result.x))>=-1e-6),
        validated_swingup=bool(result.success and defect<1e-5 and coll_clear>=-1e-4 and actual_clear>=-1e-4 and np.linalg.norm(independent.y[:2,-1]-target)<.05),
        wall_seconds=time.perf_counter()-start)
    return dict(time=t,state=x,input=u,dense_time=independent.t,independent_state=independent.y.T,spline_state=interpolated),metrics
