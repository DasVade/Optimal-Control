"""New HW6 implementation: error-coordinate MPC and matched nonlinear simulation.

State: y,z,theta,vy,vz,omega (SI); inputs: left/right thrust (N).
Prediction uses the handout's forward Euler model; plant uses held-input RK4.
"""
import time
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_continuous_are


def dynamics(x, u, mass=1.0):
    return np.array([*x[3:], -np.sin(x[2])*sum(u)/mass,
                     np.cos(x[2])*sum(u)/mass-9.81, .25*(u[0]-u[1])/.0625])


def linearize():
    a=np.zeros((6,6)); a[:3,3:]=np.eye(3); a[3,2]=-9.81
    b=np.zeros((6,2)); b[4]=1.; b[5]=[4.,-4.]
    return a,b


def rk4(f,x,u,dt):
    k1=f(x,u); k2=f(x+dt*k1/2,u); k3=f(x+dt*k2/2,u); k4=f(x+dt*k3,u)
    return x+dt*(k1+2*k2+2*k3+k4)/6


class Controller:
    def __init__(self, method='mpc', horizon=10, prediction_dt=.1, r_scale=1., max_iter=10000):
        if method not in ('lqr','mpc'): raise ValueError(method)
        if horizon<2 or prediction_dt<=0: raise ValueError('Invalid horizon/timestep')
        self.method=method; self.n=horizon; self.hover=np.full(2,4.905)
        self.q=np.diag([10.,10.,1.,1.,1.,1.]); self.r=np.eye(2)*r_scale
        a,b=linearize(); s=solve_continuous_are(a,b,self.q,self.r)
        self.k=np.linalg.solve(self.r,b.T@s)
        self.ad=np.eye(6)+prediction_dt*a; self.bd=prediction_dt*b
        if method=='lqr':return
        # x[0:N], ue[0:N-1]. Qf=Q, as supplied assignment notebook.
        # Intentionally a discrete sum with no prediction_dt stage multiplier.
        # It is not a finite-horizon quadrature of CARE's continuous objective;
        # multiplying the ENTIRE sum by T maps its terminal matrix to T*Q.
        n=horizon; nx=6*n; nv=nx+2*(n-1)
        p=sparse.block_diag([self.q]*n+[self.r]*(n-1),format='csc')*2
        eq=sparse.lil_matrix((6*n,nv)); eq[:6,:6]=np.eye(6)
        for i in range(n-1):
            row=slice(6*(i+1),6*(i+2))
            eq[row,6*i:6*(i+1)]=-self.ad
            eq[row,6*(i+1):6*(i+2)]=np.eye(6)
            eq[row,nx+2*i:nx+2*i+2]=-self.bd
        bound=sparse.hstack([sparse.csc_matrix((2*(n-1),nx)),sparse.eye(2*(n-1))])
        self.constraint=sparse.vstack([eq,bound],format='csc')
        self.lower=np.r_[np.zeros(6*n),np.tile(-self.hover,n-1)]
        self.upper=np.r_[np.zeros(6*n),np.tile(5.5-self.hover,n-1)]
        self.solver=osqp.OSQP()
        self.solver.setup(P=p,q=np.zeros(nv),A=self.constraint,l=self.lower,u=self.upper,
                          verbose=False,eps_abs=1e-6,eps_rel=1e-6,max_iter=max_iter,polishing=True)

    def command(self,x):
        start=time.perf_counter(); fallback=False; violation=0.
        if self.method=='lqr':
            u=self.hover-self.k@x; status='analytic CARE'; primal=dual=0.
        else:
            self.lower[:6]=x; self.upper[:6]=x
            self.solver.update(l=self.lower,u=self.upper)
            result=self.solver.solve(raise_error=False)
            status=result.info.status; primal=float(result.info.prim_res); dual=float(result.info.dual_res)
            if result.x is not None:
                value=self.constraint@result.x
                violation=float(max(np.max(self.lower-value),np.max(value-self.upper),0.))
            if result.info.status_val==1 and result.x is not None and violation<1e-4:
                u=self.hover+result.x[6*self.n:6*self.n+2]
            else:
                fallback=True; u=self.hover-self.k@x
        return u,dict(status=status,fallback=fallback,primal_residual=primal,dual_residual=dual,
                      constraint_violation=violation,solve_seconds=time.perf_counter()-start)


def simulate(controller, initial, duration=10., control_dt=.02, integration_dt=.005,
             mass=1., disturbance=0.):
    steps=round(duration/control_dt); sub=round(control_dt/integration_dt)
    if not np.isclose(sub*integration_dt,control_dt): raise ValueError('Control period must be an integration-step multiple')
    x=np.array(initial,dtype=float); xx=[x.copy()]; commands=[]; applied=[]; logs=[]
    start=time.perf_counter()
    for i in range(steps):
        raw,log=controller.command(x); u=np.clip(raw,0.,5.5)
        commands.append(raw); applied.append(u); logs.append(log)
        for j in range(sub):
            t=i*control_dt+j*integration_dt
            def f(state,action):
                dx=dynamics(state,action,mass)
                if 2<=t<2.5: dx[3]+=disturbance/mass
                return dx
            x=rk4(f,x,u,integration_dt)
        if not np.all(np.isfinite(x)): raise FloatingPointError('Nonfinite rollout')
        xx.append(x.copy())
    xx=np.array(xx); applied=np.array(applied); commands=np.array(commands)
    t=np.arange(steps+1)*control_dt
    # Componentwise tolerances: m,m,rad,m/s,m/s,rad/s. Remain inside until end.
    tolerances=np.array([.02,.02,.02,.05,.05,.05])
    good=np.all(np.abs(xx)<=tolerances,axis=1)
    settled=np.flatnonzero(np.logical_and.accumulate(good[::-1])[::-1])
    times=np.array([z['solve_seconds'] for z in logs])
    metrics=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(xx[:,:2]**2,axis=1)))),
        final_state_error=xx[-1].tolist(),success=bool(good[-1]),
        settling_time_s=float(t[settled[0]]) if len(settled) else None,
        # Legacy key retained; display name: "Control effort relative to hover".
        # Both signs of deviation contribute, using held APPLIED thrust.
        control_effort_above_hover_N2s=float(np.sum((applied-4.905)**2)*control_dt),
        peak_input_N=float(applied.max()),saturation_fraction=float(np.mean((applied<=1e-7)|(applied>=5.5-1e-7))),
        applied_bound_violation_N=float(max(-applied.min(),applied.max()-5.5,0.)),
        solver_failures=sum(z['fallback'] for z in logs),
        max_constraint_violation=max(z['constraint_violation'] for z in logs),
        solve_time_median_s=float(np.median(times)),solve_time_p95_s=float(np.quantile(times,.95)),
        wall_seconds=time.perf_counter()-start)
    return dict(time=t,state=xx,command=commands,applied=applied),metrics,logs
