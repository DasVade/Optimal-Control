import time
import numpy as np
from .cartpole import Parameters, dynamics, wrap


def simulate(controller, initial, duration=20., dt=.01, plant=Parameters(), disturbance=None):
    """Fixed-step RK4 with continuous state feedback evaluated at every RK stage."""
    steps = round(duration/dt)
    if not np.isclose(steps*dt,duration):raise ValueError('Duration must be an integer multiple of dt.')
    times = np.arange(steps+1)*dt
    states = np.full((steps+1,4),np.nan);states[0] = initial
    applied=np.full(steps+1,np.nan);raw=applied.copy();external=applied.copy();modes=[]
    disturbance = disturbance or (lambda t:0.)
    start=time.perf_counter();stopped=False
    def derivative(t,x):return dynamics(x,controller.command(x)[0]+disturbance(t),plant)
    for i,t in enumerate(times):
        applied[i],raw[i],mode=controller.command(states[i]);modes.append(mode);external[i]=disturbance(t)
        if i==steps:break
        x=states[i]
        k1=derivative(t,x);k2=derivative(t+dt/2,x+dt*k1/2)
        k3=derivative(t+dt/2,x+dt*k2/2);k4=derivative(t+dt,x+dt*k3)
        states[i+1]=x+dt*(k1+2*k2+2*k3+k4)/6
        if not np.isfinite(states[i+1]).all() or abs(states[i+1,0])>50 or np.linalg.norm(states[i+1,2:])>150:
            stopped=True;break
    return {'time':times,'state':states,'input':applied,'raw_input':raw,'disturbance':external,
            'mode':np.array(modes),'elapsed_seconds':time.perf_counter()-start,'stopped':stopped}


def metrics(run, limit):
    x=run['state'];t=run['time'];u=run['input'];raw=run['raw_input']
    if run['stopped']:
        return {'stopped':True,'settling_time_s':None,'elapsed_seconds':run['elapsed_seconds']}
    e=wrap(x[:,1]-np.pi)
    within=(abs(x[:,0])<=.05)&(abs(e)<=.05)&(abs(x[:,2])<=.1)&(abs(x[:,3])<=.1)
    remains=np.logical_and.accumulate(within[::-1])[::-1]
    valid=np.flatnonzero(remains & (t<=t[-1]-1.))
    return {'stopped':False,'cart_rmse_m':float(np.sqrt(np.mean(x[:,0]**2))),
            'angle_rmse_rad':float(np.sqrt(np.mean(e**2))), 'final_cart_error_m':float(abs(x[-1,0])),
            'final_angle_error_rad':float(abs(e[-1])),
            'settling_time_s':float(t[valid[0]]) if len(valid) else None,
            'control_effort_N2s':float(np.trapezoid(u*u,t)), 'peak_input_N':float(abs(u).max()),
            'requested_limit_exceedance_fraction':float(np.mean(abs(raw)>limit+1e-9)),
            'applied_limit_violation_fraction':float(np.mean(abs(u)>limit+1e-9)),
            'cart_bound_exceedance_fraction':float(np.mean(abs(x[:,0])>2.)),
            'clipping_fraction':float(np.mean(abs(raw-u)>1e-9)), 'mode_switches':int(np.sum(run['mode'][1:]!=run['mode'][:-1])),
            'elapsed_seconds':run['elapsed_seconds']}
