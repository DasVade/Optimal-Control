"""Original spline heuristic and flatness inversion, without GUI/random import side effects."""
from dataclasses import dataclass
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class Obstacle:
    radius:float
    y:float
    z:float


def plan(obstacle, duration=10.):
    r=obstacle.radius;side=1. if obstacle.z<=0 else -1.
    center=float(np.clip(obstacle.y,.25,9.75))
    band=r+1.;knots=np.linspace(max(.25,center-band),min(9.75,center+band),15)
    forward=CubicSpline([0.,duration],[0.,10.],bc_type=((1,10/duration),(1,10/duration)))
    lateral=CubicSpline(np.r_[0.,knots*duration/10,duration],np.r_[0.,np.full(15,side*(r+1.)),0.],bc_type='natural')
    return forward,lateral


def inputs(t,reference):
    y,z=reference;dy,dz=y(t,1),z(t,1);ddy,ddz=y(t,2),z(t,2)
    # Preserve the original small denominator safeguard.
    return np.array([(dy*ddz-dz*ddy)/(dy**2+dz**2+1e-9),np.hypot(dy,dz)])


def experiment(seed=101,heading_offset=0.,speed_scale=1.,duration=10.,max_step=.005,rtol=1e-8,atol=1e-10):
    rng=np.random.default_rng(seed);radius=rng.uniform(2,4)
    obs=Obstacle(float(radius),float(rng.uniform(.5+radius,9.5-radius)),float(rng.uniform(-radius,radius)))
    reference=plan(obs,duration);times=np.linspace(0,duration,2001)
    theta=np.arctan2(reference[1](0,1),reference[0](0,1))+heading_offset
    def f(t,x):
        omega,speed=inputs(t,reference)
        return [speed_scale*speed*np.cos(x[2]),speed_scale*speed*np.sin(x[2]),omega]
    sol=solve_ivp(f,[0,duration],[0,0,theta],t_eval=times,max_step=max_step,rtol=rtol,atol=atol)
    desired=np.column_stack([s(times) for s in reference]);errors=sol.y[:2].T-desired
    clearance=np.linalg.norm(sol.y[:2].T-np.array([obs.y,obs.z]),axis=1)-obs.radius
    control=np.column_stack([inputs(t,reference) for t in times]).T
    return {'time':times,'state':sol.y.T,'desired':desired,'input':control,'obstacle':obs,
            'metrics':{'seed':seed,'heading_offset_rad':heading_offset,'speed_scale':speed_scale,
                       'position_rmse_m':float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
                       'final_position_error_m':float(np.linalg.norm(errors[-1])),
                       'sampled_min_clearance_m':float(clearance.min()),'sampled_collision_fraction':float(np.mean(clearance<0)),
                       'peak_speed_mps':float(control[:,1].max()),'peak_yaw_rate_radps':float(abs(control[:,0]).max()),
                       'final_reference_speed_mps':float(control[-1,1]),
                       'positive_forward_derivative':bool(np.all(reference[0](times,1)>0)),
                       'solver_success':bool(sol.success)}}
