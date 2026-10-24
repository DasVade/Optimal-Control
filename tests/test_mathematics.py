import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import eigvals
from control_portfolio.cartpole import Controller,Parameters,dynamics,linearization,wrap
from control_portfolio.unicycle import Obstacle,plan,inputs
from control_portfolio.simulation import simulate


def test_linearization_matches_nonlinear_finite_differences():
    p=Parameters(mc=1.3,mp=.7,length=.8);x=np.array([0.,np.pi,0,0]);eps=1e-6
    a,b=linearization(p)
    numerical=np.column_stack([(dynamics(x+eps*v,0,p)-dynamics(x-eps*v,0,p))/(2*eps) for v in np.eye(4)])
    np.testing.assert_allclose(a,numerical,atol=1e-7)
    np.testing.assert_allclose(b[:,0],(dynamics(x,eps,p)-dynamics(x,-eps,p))/(2*eps),atol=1e-7)
    assert linearization(p,legacy=True)[0][2,2]==1 and a[2,2]==0


def test_care_residual_and_closed_loop_stability():
    c=Controller();a,b=linearization();s=c.riccati
    np.testing.assert_allclose(a.T@s+s@a-s@b@b.T@s/c.r+c.q,0,atol=1e-9)
    assert np.max(eigvals(a-b@c.k).real)<0


def test_partial_feedback_linearization_enforces_requested_acceleration():
    p=Parameters();state=np.array([.2,.6,.3,-.8]);desired=2.
    theta_dd=-np.cos(state[1])*desired-p.gravity*np.sin(state[1])
    force=2*desired-np.sin(state[1])*state[3]**2+np.cos(state[1])*theta_dd
    np.testing.assert_allclose(dynamics(state,force,p)[2],desired,atol=1e-12)


def test_angles_and_actuator_limits_apply_to_both_modes():
    c=Controller(force_limit=2.)
    for x in [[1,np.pi+.1,1,0],[1,.5,1,1]]:
        assert abs(c.command(np.array(x))[0])<=2.
        shifted=np.array(x)+np.array([0,2*np.pi,0,0])
        np.testing.assert_allclose(c.command(np.array(x))[0],c.command(shifted)[0],atol=1e-10)


def test_unicycle_flatness_matches_spline_velocity():
    reference=plan(Obstacle(2.,5.,1.))
    for t in np.linspace(0,10,21):
        omega,speed=inputs(t,reference);dy,dz=[s(t,1) for s in reference];theta=np.arctan2(dz,dy)
        np.testing.assert_allclose([speed*np.cos(theta),speed*np.sin(theta)],[dy,dz],atol=1e-12)
        assert dy>0 and np.isfinite(omega)


def test_rk4_agrees_with_tight_adaptive_solver_near_equilibrium():
    c=Controller(mode='lqr');x0=[.1,np.pi+.1,0,0]
    run=simulate(c,x0,2.,.01)
    reference=solve_ivp(lambda t,x:dynamics(x,c.command(x)[0]),[0,2],x0,rtol=1e-10,atol=1e-12)
    np.testing.assert_allclose(run['state'][-1],reference.y[:,-1],atol=1e-6)
