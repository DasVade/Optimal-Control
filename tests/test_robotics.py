import numpy as np
from control_portfolio import quadrotor,minsnap,ilqr,acrobot


def finite_jacobian(f,x,h=1e-5):
    return np.column_stack([(f(x+h*e)-f(x-h*e))/(2*h) for e in np.eye(len(x))])


def test_quad_hover_and_linearization():
    x=np.zeros(6);u=np.full(2,4.905);a,b=quadrotor.linearize()
    np.testing.assert_allclose(quadrotor.dynamics(x,u),0,atol=1e-12)
    np.testing.assert_allclose(a,finite_jacobian(lambda z:quadrotor.dynamics(z,u),x),atol=1e-8)
    np.testing.assert_allclose(b,finite_jacobian(lambda z:quadrotor.dynamics(x,z),u),atol=1e-8)


def test_mpc_hover_bounds_and_fallback():
    c=quadrotor.Controller();u,log=c.command(np.zeros(6))
    np.testing.assert_allclose(u,4.905,atol=1e-6);assert not log['fallback']
    u,log=c.command(np.array([.5,.5,0,1,1,0]))
    assert u.min()>=-1e-5 and u.max()<=5.5+1e-5 and log['constraint_violation']<1e-5
    c=quadrotor.Controller(max_iter=1);_,log=c.command(np.ones(6))
    assert log['fallback'] and log['status']!='solved'


def test_rk4_uses_evolving_state():
    # A callback frozen at the initial state fails this constant-acceleration check.
    x=np.array([0.,1.]);result=quadrotor.rk4(lambda x,u:np.array([x[1],u[0]]),x,[2.],.1)
    np.testing.assert_allclose(result,[.11,1.2],atol=1e-12)


def test_snap_constraints_and_duration_scaling():
    points=[[-3,-4],[0,0],[2,3],[4,0],[8,-4]]
    a=minsnap.solve(points,[1]*4);b=minsnap.solve(points,[2]*4)
    assert a['metrics']['waypoint_residual']<1e-8 and a['metrics']['continuity_residual']<1e-7
    np.testing.assert_allclose(b['metrics']['snap_integral']/a['metrics']['snap_integral'],1/128,rtol=1e-6)
    for t in np.linspace(0,4,11):
        np.testing.assert_allclose(minsnap.evaluate(a,t),minsnap.evaluate(b,2*t),atol=1e-6)


def test_pendulum_hover_and_discrete_derivatives():
    x=np.array([.1,.2,.05,.4,.1,-.2,.15,-.3]);u=np.array([5.8,6.])
    a,b=ilqr.jacobians(x,u,.02)
    np.testing.assert_allclose(a,finite_jacobian(lambda z:ilqr.step(z,u,.02),x),atol=1e-7)
    np.testing.assert_allclose(b,finite_jacobian(lambda z:ilqr.step(x,z,.02),u),atol=1e-7)
    for angle in [0.,np.pi]:
        equilibrium=np.zeros(8);equilibrium[3]=angle
        np.testing.assert_allclose(ilqr.dynamics(equilibrium,np.full(2,1.2*9.81/2)),0,atol=1e-12)


def test_acrobot_energy_and_equilibrium():
    x=np.array([.3,-.7,.4,-.2])
    def energy(z):
        q0,q1,v0,v1=z
        kinetic=.5*((6.16+5*np.cos(q1))*v0*v0+2*(2.58+2.5*np.cos(q1))*v0*v1+2.58*v1*v1)
        return kinetic-9.81*(3.5*np.cos(q0)+2.5*np.cos(q0+q1))
    grad=finite_jacobian(lambda z:np.array([energy(z)]),x).ravel()
    for u in [0.,3.]:np.testing.assert_allclose(grad@acrobot.dynamics(x,u),u*x[3],atol=1e-7)
    np.testing.assert_allclose(acrobot.dynamics(np.zeros(4),0),0,atol=1e-12)
    assert acrobot.clearance(np.zeros((1,4)),[[-1.5,0],[1.5,0]]).min()>0


def test_collocation_constant_equilibrium():
    assert np.max(np.abs(acrobot.defects(np.zeros((5,4)),np.zeros(5),.1)))==0


def test_osc_static_contact_force_signs():
    from control_portfolio.osc import solve_qp
    # Point mass at rest on ground: positive normal force must balance gravity.
    j=np.array([[0.],[0.],[1.]])
    solution,log=solve_qp(np.array([[2.]]),np.zeros(1),np.array([19.62]),
        np.zeros((1,1)),j,np.zeros(3),[],[10.])
    assert log['success'] and log['constraint_violation']<1e-6
    np.testing.assert_allclose(solution[-1],19.62,atol=1e-6)
