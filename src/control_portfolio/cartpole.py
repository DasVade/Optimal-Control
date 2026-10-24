"""Continuous-time cart-pole, downward-zero angle, and audited hybrid control."""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class Parameters:
    mc: float = 1.
    mp: float = 1.
    length: float = 1.
    gravity: float = 9.81


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def dynamics(state, force, p=Parameters()):
    """State [cart m, pole rad, cart m/s, pole rad/s]; force in N."""
    _, theta, velocity, omega = state
    c, s = np.cos(theta), np.sin(theta)
    a, b, d = p.mc+p.mp, p.mp*p.length*c, p.mp*p.length**2
    rhs1 = force+p.mp*p.length*s*omega**2
    rhs2 = -p.mp*p.gravity*p.length*s
    determinant = a*d-b*b
    return np.array([velocity, omega, (d*rhs1-b*rhs2)/determinant,
                     (-b*rhs1+a*rhs2)/determinant])


def linearization(p=Parameters(), legacy=False):
    """Continuous Jacobian about [0,pi,0,0], u=0; no discretization."""
    a = np.zeros((4,4));a[0,2] = a[1,3] = 1
    a[2,1] = p.mp*p.gravity/p.mc
    a[3,1] = (p.mc+p.mp)*p.gravity/(p.mc*p.length)
    if legacy:
        a[2,1] = p.gravity*p.mc/p.mp
        a[2,2] = 1.  # Reproduce the erroneous coefficient in original cartpole.py.
    b = np.array([[0.],[0.],[1/p.mc],[1/(p.mc*p.length)]])
    return a, b


class Controller:
    def __init__(self, p=Parameters(), q=(1,1,1,10), r=1., force_limit=180.,
                 legacy=False, mode='hybrid'):
        self.p, self.force_limit, self.legacy, self.mode = p, force_limit, legacy, mode
        self.q, self.r = np.diag(q), float(r)
        a, b = linearization(p, legacy)
        self.riccati = solve_continuous_are(a,b,self.q,np.array([[r]]))
        self.k = (b.T@self.riccati)/r
        self.closed_loop_eigenvalues = np.linalg.eigvals(a-b@self.k)

    def command(self, state):
        x, theta, velocity, omega = state
        p = self.p
        angle_error = theta-np.pi if self.legacy else wrap(theta-np.pi)
        energy_error = .5*omega**2-p.gravity/p.length*np.cos(theta)-p.gravity/p.length
        use_lqr = self.mode == 'lqr' or (abs(energy_error)<1. and angle_error**2+omega**2<1.)
        if use_lqr:
            raw = float(-(self.k@np.array([x,angle_error,velocity,omega])).item())
        else:
            desired_acceleration = np.clip(2.*omega*np.cos(theta)*energy_error-1.2*velocity-2.5*x,-28.,28.)
            theta_acceleration = -np.cos(theta)/p.length*desired_acceleration-p.gravity/p.length*np.sin(theta)
            raw = ((p.mc+p.mp)*desired_acceleration-p.mp*p.length*np.sin(theta)*omega**2
                   +p.mp*p.length*np.cos(theta)*theta_acceleration)
        # The original code clips only energy shaping. Audited code applies the same
        # explicit actuator clipping in both modes; this is NOT constrained LQR/MPC.
        applied = raw if self.legacy and use_lqr else float(np.clip(raw,-self.force_limit,self.force_limit))
        return applied, raw, 'lqr' if use_lqr else 'energy'
