import numpy as np
from math import sin, cos, pi

from pydrake.systems.controllers import LinearQuadraticRegulator



class Cartpole(object):
  '''

  '''
  def __init__(self):
    # k_p and k_d gains for swing up controller
    
    # TODO: choose appropriate k_p and k_d gains
    self.kp = 2.5
    self.kd = 1.2
    self.g = 9.81

    # mass of the pole and mass of the part
    self.mc = 1
    self.mp = 1
    self.L = 1

    # robust but not over-aggressive limits (baseline that gave 4/5)
    self.ddq1_max = 28.0   # was 24.0; slight bump to help the last tough case
    self.u_max    = 180.0  # actuator effort cap
    self.E_gain   = 2.0    # energy injection scale

    # Computes the lqr gains for the final stabilizing controller
    A = np.array([[0,                  0,                                1, 0],
                  [0,                  0,                                0, 1],
                  [0, self.g * (self.mc / self.mp),                      1, 0],
                  [0, self.g * (self.mc + self.mp) / (self.L * self.mc), 0, 0]])
    B = np.array([0, 0, 1/self.mc, 1/(self.L * self.mc)]).T
    Q = np.eye((4))
    Q[3, 3] = 10
    R = np.array([1])
    N = np.zeros(4)

    self.K, self.s = LinearQuadraticRegulator(A, B, Q, R, N) 
    self.x_des = np.array([0, pi, 0, 0])


  def compute_efforts(self, t, x):
    q = x[:2]
    qdot = x[-2:]

    e_tilde = 0.5 * qdot[1] ** 2 - self.g * cos(q[1]) - self.g
    angular_distance  = x[3]**2 + (x[1]-pi)**2

    if np.abs(e_tilde) < 1 and angular_distance < 1:
      return self.compute_lqr_input(x)
    else:
      return self.compute_energy_shaping_input(t, x)


  def compute_energy_shaping_input(self, t, x):
    '''
    Computes the energy shaping inputs to stabilize the cartpole
    '''
    q = x[:2]
    qdot = x[-2:]
    u = 0
    q1_ddot_des = 0
    q1_ddot_pd_term = - self.kd * qdot[0] - self.kp * q[0]

    # cache trig
    c2 = cos(q[1]); s2 = sin(q[1])

    # TODO: compute q1_ddot_des 
    # Energy shaping term: q1_ddot_des_energy = dq2 * cos(q2) * (E - E*)
    # where E = 0.5*dq2^2 - g*cos(q2), E* = g (upright)
    E_tilde = 0.5 * (qdot[1] ** 2) - self.g * c2 - self.g
    q1_ddot_energy = (qdot[1] * c2) * (self.E_gain * E_tilde)

    # Direct PD addition (no state-dependent weighting)
    q1_ddot_des = q1_ddot_energy + q1_ddot_pd_term

    # hard clip desired acceleration (no tanh softening)
    q1_ddot_des = float(np.clip(q1_ddot_des, -self.ddq1_max, self.ddq1_max))

    # TODO: compute the optimal input u according to q1_ddot_des and partial feedback linearization
    # From the second equation:
    #   q2_ddot = -(cos q2 / L) * q1_ddot_des - (g/L) * sin q2
    # And the input enforcing q1_ddot = q1_ddot_des:
    #   u = (mc+mp) * q1_ddot_des - mp*L*sin q2 * dq2^2 + mp*L*cos q2 * q2_ddot
    q2_ddot = -(c2 / self.L) * q1_ddot_des - (self.g / self.L) * s2
    u = (self.mc + self.mp) * q1_ddot_des \
        - self.mp * self.L * s2 * (qdot[1] ** 2) \
        + self.mp * self.L * c2 * q2_ddot

    # actuator saturation
    u = float(np.clip(u, -self.u_max, self.u_max))
    return u

  def compute_lqr_input(self, x):
    '''
    Stabilizes the cartpole at the final location using lqr
    '''
    return -self.K @ (x - self.x_des)