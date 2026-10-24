import numpy as np
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp

from trajectories import *

class Quadrotor(object):
  '''
  Constructor. Compute function S(t) using S(t) = L(t) L(t)^t, by integrating backwards
  from S(tf) = Qf. We will then use S(t) to compute the optimal controller efforts in 
  the compute_feedback() function
  '''
  def __init__(self, Q, R, Qf, tf):
    self.m = 1
    self.a = 0.25
    self.I = 0.0625
    self.Q = Q
    self.R = R

    ''' 
    We are integrating backwards from Qf
    '''

    # Get L(tf) L(tf).T = S(tf) by decomposing S(tf) using Cholesky decomposition
    L0 = cholesky(Qf).transpose()

    # We need to reshape L0 from a square matrix into a row vector to pass into solve_ivp()
    l0 = np.reshape(L0, (36))
    # L must be integrated backwards, so we integrate L(tf - t) from 0 to tf
    initial_condition = [0, tf]
    sol = solve_ivp(self.dldt_minus, [0, tf], l0, dense_output=True)
    t = sol.t
    l = sol.y

    # Reverse time to get L(t) back in forwards time
    t = tf - t
    t = np.flip(t)
    l = np.flip(l, axis=1) # flip in time
    self.l_spline = interp1d(t, l)

  def Ldot(self, t, L):

    x = x_d(t)
    u = u_d(t)
    Q = self.Q
    R = self.R

    dLdt = np.zeros((6,6))
    # STUDENT CODE: compute d/dt L(t)

    # Build A(t), B(t) from the quadrotor model (part 2a)
    theta_d = x[2]
    U_d = u[0] + u[1]
    m = self.m
    a = self.a
    I = self.I

    # partial(qddot)/partial q at (q_d, u_d)
    dqdq = np.array([
        [0.0, 0.0, -(np.cos(theta_d)/m) * U_d],
        [0.0, 0.0, -(np.sin(theta_d)/m) * U_d],
        [0.0, 0.0, 0.0]
    ])

    # partial(qddot)/partial u at (q_d, u_d)
    dqdu = np.array([
        [-(np.sin(theta_d)/m), -(np.sin(theta_d)/m)],
        [ (np.cos(theta_d)/m),  (np.cos(theta_d)/m)],
        [ (a/I),               -(a/I)]
    ])

    A = np.zeros((6,6))
    A[0:3,3:6] = np.eye(3)     # dq/dt = qdot
    A[3:6,0:3] = dqdq          # qddot depends on q

    B = np.zeros((6,2))
    B[3:6,0:2] = dqdu

    # Square-root Riccati: dL/dt = -1/2 Q L^{-T} - A^T L + 1/2 (L L^T) B R^{-1} B^T L
    # Compute S = L L^T
    S = L @ L.T

    # Compute Q @ L^{-T} without explicit inverse:
    # Solve (L^T) X = Q^T  ->  X = (L^T)^{-1} Q^T, then (Q L^{-T}) = X^T
    X = np.linalg.solve(L.T, Q.T)
    QLinvT = X.T

    # Compute M = B R^{-1} B^T without explicit inverse:
    # Solve R Y = B^T  ->  Y = R^{-1} B^T, then M = B Y
    Y = np.linalg.solve(R, B.T)   # (2x6)
    M = B @ Y                     # (6x6)

    dLdt = -0.5 * QLinvT - A.T @ L + 0.5 * (S @ M @ L)

    return dLdt

  def dldt_minus(self, t, l):
    # reshape l to a square matrix
    L = np.reshape(l, (6, 6))

    # compute Ldot
    dLdt_minus = -self.Ldot(t, L)

    # reshape back into a vector
    dldt_minus = np.reshape(dLdt_minus, (36))
    return dldt_minus


  def compute_feedback(self, t, x):

    # Retrieve L(t)
    L = np.reshape(self.l_spline(t), (6, 6))

    u_fb = np.zeros((2,))
    # STUDENT CODE: Compute optimal feedback inputs u_fb using LQR

    # S = L L^T
    S = L @ L.T

    # Error state
    xe = x - x_d(t)

    # Build B(t) at time t (only B is needed here)
    xd = x_d(t)
    ud = u_d(t)
    theta_d = xd[2]
    m = self.m
    a = self.a
    I = self.I

    dqdu = np.array([
        [-(np.sin(theta_d)/m), -(np.sin(theta_d)/m)],
        [ (np.cos(theta_d)/m),  (np.cos(theta_d)/m)],
        [ (a/I),               -(a/I)]
    ])
    B = np.zeros((6,2))
    B[3:6,0:2] = dqdu

    # K = R^{-1} B^T S (avoid explicit inverse by solving R * Z = B^T S)
    BTS = B.T @ S                 # (2x6)
    K = np.linalg.solve(self.R, BTS)

    # u_fb = -K x_e
    u_fb = - K @ xe

    # Add u_fb to u_d(t), the feedforward term. 
    # u = u_fb + u_d
    u = u_d(t) + u_fb;
    return u