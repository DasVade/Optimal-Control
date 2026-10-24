import numpy as np
import matplotlib.pyplot as plt
from math import factorial, atan2
from scipy.interpolate import PPoly

from pydrake.all import (
    MathematicalProgram, 
    OsqpSolver
)

import importlib

import pos_constraints
importlib.reload(pos_constraints)
from pos_constraints import Ab_i1

def _flatten_sigma_vars(sigma, n, d):
  """Flatten sigma (n,d,2) -> 1D numpy array in order (i,j,dim) with dim last."""
  flat = []
  for i in range(n):
    for j in range(d):
      flat.append(sigma[i][j][0])  # y
      flat.append(sigma[i][j][1])  # z
  return np.array(flat)

def add_pos_constraints(prog, sigma, n, d, w, dt):
  # Add A_i1 constraints here
  var_flat = _flatten_sigma_vars(sigma, n, d)
  for i in range(n):
    Aeq_i, beq_i = Ab_i1(i, n, d, float(dt[i]), w[i], w[i + 1])
    prog.AddLinearEqualityConstraint(Aeq_i, beq_i, var_flat)

def add_continuity_constraints(prog, sigma, n, d, dt):
  # TDOO: Add A_i2 constraints here
  # Hint: Use AddLinearEqualityConstraint(expr, value)
  # gamma_i^{(k)}(Δt_i) == gamma_{i+1}^{(k)}(0), for k = 1..4 (if d>=5)
  max_k = min(4, d - 1)
  for i in range(n - 1):
    dt_i = float(dt[i])
    for k in range(1, max_k + 1):
      k_fac = factorial(k)
      for dim in range(2):  # 0: y, 1: z
        # lhs: sum_{j=k}^{d-1} falling(j,k) * sigma[i][j][dim] * (dt_i)^(j-k)
        lhs = 0.0
        for j in range(k, d):
          coeff = 1
          for t in range(k):
            coeff *= (j - t)  # falling(j,k)
          lhs += coeff * sigma[i][j][dim] * (dt_i ** (j - k))
        # rhs at t=0: k! * sigma[i+1][k][dim]
        rhs = k_fac * sigma[i + 1][k][dim]
        prog.AddLinearEqualityConstraint(lhs - rhs, 0.0)

  
def add_minsnap_cost(prog, sigma, n, d, dt):
  # TODO: Add cost function here
  # Use AddQuadraticCost to add a quadratic cost expression
 for i in range(n):
    dt_i = float(dt[i])
    # falling(j,4) for j>=4
    fall4 = np.zeros(d)
    for j in range(4, d):
      fall4[j] = j * (j - 1) * (j - 2) * (j - 3)

    # Build Q_i[j,l] = fall4(j)*fall4(l) * Δt_i^{j+l-7} / (j+l-7), for j,l >= 4
    Q = np.zeros((d, d))
    for j in range(4, d):
      for l in range(4, d):
        Q[j, l] = (fall4[j] * fall4[l]) * (dt_i ** (j + l - 7)) / (j + l - 7)

    # Add cost for each dimension (y,z): vars^T Q vars
    for dim in range(2):
      vars_dim = np.array([sigma[i][j][dim] for j in range(d)], dtype=object)
      expr = (vars_dim.T @ Q @ vars_dim)   #  pydrake.symbolic.Expression
      prog.AddQuadraticCost(expr)


def minsnap(n, d, w, dt):
  n_dim = 2
  dim_names = ['y', 'z']

  prog = MathematicalProgram()
  # sigma is a (n, n_dim, d) matrix of decision variables
  sigma = np.zeros((n, d, n_dim), dtype="object")
  for i in range(n):
    for j in range(d):
      sigma[i][j] = prog.NewContinuousVariables(n_dim, "sigma_" + str(i) + ',' +str(j)) 

  add_pos_constraints(prog, sigma, n, d, w, dt)
  add_continuity_constraints(prog, sigma, n, d, dt)
  add_minsnap_cost(prog, sigma, n, d, dt)  

  solver = OsqpSolver()
  result = solver.Solve(prog)
  print(result.get_solution_result())
  v = result.GetSolution()
  
  # Reconstruct the trajectory from the polynomial coefficients
  coeffs_y = v[::2]
  coeffs_z = v[1::2]
  y = np.reshape(coeffs_y, (d, n), order='F')
  z = np.reshape(coeffs_z, (d, n), order='F')
  coeff_matrix = np.stack((np.flip(y, 0), np.flip(z, 0)), axis=-1)  
  t0 = 0
  t = np.hstack((t0, np.cumsum(dt)))
  minsnap_trajectory = PPoly(coeff_matrix, t, extrapolate=False)

  return minsnap_trajectory

if __name__ == '__main__':

  n = 4
  d = 14

  w = np.zeros((n + 1, 2))
  dt = np.zeros(n)

  w[0] = np.array([-3,-4])
  w[1] = np.array([ 0, 0])
  w[2] = np.array([ 2, 3])
  w[3] = np.array([ 5, 0])
  w[4] = np.array([ 8, -2])

  dt[0] = 1
  dt[1] = 1
  dt[2] = 1
  dt[3] = 1

  # Target trajectory generation
  minsnap_trajectory = minsnap(n, d, w, dt)

  g = 9.81
  t0 = 0
  tf = sum(dt)
  n_points = 100
  t = np.linspace(t0, tf, n_points)

  fig = plt.figure(figsize=(4,3))
  ax = plt.axes()
  ax.scatter(w[:, 0], w[:, 1], c='r', label='way pts')
  ax.plot(minsnap_trajectory(t)[:,0], minsnap_trajectory(t)[:,1], label='min-snap trajectory')
  ax.set_xlabel("y")
  ax.set_ylabel("z")
  ax.legend()

  debugging = False
  # Set debugging to true to verify that the derivatives up to 5 are continuous
  if debugging:
    fig2 = plt.figure(figsize=(4,3))
    plt.plot(t, minsnap_trajectory(t,1)[:], label='1st derivative')
    plt.legend()

    fig3 = plt.figure(figsize=(4,3))
    plt.plot(t, minsnap_trajectory(t,2)[:], label='2nd derivative')
    plt.legend()

    fig4 = plt.figure(figsize=(4,3))
    plt.plot(t, minsnap_trajectory(t,3)[:], label='3rd derivative')
    plt.legend()

    fig5 = plt.figure(figsize=(4,3))
    plt.plot(t, minsnap_trajectory(t,4)[:], label='4th derivative')
    plt.legend()
    
  plt.show()  