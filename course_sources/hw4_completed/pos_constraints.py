import numpy as np

def Ab_i1(i, n, d, dt_i, w_i, w_ip1):
  '''
  Ab_i1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
  constants that require the ith polynomial to meet waypoints w_i and w_{i+1}
  at it's endpoints.
  Parameters:
     i - index of the polynomial.
     n - total number of polynomials.
     d - the number of terms in each polynomial.
     dt_i - Delta t_i, duration of the ith polynomial.
     w_i - waypoint at the start of the ith polynomial.
     w_ip1 - w_{i+1}, waypoint at the end of the ith polynomial.
  Outputs
     A_i1 - A matrix from linear equality constraint A_i1 v = b_i1
     b_i1 - b vector from linear equality constraint A_i1 v = b_i1
  '''

  A_i1 = np.zeros((4, 2*d*n))
  b_i1 = np.zeros((4, 1))

  # TODO: fill in values for A_i1 and b_i1
  def idx(seg, j, dim):
    # dim=0 (y), dim=1 (z)
    return (seg * d + j) * 2 + dim

  # y(0) = w_i[0], z(0) = w_i[1]
  A_i1[0, idx(i, 0, 0)] = 1.0
  b_i1[0] = float(w_i[0])

  A_i1[1, idx(i, 0, 1)] = 1.0
  b_i1[1] = float(w_i[1])

  # y(Δt_i) = Σ_{j=0}^{d-1} sigma_{i,j,y} * (Δt_i)^j
  for j in range(d):
    A_i1[2, idx(i, j, 0)] = (dt_i ** j)
  b_i1[2] = float(w_ip1[0])

  # z(Δt_i) = Σ_{j=0}^{d-1} sigma_{i,j,z} * (Δt_i)^j
  for j in range(d):
    A_i1[3, idx(i, j, 1)] = (dt_i ** j)
  b_i1[3] = float(w_ip1[1])

  return A_i1, b_i1