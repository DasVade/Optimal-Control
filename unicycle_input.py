import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def unicycle_input(t, y_spline, z_spline):
  #UNICYCLE_INPUT returns input to the unicycle
  #   @param t - current time
  #   @param y_spline - spline object for desired y trajectory
  #   @param z_spline - spline object for desired z trajectory
  #   
  #   @output u - input u(t) to the unicycle system

  # TODO: modify u to return the correct input for time t.
  # Evaluate derivatives of y and z from the spline objects
  dy   = float(y_spline(t, 1))   # ydot
  ddy  = float(y_spline(t, 2))   # yddot
  dz   = float(z_spline(t, 1))   # zdot
  ddz  = float(z_spline(t, 2))   # zddot

  # Heading angle (not returned here, but used in simulation)
  theta = np.arctan2(dz, dy)

  # Flatness-based inversion for inputs
  u2 = np.hypot(dy, dz)                        # forward speed
  eps = 1e-9                                   # numerical safeguard
  denom = dy * dy + dz * dz + eps
  u1 = (dy * ddz - dz * ddy) / denom           # angular velocity = theta_dot

  # Assemble input vector u = [u1, u2]
  u = np.array([u1, u2], dtype=float)


  return u
