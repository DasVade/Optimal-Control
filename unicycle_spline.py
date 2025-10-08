import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from random import uniform

def unicycle_spline(t0, tf, obs):
  #UNICYCLE_SPLINE returns a spline object representing a path from
  # (y(t0),z(t0)) = (0,0) to (y(t0),z(t0)) = (10,0) that avoids a circular
  # obstacle, such that d\dt y(t) > 0
  #   @param t0 - initial time
  #   @param tf - final time
  #
  #   @output y_spline - spline object for desired y trajectory
  #   @output z_spline - spline object for desired z trajectory
  y0 = 0;
  z0 = 0;

  yf = 10;
  zf = 0;

  # TODO: design the spline here
   # Add a dense flat plateau in z across the obstacle band to prevent sagging
  R = float(obs.radius)
  margin = 1.0                                   # a bit larger for safety
  side = +1.0 if obs.z <= 0.0 else -1.0
  h = side * (R + margin)

  # Define obstacle band in y (clamped away from endpoints)
  epsy = 0.25
  y0f_lo = y0 + epsy
  y0f_hi = yf - epsy
  yc = float(np.clip(obs.y, y0f_lo, y0f_hi))
  band = R + margin
  y1 = max(y0f_lo, yc - band)
  y2 = min(y0f_hi, yc + band)

  # Map y-positions to times since y(t) will be linear with fixed slope
  slope = (yf - y0) / (tf - t0)
  def t_of_y(yy): return t0 + (yy - y0) / slope

  # y(t): strictly increasing linear spline (dy/dt = slope > 0)
  y_spline = CubicSpline([t0, tf], [y0, yf],
                         bc_type=((1, slope), (1, slope)))

  # z(t): many knots forming a flat plateau h across [y1, y2]
  # start/end + N equally spaced plateau knots
  N = 15                                           # more knots => less sag
  y_plateau = np.linspace(y1, y2, N)
  t_plateau = t_of_y(y_plateau)

  t_pts = np.concatenate(([t0], t_plateau, [tf])).astype(float)
  z_pts = np.concatenate(([z0], np.full_like(t_plateau, h), [zf])).astype(float)

  z_spline = CubicSpline(t_pts, z_pts, bc_type='natural')

  return y_spline, z_spline