import numpy as np
from math import *
from grid_world import *

import matplotlib.pyplot as plt

def plot_value_function_and_optimal_policy(world, V, u_opt):
  plt.clf()
  v_plot = plt.imshow(V, interpolation='nearest')
  colorbar = plt.colorbar()
  colorbar.set_label("Value function")
  plt.xlabel("Column")
  plt.ylabel("Row")
  arrow_length = 0.25
  for row in range(world.rows):
    for col in range(world.cols):
      if u_opt[row, col] == 0: #N
        plt.arrow(col, row, 0, -arrow_length, head_width=0.1)
      elif u_opt[row, col] == 1: #E
        plt.arrow(col, row, arrow_length, 0, head_width=0.1)
      elif u_opt[row, col] == 2: #S
        plt.arrow(col, row, 0, arrow_length, head_width=0.1)
      elif u_opt[row, col] == 3: #W
        plt.arrow(col, row, -arrow_length, 0, head_width=0.1)
      else:
        raise ValueError("Invalid action")
  plt.savefig('value_function.png', dpi=240)
  plt.show()

def value_iteration(world, threshold, gamma, plotting=True):
  V = np.zeros((world.rows, world.cols))
  u_opt = np.zeros((world.rows, world.cols))
  grid_x, grid_y = np.meshgrid(np.arange(0, world.rows, 1), 
                               np.arange(0, world.cols, 1))
  delta = 10.0

  fig = plt.figure("Gridworld")
  
  # STUDENT CODE: calculate V and the optimal policy u using value iteration
  actions = [0, 1, 2, 3]  # N=0, E=1, S=2, W=3
  num_iters = 0

  while True:
    num_iters += 1
    delta = 0.0
    V_new = V.copy()

    for row in range(world.rows):
      for col in range(world.cols):
        s = world.map_row_col_to_state(row, col)

        best_q = float('inf')
        best_a = 0
        for a in actions:
          q_sa = 0.0
          for (p, s2, g) in world.P[s][a]:
            if p == 0.0:
              continue
            r2, c2 = world.map_state_to_row_col(s2)
            q_sa += p * (g + gamma * V[r2, c2])

          # choose action with minimum q, tie-breaking by action order
          if (q_sa < best_q - 1e-12) or (abs(q_sa - best_q) <= 1e-12 and a < best_a):
            best_q, best_a = q_sa, a

        V_new[row, col] = best_q
        u_opt[row, col] = best_a
        delta = max(delta, abs(V_new[row, col] - V[row, col]))

    V = V_new
    if delta < threshold:
      break

  print(f"Value iteration converged in {num_iters} iterations")

  if plotting:
    plot_value_function_and_optimal_policy(world, V, u_opt)


  return V, u_opt

if __name__=="__main__":
  world = GridWorld() 
  threshold = 0.0001
  gamma = 0.9
  # value_iteration(world, threshold, gamma, False)
  value_iteration(world, threshold, gamma, True)
