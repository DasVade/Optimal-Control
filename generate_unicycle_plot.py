import matplotlib.pyplot as plt
from random import uniform

class Obstacle:
  radius = uniform(1, 3)
  y = uniform(0.5 + radius, 9.5 - radius)
  z = uniform(-radius, radius)

def main():
  obs = Obstacle()
  fig = plt.figure(figsize=(8,6))
  ax = plt.axes()
  ax.scatter(0, 0, c='b', label='Initial position')
  ax.scatter(10, 0, c='g', label='Target position')
  a_circle = plt.Circle((obs.y, obs.z), obs.radius, color='r', label='Obstacle')
  ax.add_artist(a_circle)
  ax.set_xlim(-2, 12)
  ax.set_ylim(-5,5)
  ax.set_xlabel('y (m)')
  ax.set_ylabel('z (m)')
  ax.set_aspect('equal')
  ax.legend(loc='upper right')
  plt.show()

if __name__ == '__main__':
  main()