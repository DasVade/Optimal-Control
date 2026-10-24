from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation


class Quadrotor2DVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(8,6))
        self.length = 0.25  # moment arm (meters)

    def draw(self, x, t, x_des, x_anim, x_axis_min, x_axis_max, y_axis_min, y_axis_max):
        self.ax.clear()

        # draw the quadrotor
        self.base = np.vstack(
            (
                1.2 * self.length * np.array([1, -1, -1, 1, 1]),
                0.025 * np.array([1, 1, -1, -1, 1]),
            )
        )
        self.pin = np.vstack(
            (
                0.005 * np.array([1, 1, -1, -1, 1]),
                0.1 * np.array([1, 0, 0, 1, 1]),
            )
        )
        a = np.linspace(0, 2 * np.pi, 50)
        self.prop = np.vstack(
            (self.length / 1.5 * np.cos(a), 0.1 + 0.02 * np.sin(2 * a))
        )
        self.base_fill = self.ax.fill(
            self.base[0, :], self.base[1, :], zorder=1, edgecolor="k",
            facecolor=[.6, .6, .6])
        self.left_pin_fill = self.ax.fill(
            self.pin[0, :], self.pin[1, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 0])
        self.right_pin_fill = self.ax.fill(
            self.pin[0, :], self.pin[1, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 0])
        self.left_prop_fill = self.ax.fill(
            self.prop[0, :], self.prop[0, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 1])
        self.right_prop_fill = self.ax.fill(
            self.prop[0, :], self.prop[0, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 1])
        R = np.array([[np.cos(x[2]), -np.sin(x[2])], [np.sin(x[2]), np.cos(x[2])]])

        p = np.dot(R, self.base)
        self.base_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.base_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R, np.vstack((-self.length + self.pin[0, :], self.pin[1, :])))
        self.left_pin_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.left_pin_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]
        p = np.dot(R, np.vstack((self.length + self.pin[0, :], self.pin[1, :])))
        self.right_pin_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.right_pin_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R, np.vstack((-self.length + self.prop[0, :], self.prop[1, :])))
        self.left_prop_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.left_prop_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R, np.vstack((self.length + self.prop[0, :], self.prop[1, :])))
        self.right_prop_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.right_prop_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        # draw trajectory and desired position
        self.ax.plot(x_des[0], x_des[1], 'b*', label='desired position')
        self.ax.plot(x_anim[:, 0], x_anim[:, 1], '--', label='actual trajectory')

        x_padding = 0.5 * (x_axis_max - x_axis_min)
        y_padding = 0.5 * (y_axis_max - y_axis_min)

        # set plot x-y limits
        if(np.abs((x_axis_max - x_axis_min) - (y_axis_max - y_axis_min)) < 5):
            self.ax.set_xlim(x_axis_min - x_padding, x_axis_max + x_padding)
            self.ax.set_ylim(y_axis_min - y_padding, y_axis_max + y_padding)
        self.ax.set_xlabel('y (m)')
        self.ax.set_ylabel('z (m)')
        self.ax.set_aspect('equal')
        self.ax.legend(loc='upper left')
        self.ax.set_title("t = {:.1f}".format(t))

def x_d(t):
  return np.zeros(6)

def create_animation(x, tf, n_frames=240):
  n_samples = 1005
  t_samples = np.linspace(0.0, tf, n_samples)
  dt = tf / (n_frames - 1)
  x_des = np.zeros((n_samples, 6))
  for i in range(t_samples.shape[0]):
    x_des[i] = x_d(t_samples[i])

  frame_idx = [round(x) for x in np.linspace(0, x.shape[0]-1, n_frames).tolist()]
  x_anim = np.zeros((n_frames, 6))
  t_anim = np.zeros(n_frames)
  for i in range(n_frames):
    x_anim[i, :] = x[frame_idx[i], :]
    t_anim[i] = t_samples[frame_idx[i]]

  quad_vis = Quadrotor2DVisualizer()

  x_max = max(np.max(x_des[:, 0]), np.max(x[:, 0]))
  x_min = min(np.min(x_des[:, 0]), np.min(x[:, 0]))
  y_max = max(np.max(x_des[:, 1]), np.max(x[:, 1]))
  y_min = min(np.min(x_des[:, 1]), np.min(x[:, 1]))

  def frame(i):
    quad_vis.draw(x_anim[i], t_anim[i], x_des[0], x_anim[:i+1], x_min, x_max, y_min, y_max)

  return animation.FuncAnimation(quad_vis.fig, frame, n_frames, interval=dt*500)
