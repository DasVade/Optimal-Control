#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase portrait for Problem 5(c).

System:
    x1' = x2 - 2
    x2' = -x1 - x2 + (1/3)*(x1 + 1)^3 + 1

Generates a quiver plot centered at the stable equilibrium (-1, 2)
and marks all equilibria (stable: solid; others: open).
"""

import numpy as np
import matplotlib.pyplot as plt


def f(x1, x2):
    """Vector field f(x) = (dx1, dx2)."""
    dx1 = x2 - 2.0
    dx2 = -x1 - x2 + (1.0/3.0)*(x1 + 1.0)**3 + 1.0
    return dx1, dx2


def equilibria_points():
    """
    Solve equilibria analytically:
        From dx1 = 0 -> x2 = 2
        Plug into dx2 = 0:
            -x1 - 2 + (1/3)(x1+1)^3 + 1 = 0
        Let y = x1 + 1 -> (1/3) y^3 - y - 1 = 0 -> y in {0, ±sqrt(3)}
        => x1 in {-1, -1±sqrt(3)}, with x2=2.
    """
    return [
        (-1.0, 2.0),                     # y = 0  (stable)
        (-1.0 + np.sqrt(3.0), 2.0),      # y = +sqrt(3)
        (-1.0 - np.sqrt(3.0), 2.0),      # y = -sqrt(3)
    ]


def plot_phase_portrait(center=(-1.0, 2.0), span=8.0, density=25, normalize=True):
    """
    Draw the quiver plot around `center` with square window of width/height `span`.
    """
    cx, cy = center
    half = span / 2.0
    x1_min, x1_max = cx - half, cx + half
    x2_min, x2_max = cy - half, cy + half

    x1 = np.linspace(x1_min, x1_max, density)
    x2 = np.linspace(x2_min, x2_max, density)
    X1, X2 = np.meshgrid(x1, x2)

    U, V = f(X1, X2)

    if normalize:
        speed = np.sqrt(U**2 + V**2)
        eps = 1e-12
        U = U / (speed + eps)
        V = V / (speed + eps)

    fig = plt.figure(figsize=(7, 7))
    plt.quiver(X1, X2, U, V, pivot='mid', scale=35)

    # Mark equilibria
    eqs = equilibria_points()
    stable = (-1.0, 2.0)
    for eq in eqs:
        if eq == stable:
            plt.plot(eq[0], eq[1], 'o', markersize=8)  # solid for stable
        else:
            plt.plot(eq[0], eq[1], 'o', markersize=8, fillstyle='none')  # open for saddles/others

    plt.xlim(x1_min, x1_max)
    plt.ylim(x2_min, x2_max)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("Phase Portrait for Problem 5(c): centered at stable equilibrium (-1, 2)")
    plt.xlabel("$x_1$")
    plt.ylabel("$x_2$")
    plt.grid(True, linestyle=':', linewidth=0.5)

    plt.tight_layout()
    plt.show()
    return fig


if __name__ == "__main__":
    plot_phase_portrait(center=(-1.0, 2.0), span=8.0, density=25, normalize=True)
