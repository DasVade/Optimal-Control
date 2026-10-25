# Cart-pole and unicycle implementation notes

The original HW3 repository at `3efbb4c` contains cart-pole energy shaping/LQR and
unicycle flatness-based feedforward planning. Original files and outputs are
recorded in [source_manifest.json](source_manifest.json). For the expanded
HW1–HW6 project, see the [implementation inventory](ASSIGNMENT_INVENTORY.md)
and [robotics report](ROBOTICS_REPORT.md).

## Source modules

| Module | Function |
|---|---|
| `unicycle_spline.py` | Interpolating cubic spline with obstacle-dependent lateral plateau |
| `unicycle_input.py` | Flatness inverse returning yaw rate and forward speed |
| `unicycle_sim.py`, `simulate_unicycle.py` | Nonlinear SciPy simulation entry points |
| `cartpole.py` | Energy shaping, cart recentering, partial feedback linearization, LQR and switching |
| `cartpole_sim.py` | Nonlinear mass-matrix dynamics and animation |
| `hw3.ipynb` | Coursework examples and stored outputs |

## Model corrections

The original LQR `A[2,2]=1` introduces cart-velocity dependence absent from the
nonlinear dynamics. `A[2,1]=g*mc/mp` also has the wrong mass ratio, hidden when
both masses equal one; the derivative is `g*mp/mc`. Finite differences check the
corrected linearization with unequal masses and nonunit pole length.

The Python package wraps upright-angle errors and applies force clipping in
both energy-shaping and LQR modes. This is actuator clipping of unconstrained
controllers. Normalized energy uses `g/L`; the original expression assumes
`L=1`. Nominal controller and true plant parameters are separate for mismatch
experiments, and obstacle draws use per-instance fixed seeds.

SciPy CARE provides the portable continuous-time LQR workflow. Legacy-A
comparisons use the original coefficients in that solver. The original
Drake-dependent notebooks are preserved separately. Dependencies include NumPy,
SciPy, Matplotlib, Pillow and pytest; the Drake modules use the persistent
environment described in [WSL_SETUP.md](WSL_SETUP.md).
