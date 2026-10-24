# Robotic Motion Planning and Optimal Control

Reproducible robotics simulations exploring actuator limits, prediction horizons, trajectory smoothness and nonlinear optimization failures. Built from Penn MEAM 5170 coursework, with audited models, new solver completions and controlled experiments. Python, NumPy, SciPy and OSQP; local CPU execution.

<img src="docs/figures/robotics/quadrotor_comparison.png" width="850" alt="Matched nonlinear quadrotor LQR and MPC simulations with horizon, error, pitch and effort comparisons">

## Methods

- **Planar quadrotor:** continuous LQR with actuator clipping versus input-constrained linear MPC on the same nonlinear plant; horizon, cost-weight, disturbance and mass-mismatch studies.
- **Minimum-snap planning:** waypoint interpolation with C4 continuity, a scaled numerical solver, constraint residuals and duration-scaling experiments.
- **Underactuated examples:** preserved cart-pole energy shaping/LQR and unicycle flatness planning; new quadrotor-pendulum iLQR and Acrobot collocation experiments with unsuccessful/physically infeasible outcomes explicitly reported.
- **Supporting coursework:** reproduced phase portraits and GridWorld value iteration. Biped walking remains blocked by the Drake environment; only the portable OSC matrix QP has been checked.

## Representative results

Quadrotor: initial `[0.5,0.5,0,1,1,0]`, 10 s nonlinear simulation, both motors limited to 0-5.5 N. MPC uses .1 s prediction spacing and .02 s control updates. N counts **states**; N=10 has nine input steps. All configurations and unsuccessful trials are retained in the [technical report](docs/ROBOTICS_REPORT.md).

| Method | Position RMSE (m) | Full-state settling (s) | Above-hover effort (N² s) |
|---|---:|---:|---:|
| Clipped LQR | 0.2873 | 2.68 | 3.8730 |
| MPC, N=5 | 0.7994 | not settled | 0.8692 |
| MPC, N=10 | 0.3501 | 3.26 | 2.4021 |
| MPC, N=20 | 0.3257 | 2.62 | 2.8863 |

MPC N=10 uses 38% less above-hover effort here, at higher position error and slower settling. Short horizons, aggressive clipped LQR and mass mismatch expose failures; this is not a universal robustness claim. Settling requires all position, angle and velocity errors to remain inside the report's explicit tolerances through the end of the run.

<img src="docs/figures/robotics/minimum_snap.png" width="850" alt="Minimum-snap waypoint paths and speed/snap profiles under duration scaling">

Doubling minimum-snap trajectory duration from 4 to 8 s reduces integrated squared snap from 210.6954 to 1.6461 (128×), as predicted by time scaling. Endpoint derivatives are free; dynamic flight feasibility is not imposed. [Simulation GIF](docs/figures/robotics/quadrotor.gif) · [Failure cases](docs/figures/robotics/quadrotor_failures.png) · [iLQR limitations](docs/figures/robotics/ilqr.png) · [Preserved cart-pole animation](docs/figures/swingup.gif)

## Install and run

From the repository root, using Python 3.12 (tested on Windows):

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
.\.venv\Scripts\python.exe -m control_portfolio.cli robotics-benchmark --output 'outputs/my_benchmark'
.\.venv\Scripts\python.exe -m control_portfolio.cli quadrotor --output 'outputs/my_quadrotor'
.\.venv\Scripts\python.exe -m control_portfolio.cli minsnap --output 'outputs/my_minsnap'
.\.venv\Scripts\python.exe -m pytest -q
```

Choose a fresh output folder for each run. JSON configs, numerical CSV/NPZ traces, solver logs and environment details are exported automatically. [Full reproduction instructions](docs/REPRODUCTION.md) include iLQR, Acrobot, supporting examples, the existing HW3 suite and rebuilding the figures.

## Coursework and contributions

Original sources remain traceable in Git history and [course_sources](course_sources/README.md). Existing completed work includes HW3 control/planning, HW2 GridWorld and an alternative HW4 minimum-snap implementation. This update adds the portable MPC, scaled minimum-snap solver, iLQR/collocation completions, verification and experiments; these are established course methods. Instructor frameworks/models and the supplied footstep planner are attributed separately in the [HW1-HW6 inventory](docs/ASSIGNMENT_INVENTORY.md).

Keep the repository private pending confirmation of course-material redistribution conditions. No hardware deployment, original algorithm, successful constrained Acrobot swing-up or demonstrated biped walking is claimed.

[Detailed experimental report](docs/ROBOTICS_REPORT.md) · [Preserved HW3 report](docs/TECHNICAL_REPORT.md) · [Raw numerical summaries](docs/results/robotics)
