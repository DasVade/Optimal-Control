# Audit and contribution boundaries

**Scope update, October 23, 2026:** The sections below preserve the earlier audit of the original HW3-only repository. New HW1-HW6 attachments now supply the previously missing handouts and additional methods. Global absence statements below apply to that original repository only, not the expanded project. The authoritative updated inventory is [ASSIGNMENT_INVENTORY.md](ASSIGNMENT_INVENTORY.md), and new implementation/results boundaries are in [ROBOTICS_REPORT.md](ROBOTICS_REPORT.md). Completed minimum-snap and GridWorld sources are present, finite-horizon TVLQR code has missing dependencies, and new MPC/iLQR/collocation completions are explicitly labeled new work.

## Inspected source

Repository: `DasVade/Optimal-Control`. Original commit: `3efbb4c` ("Initial commit: add HW3 materials"). The implementation branch `codex/portfolio-update` was merged into `main`; source history is preserved. Original Python files, notebooks, and two PNGs remain unchanged; their SHA-256 values are recorded in [source_manifest.json](source_manifest.json). Tracked Python bytecode caches are removed from the new index, retained in history, and ignored locally.

The GitHub app returned 404, but the user's existing Git Credential Manager authentication successfully cloned the repository and the authenticated CLI confirmed private visibility. No new remote is created or changed.

## What actually exists

| Material | Verified contents | Attribution boundary |
|---|---|---|
| `unicycle_spline.py` | Cubic splines from (0,0) to (10,0), increasing forward coordinate; obstacle-dependent lateral plateau | Completed TODO-region implementation in supplied work; no numerical objective or optimization solver |
| `unicycle_input.py` | Differential-flatness inverse producing `[yaw rate, forward speed]` | Completed TODO region; feedforward only, no state-error feedback |
| `unicycle_sim.py`, `simulate_unicycle.py` | Duplicate simulation entry points, random obstacle class attributes drawn at import; SciPy solve_ivp | Mixed framework/helpers; no separate pristine starter exists to establish line-level authorship |
| `cartpole.py` | Energy shaping, PD cart recentering, partial feedback linearization, continuous LQR and heuristic switching | Completed TODO regions and parameter tuning in supplied work; original Drake dependency |
| `cartpole_sim.py` | Nonlinear two-degree-of-freedom mass-matrix dynamics, solve_ivp with max_step=0.001, animation | Simulator/framework-style code; instructor authorship cannot be conclusively assigned from one initial commit |
| `hw3.ipynb` | Headings for Problem 1, "Unicycle Differential Flatness," and Problem 4, "Energy Shaping"; simulation calls, embedded animation outputs, figure exports | Thin task context, not the full assignment instructions |
| `Init.ipynb` | Deepnote initialization template; output says no requirements.txt existed | Third-party platform boilerplate; its package-install instructions are source content, not instructions for this task |
| Original two PNGs | Historical qualitative outputs | No accompanying numeric traces, seeds, or reproducible configuration; not used as new measurements |

No MATLAB files, MPC implementation, receding-horizon solver, finite-horizon planner, quadrotor controller, minimum-snap formulation, GridWorld, or generic numerical trajectory optimizer is present. Cart-pole LQR is present; quadrotor LQR is not. Problems 2/3 and other coursework cannot be inferred from notebook numbering.

## Missing materials and unsupported resume claims

Missing: full HW3 PDF/handout; formal problem statements, grading tests and rubrics; a pristine instructor starter; author-by-author contribution records; original environment/Drake version; original numerical logs/seeds. Consequently, equations can be checked against the supplied simulator and mathematical derivatives, but not certified against absent course instructions. A comment mentioning "4/5" is not a verified score.

- **LQR:** implemented originally, but the supplied A matrix does not match the simulator. New results use the audited Jacobian.
- **MPC:** unsupported by these materials. Remove the claim unless another implementation is supplied; no MPC was added.
- **Trajectory optimization:** unsupported here as a numerical optimization claim. The unicycle planner is a manually designed interpolating-spline heuristic.
- **Horizon comparisons:** unsupported; continuous infinite-horizon LQR has no planning horizon. Simulation duration is not a planning horizon.
- **Q/R comparisons:** not present in original saved results; now measured as a new controlled extension.

## Corrections and extensions

New `src/control_portfolio/` modules, CLI, configurations, tests, metric exports, selected figures/GIF, and report were developed with coding-assistant support. New code separates nominal controller parameters from true plant parameters, wraps upright errors, and clips actuator commands consistently in both controller modes. Original files remain the traceable reference, not silently replaced.

The original LQR A[2,2]=1 models cart-velocity dependence absent from the nonlinear dynamics. A[2,1]=g*mc/mp also has the wrong mass ratio (hidden when both masses equal one). The audited derivative is g*mp/mc. The original unwrapped switching/LQR error mishandles equivalent angles differing by 2pi. The original force cap applies only during energy shaping; new clipping also applies to LQR and is explicitly not constrained optimization. Normalized energy uses g/L for general pole length; the original g expression assumes L=1.

SciPy's continuous algebraic Riccati solver removes the required Drake installation from the new reproducible path. The "legacy linearization" cases are a **SciPy reimplementation of the original continuous LQR equations**, with the legacy A coefficients retained; they are not a verified execution of the original Drake stack. Original notebooks remain runnable only with their dependencies/environment restored. No unsupported controller was introduced to match a resume claim.

Third-party dependencies: NumPy, SciPy, Matplotlib, Pillow, pytest; original code additionally imports Drake/IPython. The spline/control methods are established methods and course implementations, not new algorithms. The owner confirmed on October 24, 2026 that course rules impose no publication restrictions and authorized public release. Original credits and third-party notices are retained; local data, environments and temporary outputs are excluded. See [current release notes](RELEASE.md).
