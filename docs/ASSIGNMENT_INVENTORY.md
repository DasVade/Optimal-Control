# HW1-HW6 evidence and implementation inventory

The persistent WSL Drake adapter executes the supplied biped model and planner; see [WSL_SETUP.md](WSL_SETUP.md) for the measured diagnostics.

The inventory covers the supplied MEAM 5170 Fall 2025 HW1–HW6 materials.

## Sources and preservation

Archives are extracted, unchanged, under ignored `materials/{hw4,hw5,hw6,archive}`. [materials_manifest.json](materials_manifest.json) records every extracted archive entry's path, byte count, and SHA-256, including written PDFs, TeX, models, notebooks, and outputs. Mac metadata was excluded; bytecode is never executed. PDFs have local page-delimited text extracts; scanned HW1/HW2 solutions were rendered and visually read. Original notebooks retain embedded outputs locally. Small original source snapshots are in [course_sources](../course_sources/README.md). Original source credits are retained. Original root HW3 files and historical numerical results remain unchanged.

| Assignment | Requirements / mathematical formulation | Existing evidence, inspected bodies | Current portfolio scope |
|---|---|---|---|
| HW1 | Linear stabilization; underactuated Acrobot and fully actuated feedback linearization; scalar phase/bifurcation analysis; nonlinear equilibrium and phase portrait | Six-page scanned written solution; phase-portrait Python and three historical images. `5c.py` is only a stub. The substantive portrait code returns equilibria `(-1,2),(-1 +/- sqrt(3),2)` correctly; its explanatory cubic contains an extra constant typo. No executable Acrobot controller here. | Original portrait adapter executed; equilibrium residual checked. Analytical exercises retained as source evidence, not new controller achievements. |
| HW2 | Planar quadrotor cost design; finite-horizon time-varying LQR and square-root Riccati ODE; stochastic discounted GridWorld value iteration | Completed GridWorld transition and value-iteration bodies despite remaining TODO comments. Completed quadrotor Riccati/feedback bodies, but missing `trajectories.py` and simulation driver. Scanned solution explicitly says a correct initial condition was not obtained. Written TeX gives equations, not execution proof. | Original GridWorld executed at gamma .9/.1; Bellman residual exported. Original TVLQR preserved but not demonstrated. New HW6 hover LQR is separate. |
| HW3 | Differential flatness with positive forward speed; discrete LQR derivation; cart-pole energy shaping and Lyapunov analysis | Original supplied repository contains completed spline/feedforward and energy shaping/hybrid LQR. Archive's eight Python duplicates differ in bytes but have identical ASTs. Written PDF adds full task equations. Existing historical animations are qualitative. | Existing audited package, measured results, original sources and report preserved. No new reruns used to replace old measurements. |
| HW4 | Convexity, QP, parametric programming; waypoint minimum-snap QP, position constraints and continuity through derivative 4 | Standalone ZIP has zero position matrices and `pass` continuity/cost. `Archive/HW4/code` contains completed position, continuity and snap objective bodies, even though TODO comments remain. Notebook's stored OSQP error belongs to the unfinished version, not proof the completed copy fails. Written report has a placeholder trajectory figure. | New scaled numerical solver of the same formulation, 14 ascending coefficients per coordinate/segment; free endpoint derivatives. Completed original retained separately from template. |
| HW5 | Iterative LQR on quadrotor + pendulum; Hermite-Simpson direct collocation; bounded elbow torque and end-effector box avoidance | iLQR has supplied Lagrangian/dynamics, costs and outer loop, but zero gradients/Hessians/backward/forward passes. Acrobot dynamics wrapper exists; dynamics defects return zeros, obstacle distance is undefined, and optimization constraints/cost/guess are absent. URDF specifies rod COM inertia .33 about y and a 2 kg tip mass. | New portable iLQR and Acrobot collocation implementations, mathematical tests and initialization experiments. iLQR reaches goal numerically but hits iteration limit and uses negative thrust. Acrobot candidates violate dynamics/obstacle constraints; independent replays miss the target. |
| HW6 | Hover MPC in error coordinates, Euler prediction; operational-space walking control QP | Analytic continuous hover LQR exists. MPC constraint/cost helpers are `pass`, first action returns zeros. Simulator freezes the initial state inside the ODE callback; stops using position only. OSC has a literal stray backtick syntax error, missing joint Jacobian, contact Jacobian and QP bodies. Model, animation code, and instructor-marked footstep planner are supplied. Notebook records the syntax error. | New tested bounded MPC and corrected nonlinear simulation. Portable OSC QP passes a static-force check; the WSL adapter executes a ten-second multibody simulation with recorded contact and solver diagnostics. |

## Duplicate and output reconciliation

- [duplicate_comparison.json](duplicate_comparison.json) compares the eight archive HW3 Python files with the root originals using byte equality and parsed AST equality. All ASTs match despite formatting differences. Archive notebook outputs are historical and not substituted for saved new numeric traces.
- HW4 template versus completed code are substantively different. The completed standalone script uses waypoints `[-3,-4],[0,0],[2,3],[5,0],[8,-2]`; the notebook uses `[4,0],[8,-4]` as the last two. New experiments explicitly use the notebook waypoint set. These are not pooled as a single historical result.
- HW5 notebook requests N=11 but its stored Acrobot output has only five states and five inputs, cost 230.162679. It cannot verify the current source/configuration; its animation plays a prescribed state trajectory with zero plant input, not independent dynamics replay. The iLQR cell contains an error output. Neither is a reproducible historical success.
- HW6 embeds large animations but the saved walking setup fails importing `osc.py` with invalid syntax. Animation presence does not establish successful MPC or walking.

## Course models and implementation

The course supplies models, simulation infrastructure and problem formulations.
`quad_visualizer.py` credits Russ Tedrake's underactuated example;
`footstep_planner.py` is instructor-provided and cites an ALIP planning paper.
The Python package adds completed MPC, scaled minimum-snap, iLQR, Acrobot and OSC
solvers, experiment adapters and numerical checks. Original implementations and
templates remain in `course_sources/`, with their source credits.

Engineering observations: HW2 HJB derivation mixes half factors and its claim that the general square-root ODE preserves triangularity is false in general; HW3 text mislabels upright as theta=0 although its model uses pi, and Q positive semidefinite plus R positive definite alone does not make every closed-loop stage cost strictly positive. New numerical conventions follow the actual dynamics and handout objectives, not these prose mistakes.

## Local import procedure

Extract each attached archive into the corresponding `materials/` folder without flattening nested paths. The exact source paths are enumerated in the manifest. Then run:

```powershell
.\.venv\Scripts\python.exe scripts/reconcile_materials.py
```

This verifies source snapshots before copying and refuses to overwrite a different snapshot. Large source PDFs/notebooks/archives stay local; repository-only reproduction of the new numerical modules does not require them.
