# Robotic Motion Planning and Optimal Control: experimental report

October 23, 2026. This extends the preserved [HW3 report](TECHNICAL_REPORT.md). [Assignment inventory and provenance](ASSIGNMENT_INVENTORY.md) distinguish original student regions, course frameworks, missing functions, and new completions. These are established methods, not novel algorithms. All experiments use local CPU simulation; none use hardware or paid compute.

## Scope and status

| Module | Original state | New work | Validation / demonstration |
|---|---|---|---|
| Quadrotor LQR/MPC | Hover LQR exists; MPC helpers empty; faulty simulation callback | Sparse bounded error-input MPC, corrected held-input nonlinear integration, full-state criteria, solver/fallback logs | Mathematical and behavioral checks; nominal, horizon, weights, disturbance and mismatch experiments demonstrated |
| Minimum snap | Completed alternative and unfinished template both supplied | Scaled equality-nullspace solver of the same objective | Waypoints, C4 continuity, stationarity and duration scaling checked; trajectory demonstrated |
| Cart-pole / unicycle | Completed student regions in original HW3 | Prior audit/corrections and reproducible package at ae7498a | Existing verified experiments and figures preserved unchanged |
| Quadrotor-pendulum iLQR | Dynamics/cost scaffolding; solver passes empty | Discrete RK4 derivatives, backward pass, line search and regularization | Goal approach demonstrated numerically; iteration limit and open-loop integration sensitivity remain; physically infeasible negative thrust |
| Acrobot | Drake model/dynamics wrapper; constraints/optimization empty | URDF-parameterized dynamics, Hermite-Simpson, bounds and end-effector obstacles | Equilibrium/energy/defect checks pass; all nonlinear optimization trials infeasible; no swing-up demonstration |
| Biped OSC | Model and supplied planner present; incomplete controller with syntax error | Portable acceleration QP core with correct force signs, contact/friction/input constraints | Static matrix test only. Multibody Jacobians and walking not executed; blocked by Drake environment |
| HW1 / HW2 support | Completed portrait and GridWorld; incomplete-dependency TVLQR | Headless adapters and residual checks | Portrait and discounted policies reproduced; original TVLQR not demonstrated |

## Shared experiment discipline

Saved runs are immutable folders under ignored `outputs/`; the CLI refuses an existing output path. Every run saves its configuration, environment, summary, and numerical traces. State and input CSVs accompany NPZ data; MPC stores every solve status and residual in `solver_log.json`, including fallback events. Small summaries/configurations/environments are curated under [results/robotics](results/robotics); complete local traces remain in the corresponding `outputs/*_20261023` folders. The previous HW3 results are untouched.

This is an explanatory development study, not a held-out evaluation or a certified robustness study. Configurations were selected for comparison and debugging. Initial conditions, simulation duration, plant, motor limits, update period and integrator are matched within controller comparisons. Nonlinear optimizers solve different tasks and are not ranked against hover controllers.

Hardware: Intel Core i7-13700K, 24 logical CPUs, Windows 11, Python 3.12.14. NumPy 2.5.4, SciPy 1.18.1, Matplotlib 3.11.2, Pillow 13.0.0, OSQP 1.1.3. Exact installed versions are in [requirements-lock.txt](../requirements-lock.txt). Wall-clock times are local observations, not real-time guarantees. An initial one-second MPC smoke benchmark preceded the sweep; a separately saved repeat is `robotics_benchmark_20261023`. Three-iteration iLQR and Acrobot benchmarks took 0.607 s and 0.156 s for their dispatches, estimating roughly 73 s for three 120-iteration iLQR trials and 39 s for three 250-iteration Acrobot trials by linear extrapolation. Actual sweeps took 104.15 s and 6.72 s; rejected line searches and early terminations make linear estimates approximate. No external job was started.

## A. Planar quadrotor: equations and MPC

State `[y,z,theta,vy,vz,omega]` uses m, rad, s; input `[u1,u2]` is motor thrust in N. Nominal `m=1 kg, I=.0625 kg m^2, a=.25 m, g=9.81 m/s^2`. Acceleration is `[-sin(theta)(u1+u2)/m, -g+cos(theta)(u1+u2)/m, a(u1-u2)/I]`. Hover is zero state with each motor at 4.905 N; derivative tests verify the handout A/B including `A[vy,theta]=-g`. The mismatch changes only mass to 1.1 kg, leaving I and a unchanged, explicitly an idealized parameter perturbation.

MPC optimizes error states `xe=x-xd` and error inputs `ue=u-ud`. It fixes the measured initial state, enforces every predicted transition, and bounds each error input in `[-4.905,.595]`, equivalent to actual thrust `[0,5.5]`. Objective is sum of `xe'Q xe + ue'R ue` plus terminal `xe'Qf xe`, `Q=diag(10,10,1,1,1,1)`, `R=r I`, `Qf=Q`. No terminal invariant set, state constraint, obstacle constraint or nonlinear prediction is used. The first optimized error action is converted back to physical thrust. This is constrained optimization; the comparator is **unconstrained continuous CARE LQR followed by clipping**, not constrained LQR.

The assignment's forward Euler predictor is `Ad=I+A*T, Bd=B*T`. Prediction T=.1 s; controller update=.02 s; nonlinear RK4 plant step=.005 s. `N` counts predicted states, so N=5/10/20 means 4/9/19 actions and .4/.9/1.9 s prediction horizons. The input is held between updates for both methods. Replanning more frequently than the prediction spacing is a chosen sampled implementation, not five applications of a predicted action. Future work could compare matched prediction/update periods and exact ZOH; neither is silently substituted here.

OSQP uses absolute/relative tolerances 1e-6, max 10000 iterations, warm starts and polishing. A solve is accepted only with status `solved` and explicit maximum linear-constraint violation <1e-4. Otherwise an event is logged and clipped LQR is the fallback. The forced one-iteration failure test exercises this branch; there were zero solver failures in the reported sweep. Tiny final clipping protects physical motor bounds and is separately observable in commanded/applied traces. No CLF controller or flag is exposed.

The original callback evaluated derivatives at fixed `current_x`; this implementation evaluates every RK4 substage at its evolving state. It always simulates the full 10 s, avoiding a position-only early stop. Success at the final sample requires componentwise absolute errors below `[.02,.02,.02,.05,.05,.05]` in the state units. Settling is the first sampled time after which **all six** remain within these tolerances through the end. A null settling time is failure to settle over this finite duration, not proof of infinite-time instability. Position RMSE is `sqrt(mean(y^2+z^2))`, including the initial sample; effort is integral of squared **applied input above nominal hover**. Saturation fraction counts individual motor-time samples at either bound, not just clipped controller calls.

### Measured nominal comparison

Initial `[.5,.5,0,1,1,0]`, identical 10 s plant and motor limits:

| Controller | Position RMSE (m) | Settling (s) | Effort (N^2 s) | Peak thrust (N) | Saturation fraction | Median / p95 solve (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Clipped LQR | .2873 | 2.68 | 3.8730 | 5.5000 | .015 | .0015 / .0043 |
| MPC N=5 | .7994 | not settled | .8692 | 5.0740 | 0 | .0712 / .1097 |
| MPC N=10 | .3501 | 3.26 | 2.4021 | 5.2913 | 0 | .1064 / .1608 |
| MPC N=20 | .3257 | 2.62 | 2.8863 | 5.3156 | 0 | .1514 / .2519 |

MPC N=10 spends 38.0% less above-hover effort than clipped LQR in this case, while having higher position RMSE and slower settling. A longer MPC horizon improves this case's error at additional computation/effort. It is not universally superior. All three nominal initial conditions settle under LQR and N=10/20; N=5 fails two. See [comparison figure](figures/robotics/quadrotor_comparison.png) and [actual simulation GIF](figures/robotics/quadrotor.gif).

### Weights, disturbances, mismatch and failures

With R=.1 I at the same initial condition, clipped LQR fails (RMSE 81.08 m, saturation .891), while MPC N=10 settles in 2.60 s (RMSE .2984 m). With R=10 I, MPC does not settle within 10 s (RMSE .5730 m), while clipped LQR settles. These are controller tradeoffs, not evidence that an optimization cost is a physical accuracy metric.

Fixed seeds 0/1/2 draw horizontal forces uniformly in [-1,1] N, applied during [2,2.5) s. Both baseline methods settle after these nominal-plant disturbances. Under 10% mass mismatch, both methods fail the full-state criterion for all three seeds; neither includes integral action or disturbance estimation. Excessive tilt/clipping can lead to large falling trajectories. The mathematical plant has no ground collision, aerodynamic drag, attitude envelope or crash termination, so its large error values document numerical failure, not realistic post-crash flight. [Failure plots](figures/robotics/quadrotor_failures.png) retain the full magnitudes.

Halving RK4 plant step to .0025 s changes the nominal LQR and N=10 MPC sampled states by at most 3.54e-10 and 1.01e-10 respectively. This supports integrator convergence for those cases only. Solver residuals apply to the linear predictor, not nonlinear model accuracy. Full distributions and final component errors remain in the raw summaries.

## B. Minimum-snap planning

Notebook waypoints are `[-3,-4],[0,0],[2,3],[4,0],[8,-4]`. Four polynomial segments have 14 coefficients (degree 13), ascending powers of local normalized time `tau=t/T`. The original interleaved y/z coefficients are separated by coordinate in the new linear algebra; output coefficients explicitly use `[segment,power,coordinate]`. Endpoints meet each waypoint and derivatives 1-4 match between segments. **Endpoint velocity, acceleration and jerk are unconstrained**, as in the assignment; this is not a rest-to-rest trajectory.

Minimize `sum integral ||d^4 gamma/dt^4||^2 dt`. Row-normalized equalities yield a particular solution plus an SVD nullspace. Weighted Gauss-Legendre snap samples exactly integrate the squared polynomial; least squares in the nullspace avoids explicitly squaring the condition number in a monomial Hessian. No ridge or altered objective is added. The reduced system still has condition number about 2.82e6, rank 36, so conditioning is reported rather than claiming all parameter choices are safe. There is no iterative QP status to invent: the solver reports its least-squares rank, condition, equality residuals, projected gradient and objective.

| Segment duration (s) | Total (s) | Integrated squared snap | Peak speed (m/s) | Peak acceleration (m/s^2) | Max waypoint / continuity residual |
|---|---:|---:|---:|---:|---:|
| .5 | 2 | 26969.0066 | 11.9310 | 39.9695 | 1.07e-14 / 1.12e-9 |
| 1 | 4 | 210.6954 | 5.9655 | 9.9924 | 1.12e-14 / 6.91e-11 |
| 2 | 8 | 1.6461 | 2.9827 | 2.4981 | 1.07e-14 / 4.37e-12 |

Residual derivative rows have their corresponding physical derivative units; the maximum is a numerical diagnostic, not a dimensionless physical error score. Peaks are sampled on 1001 times, not certified continuous maxima. Uniformly doubling duration reduces integrated squared snap by 128x, speed by 2x and acceleration by 4x, matching time-scaling identities. [Figure](figures/robotics/minimum_snap.png). This is flat-output planning only: thrust/tilt feasibility, obstacle avoidance and nonlinear quadrotor tracking are not demonstrated for these paths.

## C. Preserved HW3 and supporting examples

Cart-pole energy shaping, corrected upright linearization, angle wrapping and LQR/hybrid saturation comparisons remain documented in [the original detailed report](TECHNICAL_REPORT.md), with its existing raw metrics and GIF. The unicycle remains a heuristic obstacle-dependent spline with feedforward flatness inversion, not a generic optimal planner or robust feedback tracker. No historical metrics were replaced by new ones.

HW2's existing stochastic GridWorld transition/value-iteration bodies run headlessly with gamma=.9/.1, slip probability .1, tolerance 1e-8 and deterministic N/E/S/W tie-breaking. They converge in 27 and 9 updates respectively, with saved Bellman residuals; these differ from the historical 21/6 updates because the stopping tolerance is tighter. The HW1 original portrait is executed with equilibrium residual checked. Its historical basin illustrations do not establish a certified region of attraction. The supplied HW2 TVLQR cannot run without its missing trajectory dependency, and its inverse-transpose/terminal factor/reversed-time handling need correction before any tracking claim.

## D. Quadrotor with pendulum: iLQR

State `[y,z,theta,phi,vy,vz,omega,phidot]`; phi=0 downward, pi inverted. Parameters follow the supplied Lagrangian: quad mass 1 kg, pendulum point mass .2 kg, length 1 m, moment arm .25 m and quad inertia .0625. The full 4x4 mass matrix retains translation/pendulum coupling. Correct hover input is `(1+.2)*g/2=5.886 N` per motor, rather than template 4.905 N. It is not subject to the HW6 system's different 5.5 N bound.

The new discrete map is explicit RK4 with dt=.02 s. Complex-step derivatives are taken through that exact map, rather than ZOH-discretizing a continuous Jacobian while rolling out adaptive solve_ivp. Central-difference checks verify A/B away from equilibrium. Running cost is `.5*dt*(error'Q error + input_error'R input_error)`, terminal `.5*error'Qf error`, Q=I, R=.001 I, Qf=100 I. Costs and their quadratic derivatives are used directly in the backward pass. Positive-definite regularization, Cholesky checks and descending line-search steps [1,.5,.25,.125,.0625,.03125,.015625] protect the local updates. Every attempt and accepted objective is recorded. No constraints or input clipping are used.

N=200 states (3.98 s), initial zero, goal `[-.5,1.5,0,pi,0,0,0,0]`. Seed 0 starts with corrected hover; seeds 1/2 add fixed .5 N standard-deviation Gaussian input noise. All run a budget of 120 iterations with relative cost-change tolerance 1e-5. This is initialization sensitivity on the same development task.

| Seed | Initial cost | Final cost | Final pendulum error (rad) | Input min / peak absolute (N) | Termination |
|---|---:|---:|---:|---:|---|
| 0 | 643.0957 | 16.2336 | .002678 | -4.154 / 31.269 | iteration limit |
| 1 | 2278.7740 | 16.3469 | .002698 | -3.961 / 30.357 | iteration limit |
| 2 | 134089.5559 | 16.2418 | .002679 | -4.157 / 31.183 | iteration limit |

Accepted costs decrease, and discrete rollouts approach the goal, but convergence is **not** established. Independent open-loop replay using two .01 s RK4 steps per saved action produces final pendulum-rate differences .092-.116 rad/s and angle differences .028-.035 rad. Inverted open-loop dynamics amplify integration error; derivative correctness alone does not make this a reliable physical transfer. Negative thrust and large peak inputs are further feasibility failures. The report therefore claims numerical goal approach only, not successful constrained flight, robust tracking or global optimality. [Costs, angles and actual inputs](figures/robotics/ilqr.png).

## E. Constrained Acrobot: completed formulation, unsuccessful solves

The HW5 URDF differs from HW1's simple Acrobot: each rod has mass 1 kg, COM at .5 m, Iyy=.33, and an additional 2 kg mass at the tip. Unit links, gravity 9.81, relative elbow angle and downward zero give

```text
M11 = 6.16 + 5 cos(q1), M12 = 2.58 + 2.5 cos(q1), M22 = 2.58
Cv = [-2.5 sin(q1)(2 v0 v1+v1^2), 2.5 sin(q1) v0^2]
G  = 9.81 [3.5 sin(q0)+2.5 sin(q0+q1), 2.5 sin(q0+q1)]
M qdd = [0,u] - Cv - G
```

Equilibrium and energy-power checks verify zero-input conservation and `dE/dt=u*v1`. Drake cross-validation is unavailable, so the portable derivation is not described as an executed Drake model.

Hermite-Simpson defects use the handout midpoint state `.5*(xi+xi1)+h/8*(fi-fi1)` and midpoint derivative `1.5/h*(xi1-xi)-.25*(fi+fi1)`, with first-order-hold midpoint input. Initial state fixed zero; final **configuration only** `[pi,0]`, terminal velocities free. Node q bounds +/-pi, velocity +/-15 rad/s, torque +/-30 Nm. Objective is trapezoidal integral of u squared. Final fixed velocity or rest-to-rest claims would be false.

Box centers are (+/-1.5,0), width 2 m, height 1 m; the tip ball radius is .1 m. Euclidean signed box distance minus ball radius is constrained at nodes and, as an explicit new extension, Hermite midpoints. Only end-effector avoidance is modeled: links/base/whole-body collision are not constrained. Dense checks sample 2001 times; they do not certify continuous clearance. Cubic Hermite state reconstruction is compared with tight adaptive solve_ivp replay of the **linearly interpolated optimized input**. This fixes the source reconstruction's mismatch between linear collocation controls and zero-order-hold replay.

The new development mesh is 21 nodes at .175 s (3.5 s total), versus source 11 at .35 s. SLSQP uses ftol 1e-8. Seeds 0/1/2 perturb the state/torque guesses; a linear-angle initialization gets 250 iterations, and a folded-elbow initialization gets 600. All six runs fail feasibility. For linear seed 0 the max defect is 39.05, dense spline clearance -0.596 m, independent final configuration error 4.406 rad; folded seed 0 still has defect 50.09, clearance -0.595 m and replay error 2.889 rad. These paths cannot be presented as optimized swing-up results. Some replays avoid obstacles simply because they never swing up. [Failed candidates versus independent replay](figures/robotics/acrobot_failures.png) and all unsuccessful statuses remain in the summaries. A future sparse derivative-aware NLP, continuation or mesh refinement is a possible investigation, not a completed feature.

## F. Biped OSC: exact blocker and boundaries

The supplied planar walker model has seven position/seven velocity coordinates, including planar translations and a torso revolute joint; qdot=v. Joint task J must index `velocity_start`, with zero bias for that scalar revolute task. Point/COM Jacobians should use Drake `kV` in world coordinates and matching bias accelerations. The original supplied methods for those point/COM quantities are preserved; the missing joint/contact implementations and notebook wiring are not declared validated.

The portable QP core minimizes weighted task acceleration error plus vdot regularization under `M vdot + Cv + G = B u + Jc'lambda`, `Jc vdot + Jdotc v=0`, world lambda_y=0, lambda_z>=0, |lambda_x|<=mu lambda_z and supplied input bounds. A static 2 kg point-mass contact test recovers positive 19.62 N normal force. This tests algebraic signs, not real robot Jacobians/contact switching. The URDF's actuator limits (hip 500, knee 400 Nm) must be mapped in actuator order in a future multibody adapter; the generic core accepts bounds explicitly and does not assume that mapping.

Active Windows Python has no pydrake. Existing Ubuntu-22.04 WSL Python is 3.10.12; an actual import returned ModuleNotFoundError, and `python3 -m pip --version` also failed. Permission to create a temporary WSL environment was declined. No workaround installation or further WSL mutation was attempted. `control-portfolio osc` records this blocker; it does not pretend to run walking. The instructor-marked footstep planner is preserved unchanged, with its cited planning method. No successful walking, verified multibody OSC or personal footstep-planning algorithm is claimed.

## Reproduction and next review

See [REPRODUCTION.md](REPRODUCTION.md) for exact CLI commands, archived run names, tests and figure regeneration. The key review files are the quadrotor constraints/simulation, minimum-snap scaling, iLQR dynamics/discrete derivatives, Acrobot defects/replay checks and this report's failure classifications. New methods are usable development implementations with explicit limitations; successful deployment, novel algorithms and real-time capability remain unsupported.
