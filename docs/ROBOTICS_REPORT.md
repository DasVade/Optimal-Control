# Robotic Motion Planning and Optimal Control: experimental report

**WSL follow-up:** The earlier Drake environment blocker has been resolved using a persistent project environment. The headless multibody adapter and new results are documented separately in [WSL_SETUP.md](WSL_SETUP.md); the original experiments and dated failures below remain unchanged.

October 23, 2026. This extends the preserved [HW3 report](TECHNICAL_REPORT.md). [Assignment inventory and provenance](ASSIGNMENT_INVENTORY.md) distinguish original student regions, course frameworks, missing functions, and new completions. These are established methods, not novel algorithms. All experiments use local CPU simulation; none use hardware or paid compute.

## Scope and status

| Module | Original state | New work | Validation / demonstration |
|---|---|---|---|
| Quadrotor LQR/MPC | Hover LQR exists; MPC helpers empty; faulty simulation callback | Sparse bounded error-input MPC, corrected held-input nonlinear integration, full-state criteria, solver/fallback logs | Mathematical and behavioral checks; nominal, horizon, weights, disturbance and mismatch experiments demonstrated |
| Minimum snap | Completed alternative and unfinished template both supplied | Scaled equality-nullspace solver of the same objective | Waypoints, C4 continuity, stationarity and duration scaling checked; trajectory demonstrated |
| Cart-pole / unicycle | Completed student regions in original HW3 | Prior audit/corrections and reproducible package at ae7498a | Existing verified experiments and figures preserved unchanged |
| Quadrotor-pendulum iLQR | Dynamics/cost scaffolding; solver passes empty | Discrete RK4 derivatives, backward pass, line search and regularization | Numerical goal approach; iteration limit, open-loop integration sensitivity and negative thrust remain |
| Acrobot | Drake model/dynamics wrapper; constraints/optimization empty | URDF-parameterized dynamics, Hermite-Simpson, bounds and end-effector obstacles | Model checks pass; all returned optimization candidates violate constraints; task feasibility unresolved |
| Biped OSC | Model and supplied planner present; incomplete controller with syntax error | Acceleration QP plus new headless Drake adapter | Multibody checks and ten-second simulation executed in persistent WSL; robust walking remains unverified |
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

The original callback evaluated derivatives at fixed `current_x`; this implementation evaluates every RK4 substage at its evolving state. It always simulates the full 10 s, avoiding a position-only early stop. Success at the final sample requires componentwise absolute errors below `[.02,.02,.02,.05,.05,.05]` in the state units. Settling is the first sampled time after which **all six** remain within these tolerances through the end. A null settling time is failure to settle over this finite duration, not proof of infinite-time instability. Position RMSE is `sqrt(mean(y^2+z^2))`, including the initial sample; control effort relative to hover integrates the squared **applied-input deviation from nominal hover**. Saturation fraction counts individual motor-time samples at either bound, not just clipped controller calls.

### Cost conventions and comparability

| Controller | State/input weights | Terminal cost | Prediction/model convention | Control update |
|---|---|---|---|---|
| Clipped LQR | Continuous `Qc=diag(10,10,1,1,1,1)`, `Rc=r I2` | No finite terminal cost; CARE value matrix `S` for the unconstrained infinite-horizon design | Continuous hover-linearized A/B; no prediction timestep | .02 s held inputs |
| MPC N=5/10/20 | Discrete `Qd=diag(10,10,1,1,1,1)`, `Rd=r I2`; stage sum has **no T factor** | `Pd=Qd` at state N-1; not CARE S | Euler `Ad=I+.1 A`, `Bd=.1 B`; T=.1 s; N-1 actions | .02 s held inputs |

The representative comparison uses r=1. Matched weight trials use r=.1,1,10 for both controllers. MPC counts the initial state stage cost (constant for a given solve), N-2 subsequent stage costs, and one terminal state cost. No half-factor ambiguity exists: OSQP uses `.5*z'P*z` with block Hessians `2Qd,2Rd`, producing exactly the stated sum.

LQR minimizes the continuous infinite-horizon integral of `x'Qc x+ue'Rc ue` for the linear unconstrained model. MPC minimizes an unscaled discrete finite sum plus `x_terminal'Qd x_terminal`. These are **not objective-equivalent designs**. Multiplying MPC's entire objective by T=.1 leaves its optimizer unchanged and yields `T sum(stage) + x_terminal'(T Qd)x_terminal`. Thus its rectangle-rule continuous interpretation uses terminal matrix **.1 Q**, not Q and not CARE S. If a finite-horizon continuous terminal Q had been intended, consistent rectangle scaling would require `T Q,T R,Q`, equivalently unscaled `Q,R,Q/T`; that is a different experiment and was not substituted. Stage Q/R ratios agree numerically, but terminal treatment, finite horizon, Euler prediction, sampling, constraints and clipping differ. LQR's continuous optimality also does not extend to its clipped nonlinear sampled implementation.

This is a matched-plant comparison of explicitly documented controller configurations, not an isolated test of constraint handling or proof MPC is superior. Within the MPC horizon sweep, prediction spacing, Q/R, terminal Q, controller update and plant remain fixed, so changing N is controlled. Prediction timestep itself was not swept. No weights or terminal costs were retuned during this review.

Both methods use the same `simulate` implementation: the same six-state nonlinear plant, nominal mass/inertia/arm/gravity, RK4 .005 s, .02 s held controller period, initial condition, clipping to [0,5.5] N, and full 10 s duration. The saved traces confirm matching initial states, update grids and duration. Six fixed-seed disturbance/mismatch pairs match force realization, mass and initial condition; force application uses the same [2,2.5) s interval. Neither method gets a different disturbance draw. The audit recomputes effort and RMSE from all saved quadrotor traces and verifies the paired settings. See [technical_review.json](results/robotics/technical_review.json) and `scripts/review_main_results.py`.

**Control effort relative to hover** is `E=integral_0^10 ||u_applied(t)-[4.905,4.905]||_2^2 dt` in N^2 s. Because applied inputs are held, `E=sum_k .02 * ||u_applied[k]-u_hover||^2` exactly integrates that input trace. Deviations below and above hover both contribute; raw controller commands are not used. The historical JSON key `control_effort_above_hover_N2s` is retained for compatibility and denotes this same metric; no old values are rewritten.

### Measured nominal comparison

Initial `[.5,.5,0,1,1,0]`, identical 10 s plant and motor limits:

| Controller | Position RMSE (m) | Settling (s) | Control effort relative to hover (N^2 s) | Peak thrust (N) | Saturation fraction | Median / p95 solve (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Clipped LQR | .2873 | 2.68 | 3.8730 | 5.5000 | .015 | .0015 / .0043 |
| MPC N=5 | .7994 | not settled | .8692 | 5.0740 | 0 | .0712 / .1097 |
| MPC N=10 | .3501 | 3.26 | 2.4021 | 5.2913 | 0 | .1064 / .1608 |
| MPC N=20 | .3257 | 2.62 | 2.8863 | 5.3156 | 0 | .1514 / .2519 |

From the saved applied-input traces, LQR effort is 3.872962685247318 N^2 s and MPC N=10 effort is 2.4020890610572736 N^2 s: `100*(1-E_MPC/E_LQR)=37.977996%` reduction. Position RMSE is .2873030749767701 versus .350134758210639 m: `100*(RMSE_MPC/RMSE_LQR-1)=21.869478%` increase. Both rounded claims (38% lower effort, approximately 22% higher position RMSE) remain valid for this configuration, with slower MPC settling. A longer MPC horizon improves this case's error at additional computation/effort. It is not universally superior. All three nominal initial conditions settle under LQR and N=10/20; N=5 fails two. See [comparison figure](figures/robotics/quadrotor_comparison.png) and [actual simulation GIF](figures/robotics/quadrotor.gif).

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

### Free-endpoint nullspace and independent objective audit

Free endpoint derivatives do not by themselves guarantee positive snap. Zero integrated squared snap requires every segment to be cubic or lower; C1-C3 continuity then makes these one global cubic in physical time. For the current five waypoints at t=0,1,2,3,4, the fourth finite differences are **1 in y and 10 in z**, rather than zero. The global cubic Vandermonde matrix has rank 4; augmenting it with the waypoint data raises rank to 5. No global cubic interpolates these data. The maximum waypoint residual of the least-squares global cubic is .85714 m; it is not a feasible zero-cost path. Uniformly scaling all times does not change this compatibility conclusion.

An independent coefficient-derivative construction checks the full QP per coordinate: 56 coefficients, 20 equality rows of rank 20, equality nullity 36. The fourth-derivative operator has rank 40 and a 16-dimensional kernel (four cubic coefficients on each of four segments). Restricting the equalities to this objective kernel gives rank 16, while augmenting with the right-hand sides raises rank to 17: no feasible zero-snap coefficients exist. On the 36-dimensional homogeneous equality nullspace, the snap operator has full column rank 36. Thus the quadratic curvature is strictly positive on every nonzero homogeneous feasible direction and the constrained minimizer is unique for this case. This rules out an unnoticed feasible zero-cost cubic; the nonzero reported objective was not used as the proof. Rank calculations use SVD tolerances on row-normalized equalities and the exact polynomial derivative map; raw high-degree monomials can still be ill-conditioned.

As a complementary regression check, five waypoints generated by a global cubic **do** produce near-zero snap under the same free-endpoint solver (test threshold 1e-15). No endpoint velocity/acceleration/jerk conditions were added to obtain the reported nonzero result.

The audit converts saved ascending normalized coefficients to descending physical local-time coefficients via `c_physical[j]=c_tau[j]/T^j`, constructs a separate SciPy PPoly, differentiates it four times, and integrates squared snap using adaptive quadrature on each segment. This does not reuse the solver's weighted Gauss sample matrix or `basis` evaluator. It obtains **210.6953642383607** for four 1 s segments, versus saved 210.69536423836075, with quadrature error estimate 2.34e-12. The .5 s and 2 s cases independently integrate to 26969.006622510267 and 1.646057533112199. These are corroborating checks, not replacements for the original metrics.

At the same normalized global-time samples, the three saved paths agree within 7.11e-15 m. Doubling segment/total duration gives objective ratio **.007812500000000028**, matching `2^(-7)=1/128`. Physically, four time derivatives contribute T^(-8) to squared snap and integration contributes T, hence T^(-7). Waypoint and derivative-continuity residuals are independently reconstructed and recorded in [technical_review.json](results/robotics/technical_review.json). Free endpoints and absence of thrust/obstacle constraints remain limitations of the demonstrated planner.

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

Accepted costs decrease, and discrete rollouts approach the goal, but convergence is **not** established. Independent open-loop replay using two .01 s RK4 steps per saved action produces final pendulum-rate differences .092-.116 rad/s and angle differences .028-.035 rad. Inverted open-loop dynamics amplify integration error; derivative correctness alone does not make this a reliable physical transfer. Negative thrust violates a conventional nonnegative-thrust rotor assumption, which was not imposed in this unconstrained optimizer. Large peak inputs are reported without inventing an unmodeled hardware maximum. These observations invalidate direct execution of these particular inputs on nonnegative-thrust motors; they do not prove the target task physically infeasible. The report claims numerical goal approach only, not successful constrained flight, robust tracking or global optimality. [Costs, angles and actual inputs](figures/robotics/ilqr.png).

## E. Constrained Acrobot: completed formulation, unsuccessful solves

The HW5 URDF differs from HW1's simple Acrobot: each rod has mass 1 kg, COM at .5 m, Iyy=.33, and an additional 2 kg mass at the tip. Unit links, gravity 9.81, relative elbow angle and downward zero give

```text
M11 = 6.16 + 5 cos(q1), M12 = 2.58 + 2.5 cos(q1), M22 = 2.58
Cv = [-2.5 sin(q1)(2 v0 v1+v1^2), 2.5 sin(q1) v0^2]
G  = 9.81 [3.5 sin(q0)+2.5 sin(q0+q1), 2.5 sin(q0+q1)]
M qdd = [0,u] - Cv - G
```

Equilibrium and energy-power checks verify zero-input conservation and `dE/dt=u*v1`. The original Windows run lacked Drake cross-validation; the subsequent persistent WSL environment now independently verifies this portable dynamics function against the supplied URDF at fixed-seed states and torques. That model check does not change the unsuccessful optimization results.

Hermite-Simpson defects use the handout midpoint state `.5*(xi+xi1)+h/8*(fi-fi1)` and midpoint derivative `1.5/h*(xi1-xi)-.25*(fi+fi1)`, with first-order-hold midpoint input. Initial state fixed zero; final **configuration only** `[pi,0]`, terminal velocities free. Node q bounds +/-pi, velocity +/-15 rad/s, torque +/-30 Nm. Objective is trapezoidal integral of u squared. Final fixed velocity or rest-to-rest claims would be false.

Box centers are (+/-1.5,0), width 2 m, height 1 m; the tip ball radius is .1 m. Euclidean signed box distance minus ball radius is constrained at nodes and, as an explicit new extension, Hermite midpoints. Only end-effector avoidance is modeled: links/base/whole-body collision are not constrained. Dense checks sample 2001 times; they do not certify continuous clearance. Cubic Hermite state reconstruction is compared with tight adaptive solve_ivp replay of the **linearly interpolated optimized input**. This fixes the source reconstruction's mismatch between linear collocation controls and zero-order-hold replay.

The new development mesh is 21 nodes at .175 s (3.5 s total), versus source 11 at .35 s. SLSQP uses ftol 1e-8. Seeds 0/1/2 perturb the state/torque guesses; a linear-angle initialization gets 250 iterations, and a folded-elbow initialization gets 600. All six runs fail feasibility. For linear seed 0 the max defect is 39.05, dense spline clearance -0.596 m, independent final configuration error 4.406 rad; folded seed 0 still has defect 50.09, clearance -0.595 m and replay error 2.889 rad. These paths cannot be presented as optimized swing-up results. Some replays avoid obstacles simply because they never swing up. [Failed candidates versus independent replay](figures/robotics/acrobot_failures.png) and all unsuccessful statuses remain in the summaries. A future sparse derivative-aware NLP, continuation or mesh refinement is a possible investigation, not a completed feature.

### Failure classifications and Acrobot trial audit

**Solver termination failure** means an optimizer did not report success (iteration limit, line-search failure, or QP failure). **Target nonattainment** means a trajectory or independent rollout misses its stated terminal/success criterion. **Constraint violation** refers to a specific returned candidate's measured defects, bounds or clearance. None of these alone proves **task-level physical infeasibility**, which would require an independent reachability argument or certificate. No such certificate exists for the Acrobot task. Its failed candidates are infeasible points of the transcription, not evidence that every possible motion is impossible.

| Acrobot initialization / seed | Solver status | Max dynamics defect | Dense spline clearance (m) | Independent final q error (rad) | Independent clearance (m) |
|---|---|---:|---:|---:|---:|
| Linear / 0 | Iteration limit reached | 39.051 | -.596 | 4.406 | .292 |
| Linear / 1 | Positive directional derivative for linesearch | 20.407 | -.599 | 2.915 | 1.223 |
| Linear / 2 | Positive directional derivative for linesearch | 34.658 | -.596 | 3.598 | 1.063 |
| Folded / 0 | Positive directional derivative for linesearch | 50.088 | -.595 | 2.889 | .041 |
| Folded / 1 | Positive directional derivative for linesearch | 33.051 | -.598 | 4.194 | -.403 |
| Folded / 2 | Positive directional derivative for linesearch | 30.193 | -.593 | 3.048 | .404 |

Boundary equalities alone have residuals below 6e-15; their satisfaction does not compensate for these dynamics/obstacle violations. Negative clearance means the ball intersects a box. Independent rollouts integrate the optimized first-order-hold torque, not the collocation spline; all miss the target, and folded seed 1 also violates sampled obstacle clearance. Other replays have positive clearance while failing to swing up. Full metrics retain node/midpoint clearance, integration success and state discrepancies. Task feasibility is unresolved, and no successful swing-up claim is made.

For the quadrotor sweep, QPs report no solver failure even in cases failing the final full-state criterion; those are closed-loop target/settling failures. The clipped applied inputs satisfy motor bounds. iLQR's iteration-limit status is a convergence limitation even where the discrete endpoint is close to the target. For OSC, the ten-second simulation executes with one logged QP iteration-limit fallback, torso lean and contact-velocity diagnostics; finite execution and forward displacement are not a robust-walking certificate. Historical solver statuses and raw metrics are preserved without relabeling failed runs as successes.

## F. Biped OSC: historical blocker and current boundaries

The environment blocker described below is historical. The persistent WSL adapter now passes both-foot/COM/torso kinematic checks and completes a ten-second simulation; actuator-order limits are mapped and diagnostics are recorded. [WSL_SETUP.md](WSL_SETUP.md) contains the current results and limitations. Robust walking, hardware deployment and real-time operation remain unverified. The earlier source/environment observations below are retained for provenance.

The supplied planar walker model has seven position/seven velocity coordinates, including planar translations and a torso revolute joint; qdot=v. Joint task J must index `velocity_start`, with zero bias for that scalar revolute task. Point/COM Jacobians should use Drake `kV` in world coordinates and matching bias accelerations. The original supplied methods for those point/COM quantities are preserved; the missing joint/contact implementations and notebook wiring are not declared validated.

The portable QP core minimizes weighted task acceleration error plus vdot regularization under `M vdot + Cv + G = B u + Jc'lambda`, `Jc vdot + Jdotc v=0`, world lambda_y=0, lambda_z>=0, |lambda_x|<=mu lambda_z and supplied input bounds. A static 2 kg point-mass contact test recovers positive 19.62 N normal force. This tests algebraic signs, not real robot Jacobians/contact switching. The URDF's actuator limits (hip 500, knee 400 Nm) must be mapped in actuator order in a future multibody adapter; the generic core accepts bounds explicitly and does not assume that mapping.

Active Windows Python has no pydrake. Existing Ubuntu-22.04 WSL Python is 3.10.12; an actual import returned ModuleNotFoundError, and `python3 -m pip --version` also failed. Permission to create a temporary WSL environment was declined. No workaround installation or further WSL mutation was attempted. `control-portfolio osc` records this blocker; it does not pretend to run walking. The instructor-marked footstep planner is preserved unchanged, with its cited planning method. No successful walking, verified multibody OSC or personal footstep-planning algorithm is claimed.

## Reproduction and next review

See [REPRODUCTION.md](REPRODUCTION.md) for exact CLI commands, archived run names, tests and figure regeneration. The key review files are the quadrotor constraints/simulation, minimum-snap scaling, iLQR dynamics/discrete derivatives, Acrobot defects/replay checks and this report's failure classifications. New methods are usable development implementations with explicit limitations; successful deployment, novel algorithms and real-time capability remain unsupported.
