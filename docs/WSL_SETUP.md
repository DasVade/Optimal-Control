# Persistent WSL environment and Drake follow-up

The Drake workflow uses a persistent Python 3.11 environment in Ubuntu-22.04 WSL.

- Persistent environment: `/home/yxlewis/.venvs/optimal-control`
- Interpreter: `/home/yxlewis/.venvs/optimal-control/bin/python`
- Source: the existing Windows checkout mounted at `/mnt/c/Users/Lewis/Documents/New project/Optimal-Control`
- Drake 1.51.1, NumPy 2.4.6, SciPy 1.17.1, OSQP 1.1.3. Full versions: [requirements-wsl-lock.txt](../requirements-wsl-lock.txt).

The environment is outside `/tmp` and outside Git. Source remains in the same repository with the same history and branch. Ordinary pip build caches may be temporary; the interpreter and installed project dependencies persist. Commands do not require Conda activation.

## Reuse

From PowerShell in the repository root:

```powershell
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python -m pytest -q
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/run_wsl_osc.py --duration 0.2 --output 'outputs/my_wsl_osc_benchmark'
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/run_wsl_osc.py --duration 5 --output 'outputs/my_wsl_osc_5s'
```

Choose new output folders. The headless runner saves environment metadata, finite-difference kinematic checks, simulation metrics, state traces and every QP status. It does not start Meshcat or deploy a website. The ordinary core CLI also works with this WSL interpreter; the Windows and WSL locks are separate because their Python/platform versions differ.

## Rebuild or update

From an Ubuntu WSL shell:

```bash
cd '/mnt/c/Users/Lewis/Documents/New project/Optimal-Control'
bash scripts/setup_wsl.sh
```

The setup script reuses the persistent environment if present. On a fresh machine, set `OPTIMAL_CONTROL_BASE_PYTHON` to a working Python 3.11 interpreter if the existing `hw2` Conda path differs. `OPTIMAL_CONTROL_ENV` can select another persistent environment. A mismatched Python version is rejected before installing the lock. The project is installed editable, pointing to this checkout; its original course-source snapshots remain unchanged.

## New adapter and measured checks

[osc_drake.py](../src/control_portfolio/osc_drake.py) implements the headless LeafSystem adapter around the portable QP, supplied tracking framework, URDF and footstep planner. New work includes the missing scalar joint Jacobian/bias, stance-foot contact terms, actuator-order bounds, plant/controller connections, state initialization and status logging. The planner's existing discrete update was not registered in its constructor; the adapter registers it at 1 ms. Its NumPy-2-incompatible one-element-array conversion is replaced with `.item()` in an adapter subclass. The original planner file is preserved and its planning equations are unchanged.

The 0.2 s benchmark completes with 400 QP evaluations and zero failures. Finite-difference stance-foot checks give max Jacobian error 1.585e-11 and bias error 2.249e-6 at the documented initial configuration and fixed-seed velocity. Actual actuator-order limits are [500,500,400,400] Nm. The first attempt's NumPy conversion failure is retained locally as `osc_wsl_benchmark_20261023`; the corrected benchmark is `osc_wsl_benchmark_v2_20261023`.

The adapter provides executable multibody simulation; posture, contact behavior and solver status are evaluated below.

## Five-second follow-up

The run completes 5.0 s in 59.07 s wall time, advancing 2.5802 m. Minimum/final base heights are .7805/.7988 m; final pitch is .4628 rad. One of 10000 QP evaluations reaches the iteration limit at t=.9995 s, invoking the logged previous-torque fallback. Maximum explicit constraint violation across all evaluations is 2.71e-5; a small residual does not override a failed solver status. Torso tilt and a QP fallback remain the main observed issues in this finite run.

[State plots](figures/robotics/osc_wsl_states.png) and [actual-model kinematics animation](figures/robotics/osc_wsl.gif) are generated from saved simulation states. Curated environment/check/metric JSONs are in `docs/results/robotics/osc_wsl_*`; full states and every QP status remain in ignored `outputs/osc_wsl_5s_20261023`. Regenerate presentation assets with:

```powershell
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/build_wsl_osc.py
```

The optional Drake model test also independently compares the portable Acrobot dynamics against the supplied URDF at five fixed-seed states and torques. It does not rerun or reclassify the earlier infeasible optimization trials. On Windows without Drake this test is explicitly skipped.

## Expanded Drake verification

The checks use the same persistent WSL environment. The tests now independently check both feet, center-of-mass position and the scalar torso joint over three fixed-seed configurations and velocities. They verify the model's `qdot=v` mapping, velocity-coordinate Jacobians and bias accelerations with finite differences, in addition to the Acrobot model comparison. These pass in Drake 1.51.1. This extends the initial single stance-foot check without changing controller gains or planner equations.

The preserved completed HW4 code has also now been executed directly with Drake. For the same notebook waypoints and four 1 s segments, its solver reports `kSolutionFound`; integrated squared snap is 210.6953646030 versus 210.6953642384 for the scaled solver. Maximum sampled position difference is 1.42e-8 m; original waypoint residual is 1.42e-8 m. The original implementation is therefore verified for this configuration, rather than only classified as completed from inspection. One-sided continuity checks at +/-1e-9 s give 3.37e-8 maximum difference across derivative orders 1-4 (their respective units). This is a sampled numerical check, not a blanket conditioning guarantee.

```powershell
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/verify_drake_minsnap.py --output 'outputs/my_drake_minsnap_verification'
```

The saved comparison is `docs/results/robotics/drake_minsnap_verification.json`. Original course files and all earlier numerical runs remain unchanged. The missing original HW2 trajectory module remains missing; a working Drake installation does not resolve missing source material.

## Ten-second contact-switching verification

The unchanged controller/planner settings were run for 10 s in the same persistent environment. Additional logging records applied torques, stance-point position/velocity and candidate QP contact forces; the earlier five-second experiment is preserved.

| Diagnostic | Measured result |
|---|---:|
| Completed simulation / wall time | 10 s / 128.51 s |
| Forward displacement | 5.1527 m |
| Forward velocity RMSE relative to .5 m/s | .1013 m/s |
| Planned stance transitions | 49 |
| Minimum / final base height | .7805 / .8011 m |
| Maximum absolute / final base pitch | .5381 / .4948 rad |
| QP fallback events | 1, at .9995 s |
| Median / p95 QP execution time | 5.055 / 7.808 ms |
| Maximum accepted constraint residual | 2.71e-5 |
| Maximum applied actuator-bound overshoot | 1.28e-5 Nm |
| Stance-point velocity RMS | .4544 m/s |

The numerical bound overshoot is explicitly retained rather than rounded to zero; QP solutions are accepted within a 1e-4 residual tolerance, and this adapter does not apply a post-solve clipping guard. Peak torques reach the 500/400 Nm URDF limits. Limit events are brief: per-actuator fractions within 1e-4 Nm of a limit are .00160, .00170, .00100 and .00095, respectively. This is not sustained saturation. The residual combines constraint rows with their own units; it is a numerical diagnostic, not a physical accuracy measure.

Accepted QP contact forces satisfy the candidate linear friction cone to the reported numerical tolerance. They are **predicted optimization variables**, not independently measured plant contact forces. Stance-point velocity includes contact transitions; its nonzero value is not a certified physical slip estimate. Torso lean and unverified contact-force agreement remain limitations; disturbance and mismatch studies have not been performed. Offline computation is slower than simulated time.

[Ten-second state plots](figures/robotics/osc_wsl_10s_states.png) · [Model-kinematics animation](figures/robotics/osc_wsl_10s.gif) · [Numerical diagnostics](results/robotics/osc_wsl_10s_diagnostics.json)

```powershell
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/run_wsl_osc.py --duration 10 --output 'outputs/my_wsl_osc_10s'
.\.venv\Scripts\python.exe scripts/summarize_wsl_osc.py 'outputs/my_wsl_osc_10s' --output 'outputs/my_wsl_osc_10s_diagnostics.json'
```

The summary script uses NumPy only and can instead run with the WSL interpreter. Figure regeneration for the recorded run uses `scripts/build_wsl_osc.py --input outputs/osc_wsl_10s_verified_20261023 --prefix osc_wsl_10s` under WSL. The ten-second run provides multibody and contact-switching diagnostics under the stated settings.
