# Persistent WSL environment and Drake follow-up

The earlier environment blocker is resolved. Ubuntu-22.04 WSL already existed. Existing Conda environments `hw2` (Python 3.11), `cv`, `apriltag`, and `videomem` were checked; none contained Drake or OSQP. The `hw2` interpreter was reused to create an isolated Python 3.11 virtual environment. Existing Conda packages were not changed.

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

This resolves dependency access and demonstrates actual multibody execution. A short run, matrix residual, or nonzero forward displacement alone does not establish stable walking. Inspect the longer-run metrics, posture, solver failures and contact behavior before making that claim. The original Windows-only blocked status and historical report remain dated evidence; the current environment is described here.

## Five-second follow-up

The run completes 5.0 s in 59.07 s wall time, advancing 2.5802 m. Minimum/final base heights are .7805/.7988 m; final pitch is .4628 rad. One of 10000 QP evaluations reaches the iteration limit at t=.9995 s, invoking the logged previous-torque fallback. Maximum explicit constraint violation across all evaluations is 2.71e-5; a small residual does not override a failed solver status. No stable-walking or real-time claim follows from this finite trace, especially given the torso tilt and unverified long-duration/contact robustness.

[State plots](figures/robotics/osc_wsl_states.png) and [actual-model kinematics animation](figures/robotics/osc_wsl.gif) are generated from saved simulation states, not an invented motion. Curated environment/check/metric JSONs are in `docs/results/robotics/osc_wsl_*`; full states and every QP status remain in ignored `outputs/osc_wsl_5s_20261023`. Regenerate presentation assets with:

```powershell
wsl -d Ubuntu-22.04 --exec /home/yxlewis/.venvs/optimal-control/bin/python scripts/build_wsl_osc.py
```

The optional Drake model test also independently compares the portable Acrobot dynamics against the supplied URDF at five fixed-seed states and torques. It does not rerun or reclassify the earlier infeasible optimization trials. On Windows without Drake this test is explicitly skipped.
