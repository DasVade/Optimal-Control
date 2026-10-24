# Reproduction

Run from the existing repository root on `codex/portfolio-update`. No remote operation, paid compute or external service is required after dependencies are installed. Python 3.12.14 on Windows is the measured environment; Linux/macOS use `.venv/bin/python` instead of the Windows executable.

## Environment

```powershell
Set-Location 'C:\Users\Lewis\Documents\New project\Optimal-Control'
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
```

If using the desktop app's bundled Python, the existing venv was created without its own pip. The verified bootstrap command for that environment is:

```powershell
& 'C:\Users\Lewis\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m pip --python .venv install -r requirements-lock.txt
& 'C:\Users\Lewis\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m pip --python .venv install --no-deps -e .
```

The core experiments do not require Drake, notebook servers, video codecs, raw course PDFs or the original Deepnote environment. No instructions in the supplied initialization notebooks need to be executed. The OSC multibody simulation requires an additional working Drake environment and adapter verification; installing a dependency alone would not turn the original incomplete walking notebook into a demonstrated controller.

## Main results

```powershell
.\.venv\Scripts\python.exe -m control_portfolio.cli robotics-benchmark --output 'outputs/reproduce_benchmark'
.\.venv\Scripts\python.exe -m control_portfolio.cli quadrotor --config 'configs/robotics.json' --output 'outputs/reproduce_quadrotor'
.\.venv\Scripts\python.exe -m control_portfolio.cli minsnap --config 'configs/robotics.json' --output 'outputs/reproduce_minsnap'
.\.venv\Scripts\python.exe -m control_portfolio.cli ilqr-benchmark --output 'outputs/reproduce_ilqr_benchmark'
.\.venv\Scripts\python.exe -m control_portfolio.cli ilqr --output 'outputs/reproduce_ilqr'
.\.venv\Scripts\python.exe -m control_portfolio.cli acrobot-benchmark --output 'outputs/reproduce_acrobot_benchmark'
.\.venv\Scripts\python.exe -m control_portfolio.cli acrobot --output 'outputs/reproduce_acrobot'
.\.venv\Scripts\python.exe -m control_portfolio.cli acrobot --config 'configs/acrobot_folded.json' --output 'outputs/reproduce_acrobot_folded'
.\.venv\Scripts\python.exe -m control_portfolio.cli supporting --output 'outputs/reproduce_supporting'
.\.venv\Scripts\python.exe -m control_portfolio.cli osc --output 'outputs/reproduce_osc_status'
```

Every output directory must be new. Runtime is normally seconds for the main MPC/planning experiments and about two minutes for the reported iLQR sweep on the measured CPU, with solver-dependent variation. Acrobot's reported runs fail nonlinear feasibility; this is expected output for these configurations, not a successful swing-up. `osc` writes a blocked status when Drake is absent rather than pretending to simulate.

Each task uses a subsection of `configs/robotics.json`. Initial conditions, model parameters, costs, solver tolerances, mesh and evaluation definitions are also specified in the module docstrings and [report](ROBOTICS_REPORT.md). Algorithmic constants such as fixed Q/Qf and line-search schedules are explicit in source; only the documented JSON fields are configurable. Keep those constants fixed for matched comparisons.

## Existing HW3 experiments

```powershell
.\.venv\Scripts\python.exe -m control_portfolio.cli benchmark --config 'configs/experiments.json' --output 'outputs/reproduce_hw3_benchmark'
.\.venv\Scripts\python.exe -m control_portfolio.cli suite --config 'configs/experiments.json' --output 'outputs/reproduce_hw3'
```

Original verified HW3 measurements remain in `docs/results/results.json` and their original detailed report; new robotics results do not overwrite them. Original root course scripts still require their original dependencies and are reference baselines, not the preferred entry point.

## Tests, independent replay and figures

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

The tests check hover/linearization, bound offset and forced MPC fallback, evolving-state integration, minimum-snap equality/time scaling, matching discrete iLQR derivatives, Acrobot energy-power balance/equilibrium/defects, and OSC force signs. They deliberately do not assert that unsuccessful nonlinear optimizations are successes. Original incomplete `course_sources/hw6/osc.py` has a preserved syntax error and is excluded from runnable package tests.

`scripts/verify_saved_robotics.py` performs finer integration checks on the archived main run names and writes a **new** `outputs/verification_20261023` folder. It refuses an existing verification folder. Its recorded result is curated as `docs/results/robotics/verification.json`. The observed iLQR replay discrepancy remains a documented limitation.

To regenerate the exact published figures from local saved runs:

```powershell
.\.venv\Scripts\python.exe scripts/build_robotics.py
```

The figure script reads `outputs/{quadrotor,minsnap,ilqr,acrobot,acrobot_folded,supporting,osc,ilqr_benchmark,acrobot_benchmark}_20261023`. It does not simulate or overwrite experiment inputs/results; it rebuilds only curated presentation assets. These exact local folders exist in this checkout. On a fresh clone, use those names for new CLI runs, or change the single `sources` mapping in the figure script to your reproduction folders. `scripts/build_portfolio.py` separately rebuilds preserved HW3 figures from its original saved run. No smoothing or fictitious example data is used.

Repository-only summaries are sufficient to audit reported numbers. Full run CSV/NPZ traces and per-solve logs are local outputs rather than large versioned files; rerun commands to regenerate them. The initial sweeps recorded base commit ae7498a while source changes were uncommitted; their environment files are preserved honestly. The saved repeat benchmark records source hashes. Supplemental verification records the dependency versions; no reconstructed hash is mislabeled as an original run hash.
