# Reproduction

Python 3.12.14 on Windows is the measured core environment. Linux/macOS use
`.venv/bin/python` instead of the Windows executable. The Drake multibody
workflow uses a separate persistent [WSL environment](WSL_SETUP.md).

## Environment

```powershell
Set-Location 'C:\Users\Lewis\Documents\New project\Optimal-Control'
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
```

The core experiments use the packages in `requirements-lock.txt`.
For OSC multibody simulation, follow [WSL_SETUP.md](WSL_SETUP.md).

## Main results

For the focused portfolio results and plots, run from the repository root in the prepared Windows environment:

```powershell
.\.venv\Scripts\python.exe -m control_portfolio.cli quadrotor --config 'configs/robotics.json' --output 'outputs/review_quadrotor'
.\.venv\Scripts\python.exe -m control_portfolio.cli minsnap --config 'configs/robotics.json' --output 'outputs/review_minsnap'
.\.venv\Scripts\python.exe scripts/build_robotics.py --main-only --quadrotor-run 'outputs/review_quadrotor' --minsnap-run 'outputs/review_minsnap' --figure-output 'outputs/review_figures'
Invoke-Item -LiteralPath 'outputs/review_figures/quadrotor_comparison.png'
Invoke-Item -LiteralPath 'outputs/review_figures/minimum_snap.png'
Invoke-Item -LiteralPath 'outputs/review_figures/quadrotor.gif'
```

Choose fresh run directory names if these already exist. `--main-only` reads numerical traces and writes just the two main figures and quadrotor animation into the requested figure folder; it does not curate or overwrite historical metrics. Only the reviewed effort-axis wording changed in the tracked quadrotor figure; plotted numerical data are unchanged.

The independent saved-result audit can be reproduced with:

```powershell
.\.venv\Scripts\python.exe scripts/review_main_results.py --output 'outputs/my_technical_review.json'
```

It reads the original `quadrotor_20261023` / `minsnap_20261023` run folders and refuses an existing output file. A fresh clone can rerun the CLI using those archived names to regenerate the inputs, or set `--runs` to an alternate parent containing the same two folder names. Its small curated output is [technical_review.json](results/robotics/technical_review.json); no previous run is modified. The report details the continuous/discrete cost limitation and the cubic/nullspace proof.

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

Every output directory must be new. Runtime is normally seconds for the main MPC/planning experiments and about two minutes for the reported iLQR sweep on the measured CPU, with solver-dependent variation. Acrobot's reported runs fail nonlinear feasibility; this is expected output for these configurations, not a successful swing-up. `osc` writes a blocked status when Drake is absent with the missing dependency identified.

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

The tests check hover/linearization, bound offset and forced MPC fallback, evolving-state integration, minimum-snap equality/time scaling, matching discrete iLQR derivatives, Acrobot energy-power balance/equilibrium/defects, and OSC force signs. Original incomplete `course_sources/hw6/osc.py` has a preserved syntax error and is excluded from runnable package tests.

`scripts/verify_saved_robotics.py` performs finer integration checks on the archived main run names and writes a **new** `outputs/verification_20261023` folder. It refuses an existing verification folder. Its recorded result is curated as `docs/results/robotics/verification.json`. The observed iLQR replay discrepancy remains a documented limitation.

To regenerate the exact published figures from local saved runs:

```powershell
.\.venv\Scripts\python.exe scripts/build_robotics.py
```

The figure script reads `outputs/{quadrotor,minsnap,ilqr,acrobot,acrobot_folded,supporting,osc,ilqr_benchmark,acrobot_benchmark}_20261023`. It does not simulate or overwrite experiment inputs/results; it rebuilds only curated presentation assets. These exact local folders exist in this checkout. On a fresh clone, use those names for new CLI runs, or change the single `sources` mapping in the figure script to your reproduction folders. `scripts/build_portfolio.py` separately rebuilds preserved HW3 figures from its original saved run.

Repository-only summaries are sufficient to audit reported numbers. Full run CSV/NPZ traces and per-solve logs are local outputs rather than large versioned files; rerun commands to regenerate them. The initial sweeps record base commit ae7498a with uncommitted source changes. The repeat benchmark includes source hashes; supplemental verification records dependency versions.
