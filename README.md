# Nonlinear Control and Flatness-Based Motion Planning

Reproducible extensions of existing optimal-control coursework: nonlinear cart-pole energy shaping and local LQR, plus unicycle spline planning and flatness-based feedforward inputs. The focus is mathematical verification, matched experiments, and visible failure cases—not additional controllers.

![Cart-pole regulation under matched cost weights](docs/figures/lqr_weights.png)

## Verified results

60 cart-pole simulations and 15 unicycle simulations; six mathematical/behavioral tests pass. For a matched 20 s cart-pole rollout from `[0.1, pi+0.1, 0, 0]`:

| Input weight R | Cart RMSE | Settling time | Applied effort |
|---|---:|---:|---:|
| 0.1 | 0.0647 m | 3.64 s | 2.895 N²s |
| 1 | 0.1142 m | 5.65 s | 2.405 N²s |
| 10 | 0.2505 m | 10.11 s | 2.112 N²s |

Lower R trades faster settling for larger nominal control effort. Settling requires all four state tolerances to hold for the rest of the rollout; see the report for the exact definition. The 2 N clipped LQR fails from the larger tested initial offset. Five-seed unicycle tests show approximately 0.5 m endpoint error after a 0.05 rad heading offset or +5% speed mismatch, exposing the lack of feedback correction.

![Cart-pole swing-up simulation](docs/figures/swingup.gif)

20 simulated seconds shown in accelerated playback. The cart-travel markers are diagnostics, not enforced rail constraints.

## Methods and boundaries

- Continuous-time LQR from the nonlinear model's upright Jacobian; no finite-horizon method or MPC.
- Inherited energy shaping, partial feedback linearization and local LQR switching, with wrapped angle error and explicit force clipping. **Clipping is not constrained optimization.**
- Original obstacle-dependent cubic-spline recipe and flatness inversion; feedforward only, not numerical trajectory optimization.
- Paired Q/R, initial-state, actuator-limit, disturbance and parameter-mismatch experiments; five fixed seeds for stochastic trials.
- Saved configuration/NPZ traces/JSON metrics/CSV, fixed-step RK4 and independently checked integration, and an English failure report.

## Quick start

Python 3.12 is verified. PowerShell from the repository root:

```powershell
python -m venv '.venv'
& '.\.venv\Scripts\python.exe' -m pip install -e '.[dev]'
& '.\.venv\Scripts\python.exe' -m pytest -q
& '.\.venv\Scripts\python.exe' -m control_portfolio.cli benchmark --output 'outputs/my_benchmark'
& '.\.venv\Scripts\python.exe' -m control_portfolio.cli suite --output 'outputs/my_run'
& '.\.venv\Scripts\python.exe' 'scripts/build_portfolio.py' --run 'outputs/my_run'
```

Use a new output path for each simulation run; existing directories are refused. Configuration: [configs/experiments.json](configs/experiments.json). The complete verified suite took about 30.34 s on the local CPU, excluding figure generation; this is not a real-time hardware claim. Exact versions: [requirements-lock.txt](requirements-lock.txt). No paid compute is needed.

## Source and contribution boundaries

Original Python files, notebooks and historical images remain unchanged at their original paths and in commit `3efbb4c`. The supplied work contains completed TODO regions within mixed framework code; a pristine starter and full assignment handout are missing, so individual authorship cannot be assigned line by line. New modules/tests/experiments/reporting were developed with coding-assistant support. The reproducible path uses SciPy CARE instead of requiring Drake; the original Drake execution remains unverified.

**Resume correction:** LQR is present, with an original linearization issue corrected here. MPC, finite-horizon comparisons and numerical trajectory optimization are not supported by the supplied repository. Do not claim them on the basis of this project.

- [Technical report: equations, configurations, controlled results and failures](docs/TECHNICAL_REPORT.md)
- [Source audit and provenance](docs/AUDIT_AND_PROVENANCE.md)
- [Full numeric results](docs/results/results.json) and [CSV](docs/results/cartpole_metrics.csv)
- [Original source integrity manifest](docs/source_manifest.json)
- [Two evidence-based resume bullets](docs/RESUME_BULLETS.md)

The existing repository remains private. This update is prepared locally on `codex/portfolio-update`; no remote repository creation, push or public deployment is part of this work.
