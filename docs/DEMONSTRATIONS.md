# Demonstrations from saved results

`scripts/build_portfolio_demos.py` renders existing traces without running a
controller, optimizing a trajectory, or changing recorded experiment metrics.
Run `.\.venv\Scripts\python.exe scripts/build_portfolio_demos.py` from the root
after reproducing the source runs described in REPRODUCTION.md. The source-run
paths and SHA-256 hashes are recorded in `results/robotics/demo_verification.json`.

- `figures/robotics/quadrotor_lqr_mpc.gif`: clipped LQR and MPC N=10, `ic1`,
  identical sample clocks and initial states, common axes, 0–10 s. Every fifth
  saved 0.02 s sample is displayed at 10 fps. Neither result is sped up relative
  to the other. Both applied-input traces satisfy the recorded 0–5.5 N bounds.
- `figures/robotics/cartpole_hybrid.gif`: `swing_30deg`, initial
  `[0, pi/6, 0, 0]`, downward-zero angle, pole length 1 m. Saved states are
  inspected for the hybrid switching rule and all-state settling criteria.
  Sampled mode labels are recomputed from the unchanged controller, not saved
  RK-stage mode history. Full 20 s playback at 10 fps, every tenth 0.01 s sample.
  One energy-shaping-to-LQR transition; settling 9.84 s under bounds
  `[0.05 m, 0.05 rad, 0.1 m/s, 0.1 rad/s]`, remaining through the end with at
  least 1 s dwell. The cart exceeds the ±2 m reference bound during the
  transient: this is not a rail-constrained swing-up demonstration. Exact
  downward rest is a separate failure case; this example does not establish
  global convergence.
- `figures/robotics/minimum_snap_duration.gif`: preserved `duration1.0` and
  `duration2.0` plans, 4 s and 8 s, share their geometric path. Both markers
  use one physical clock at 10 fps. The 4 s marker stays at its endpoint after
  completion. Waypoint interpolation and the existing T^-7 objective result
  apply. These are planned point motions, not dynamically simulated flight.

Only selected newly generated visualizations and their factual summaries are
copied to the website. No course source, datasets, archives or solver code is
published there. The source repository is public following the owner's October 24, 2026 publication authorization. Optional unicycle and iLQR
results remain secondary; see ROBOTICS_REPORT.md and TECHNICAL_REPORT.md for
their status rather than interpreting optimization outcomes as deployment.
