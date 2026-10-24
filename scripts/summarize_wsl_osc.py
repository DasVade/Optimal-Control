"""Summarize saved OSC diagnostics without rerunning or editing the experiment."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
if args.output.exists():raise FileExistsError(args.output)
logs=json.loads((args.input/'solver_log.json').read_text())
run=np.load(args.input/'trajectory.npz')
times=np.array([record['solve_seconds'] for record in logs]);accepted=[r for r in logs if not r['fallback']]
force=np.array([r['candidate_contact_force_N'] for r in accepted])
summary=dict(metrics=json.loads((args.input/'simulation_metrics.json').read_text()),
    solver_status_counts=dict(Counter(r['status'] for r in logs)),
    solver_time_median_s=float(np.median(times)),solver_time_p95_s=float(np.quantile(times,.95)),
    accepted_max_constraint_violation=float(max(r['constraint_violation'] for r in accepted)),
    accepted_min_candidate_normal_force_N=float(force[:,2].min()),
    accepted_max_candidate_friction_violation_N=float(max(0.,np.max(np.abs(force[:,0])-force[:,2]))),
    candidate_force_note='These are QP contact-force predictions, not measured plant contact forces.',
    forward_velocity_rmse_m_s=float(np.sqrt(np.mean((run['state'][:,7]-.5)**2))),
    final_forward_velocity_m_s=float(run['state'][-1,7]),
    fallback_events=[r for r in logs if r['fallback']],
    simulation_settings=dict(plant_step_s=.0005,planner_update_s=.001,desired_speed_m_s=.5,
        com_height_target_m=.9,task_kp_planar=100.,task_kd_planar=10.,torso_kp=1.,torso_kd=1.,
        vdot_regularization=1e-5,friction_coefficient=1.,controller_gains_changed_since_5s_run=False))
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(summary,indent=2,allow_nan=False))
print(json.dumps({k:v for k,v in summary.items() if k!='fallback_events'},indent=2))
