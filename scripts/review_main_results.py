"""Independent audit of saved traces and the free-endpoint minimum-snap QP.

No controller tuning, simulation, or edits to prior experiment outputs.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.integrate import quad
from scipy.interpolate import PPoly
from scipy.linalg import block_diag,null_space

ROOT=Path(__file__).resolve().parents[1]


def snap_matrices(waypoints,durations,terms):
    """Independent coefficient differentiation using numpy.polynomial, not solver basis()."""
    n=len(durations);size=n*terms;rows=[];targets=[]
    def derivative_row(tau,order):
        return np.array([np.polynomial.polynomial.polyval(tau,
            np.polynomial.polynomial.polyder(np.eye(terms)[j],m=order)) for j in range(terms)])
    for i,T in enumerate(durations):
        for tau,w in [(0.,waypoints[i]),(1.,waypoints[i+1])]:
            row=np.zeros(size);row[i*terms:(i+1)*terms]=derivative_row(tau,0)
            rows.append(row);targets.append(w)
    for i in range(n-1):
        for k in range(1,5):
            row=np.zeros(size);row[i*terms:(i+1)*terms]=derivative_row(1.,k)/durations[i]**k
            row[(i+1)*terms:(i+2)*terms]=-derivative_row(0.,k)/durations[i+1]**k
            rows.append(row);targets.append(np.zeros(2))
    # Exact fourth-derivative coefficient operator. Its kernel is piecewise cubics.
    derivative=np.zeros((terms-4,terms))
    for j in range(4,terms):derivative[j-4,j]=math.prod(range(j-3,j+1))
    operator=block_diag(*[derivative/T**4 for T in durations])
    cubic_kernel=np.zeros((size,4*n))
    for i in range(n):cubic_kernel[i*terms:i*terms+4,4*i:4*i+4]=np.eye(4)
    return np.array(rows),np.array(targets),operator,cubic_kernel


def review_snap(run_root,config):
    cfg=config['minsnap'];w=np.array(cfg['waypoints'],float);n=len(w)-1;terms=cfg['terms']
    a,b,d,kernel=snap_matrices(w,np.ones(n),terms)
    row_norm=np.linalg.norm(a,axis=1);scaled=a/row_norm[:,None]
    restricted=scaled@kernel
    augmented=np.column_stack([restricted,b/row_norm[:,None]])
    cubic_fit=np.linalg.lstsq(restricted,b/row_norm[:,None],rcond=None)[0]
    times=np.arange(n+1,dtype=float);vander=np.vander(times,4,increasing=True)
    fitted=vander@np.linalg.lstsq(vander,w,rcond=None)[0]
    # Homogeneous equality directions have strictly positive snap iff this map is full column rank.
    feasible_directions=null_space(scaled)
    result=dict(coefficient_count_per_coordinate=n*terms,equality_count_per_coordinate=len(a),
        equality_rank=int(np.linalg.matrix_rank(scaled)),equality_nullity=feasible_directions.shape[1],
        objective_rank=int(np.linalg.matrix_rank(d)),objective_nullity=kernel.shape[1],
        equality_on_objective_kernel_rank=int(np.linalg.matrix_rank(restricted)),
        equality_on_kernel_augmented_rank=int(np.linalg.matrix_rank(augmented)),
        objective_on_equality_nullspace_rank=int(np.linalg.matrix_rank(d@feasible_directions)),
        best_zero_snap_scaled_constraint_residual=float(np.max(np.abs(restricted@cubic_fit-b/row_norm[:,None]))),
        global_cubic_vandermonde_rank=int(np.linalg.matrix_rank(vander)),
        global_cubic_augmented_rank=int(np.linalg.matrix_rank(np.column_stack([vander,w]))),
        best_global_cubic_waypoint_max_residual_m=float(np.max(np.abs(fitted-w))),
        fourth_waypoint_differences=np.diff(w,n=4,axis=0).tolist(),duration_cases=[])
    baseline=None;costs={}
    saved=json.loads((run_root/'minsnap_20261023/summary.json').read_text())
    for scale in cfg['duration_scales']:
        trace=np.load(run_root/f'minsnap_20261023/duration{scale}/trajectory.npz')
        durations=np.full(n,scale);coefficients=trace['coefficients'];knots=np.r_[0,np.cumsum(durations)]
        # Convert ascending normalized coefficients into physical local-time PPoly coefficients.
        physical=coefficients/durations[:,None,None]**np.arange(terms)[None,:,None]
        curve=PPoly(np.moveaxis(physical,0,1)[::-1],knots);fourth=curve.derivative(4)
        integrals=[quad(lambda t:float(fourth(t)@fourth(t)),knots[i],knots[i+1],epsabs=1e-9,epsrel=1e-11) for i in range(n)]
        independent=sum(value for value,error in integrals);costs[scale]=independent
        aa,bb,_,_=snap_matrices(w,durations,terms)
        residual=aa@coefficients.reshape(n*terms,2)-bb
        points=curve(np.linspace(0,knots[-1],1001))
        if baseline is None:baseline=points
        old=next(r['metrics']['snap_integral'] for r in saved['results'] if r['settings']['duration_scale']==scale)
        result['duration_cases'].append(dict(segment_duration_s=scale,
            saved_objective=old,independent_adaptive_integral=independent,
            independent_integration_error_estimate=sum(error for value,error in integrals),
            objective_relative_difference=abs(independent-old)/max(1.,abs(old)),
            waypoint_residual_m=float(np.max(np.abs(residual[:2*n]))),
            derivative_continuity_max_residual=float(np.max(np.abs(residual[2*n:]))),
            maximum_geometric_path_difference_m=float(np.max(np.abs(points-baseline)))))
    result['duration_doubling_objective_ratio']=costs[2.]/costs[1.]
    result['expected_duration_doubling_ratio']=2.**-7
    result['global_cubic_possible']=bool(result['global_cubic_vandermonde_rank']==result['global_cubic_augmented_rank'])
    return result


def review_quad(run_root,config):
    root=run_root/'quadrotor_20261023';summary=json.loads((root/'summary.json').read_text());rows=summary['results']
    audited=[]
    for row in rows:
        trace=np.load(root/row['name']/'trajectory.npz');dt=np.diff(trace['time'])
        effort=float(np.sum(dt*np.sum((trace['applied']-np.array([4.905,4.905]))**2,axis=1)))
        rmse=float(np.sqrt(np.mean(np.sum(trace['state'][:,:2]**2,axis=1))))
        audited.append(dict(name=row['name'],effort_N2s=effort,position_rmse_m=rmse,
            effort_saved_difference=effort-row['metrics']['control_effort_above_hover_N2s'],
            rmse_saved_difference=rmse-row['metrics']['position_rmse_m'],
            initial_matches=bool(np.array_equal(trace['state'][0],row['settings']['initial'])),
            duration_s=float(trace['time'][-1]),control_update_s=float(dt[0]),
            configured_duration_matches=bool(np.isclose(trace['time'][-1],config['quadrotor']['duration_s'])),
            configured_update_grid_matches=bool(np.allclose(dt,config['quadrotor']['control_dt_s'],atol=1e-12,rtol=0)),
            applied_bound_violation_N=float(max(0.,-trace['applied'].min(),trace['applied'].max()-5.5))))
    by_name={r['name']:r for r in rows};pairs=[]
    for seed in config['quadrotor']['seeds']:
        for group in ['disturb','mismatch']:
            lqr=by_name[f'lqr_{group}_seed{seed}']['settings'];mpc=by_name[f'mpc_{group}_seed{seed}']['settings']
            pairs.append(dict(group=group,seed=seed,matched=all(lqr[k]==mpc[k] for k in ['initial','mass','disturbance','seed']),disturbance_N=lqr['disturbance']))
    lookup={r['name']:r for r in audited};lqr=lookup['lqr_ic1'];mpc=lookup['mpc_n10_ic1']
    return dict(recomputed_cases=audited,matched_disturbance_and_mismatch_pairs=pairs,
        effort_reduction_percent=100*(1-mpc['effort_N2s']/lqr['effort_N2s']),
        position_rmse_increase_percent=100*(mpc['position_rmse_m']/lqr['position_rmse_m']-1),
        effort_definition='Integral ||u_applied(t)-[4.905,4.905]||_2^2 dt; exact held-input interval sum.',
        cost_convention='LQR: continuous infinite-horizon Q/R. MPC: unscaled discrete stage Q/R, terminal Q; at T=.1 equivalent rescaled terminal is .1Q in a rectangle-rule continuous approximation, not Q or CARE S.',
        initial_nominal_pair_matches=by_name['lqr_ic1']['settings']['initial']==by_name['mpc_n10_ic1']['settings']['initial'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--runs',type=Path,default=ROOT/'outputs')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    quad_config=json.loads((args.runs/'quadrotor_20261023/config.json').read_text())
    snap_config=json.loads((args.runs/'minsnap_20261023/config.json').read_text())
    result=dict(quadrotor=review_quad(args.runs,quad_config),minimum_snap=review_snap(args.runs,snap_config),
                note='New audit artifact. Original experiment results and controller/trajectory settings unchanged.')
    assert all(abs(row['effort_saved_difference'])<1e-10 and abs(row['rmse_saved_difference'])<1e-12 and row['initial_matches'] and row['configured_duration_matches'] and row['configured_update_grid_matches'] for row in result['quadrotor']['recomputed_cases'])
    assert all(row['matched'] for row in result['quadrotor']['matched_disturbance_and_mismatch_pairs'])
    assert all(row['objective_relative_difference']<1e-10 for row in result['minimum_snap']['duration_cases'])
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(dict(quadrotor_percentages={k:result['quadrotor'][k] for k in ['effort_reduction_percent','position_rmse_increase_percent']},minimum_snap=result['minimum_snap']),indent=2))


if __name__=='__main__':main()
