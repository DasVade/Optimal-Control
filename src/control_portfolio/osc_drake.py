"""New headless HW6 adapter. Original model, tracking framework and planner reused.

Requires the persistent WSL Drake environment. No original course files edited.
"""
import importlib
import os
from pathlib import Path
import sys
import time
import traceback
import numpy as np
from .osc import solve_qp


def run(folder,duration=.2):
    from pydrake.all import (MultibodyPlant,Parser,RigidTransform,LeafSystem,
        AbstractValue,PiecewisePolynomial,DiagramBuilder,AddMultibodyPlantSceneGraph,
        HalfSpace,CoulombFriction,ConstantVectorSource,ConstantValueSource,Simulator,
        JacobianWrtVariable,JointActuatorIndex,EventStatus)
    from .robotics_experiments import save
    source=Path(__file__).resolve().parents[2]/'course_sources/hw6'
    sys.path.insert(0,str(source))
    tracking=importlib.import_module('osc_tracking_objective')
    planner_module=importlib.import_module('footstep_planner')
    fsm_module=importlib.import_module('fsm_utils')

    class JointObjective(tracking.JointAngleTrackingObjective):
        def CalcJ(self):
            result=np.zeros((1,self.plant.num_velocities()));result[0,self.joint_vel_idx]=1
            return result
        def CalcJdotV(self):return np.zeros(1)

    class Controller(LeafSystem):
        def __init__(self):
            super().__init__();self.plant=MultibodyPlant(0.)
            Parser(self.plant).AddModels(str(source/'planar_walker.urdf'))
            self.plant.WeldFrames(self.plant.world_frame(),self.plant.GetBodyByName('base').body_frame(),RigidTransform.Identity())
            self.plant.Finalize();self.pc=self.plant.CreateDefaultContext()
            self.state_port=self.DeclareVectorInputPort('state',14)
            self.ports={name:self.DeclareAbstractInputPort(name,AbstractValue.Make(PiecewisePolynomial())) for name in ['com','swing','base']}
            self.DeclareVectorOutputPort('torque',4,self.calc)
            self.contacts={i:tracking.PointOnFrame(self.plant.GetBodyByName(name+'_lower_leg').body_frame(),np.array([0.,0.,-.5])) for i,name in enumerate(['left','right'])}
            kp=np.diag([100.,0.,100.]);kd=np.diag([10.,0.,10.])
            self.objectives=[tracking.CenterOfMassPositionTrackingObjective(self.plant,self.pc,[0,1],kp,kd),
                tracking.PointPositionTrackingObjective(self.plant,self.pc,[0,1],kp,kd,{0:self.contacts[1],1:self.contacts[0]}),
                JointObjective(self.plant,self.pc,[0,1],np.eye(1),np.eye(1),'planar_roty')]
            self.weights=[np.diag([0.,0.,1.]),np.diag([1.,0.,1.]),np.eye(1)]
            self.limits=np.array([self.plant.get_joint_actuator(JointActuatorIndex(i)).effort_limit() for i in range(4)])
            self.logs=[];self.previous=np.zeros(4)

        def contact(self,fsm):
            pt=self.contacts[fsm];world=self.plant.world_frame()
            j=self.plant.CalcJacobianTranslationalVelocity(self.pc,JacobianWrtVariable.kV,pt.frame,pt.pt,world,world)
            bias=self.plant.CalcBiasTranslationalAcceleration(self.pc,JacobianWrtVariable.kV,pt.frame,pt.pt,world,world).ravel()
            return j,bias

        def calc(self,context,output):
            state=self.state_port.Eval(context);self.plant.SetPositionsAndVelocities(self.pc,state)
            t=context.get_time();fsm=fsm_module.get_fsm(t);tasks=[]
            for name,obj,weight in zip(['com','swing','base'],self.objectives,self.weights):
                obj.Update(t,self.ports[name].Eval(context),fsm)
                tasks.append((obj.J,obj.JdotV,obj.yddot_cmd,weight))
            j,bias=self.contact(fsm);start=time.perf_counter()
            solution,record=solve_qp(self.plant.CalcMassMatrix(self.pc),self.plant.CalcBiasTerm(self.pc),
                -self.plant.CalcGravityGeneralizedForces(self.pc),self.plant.MakeActuationMatrix(),j,bias,tasks,self.limits)
            ok=record['success'] and solution is not None and record['constraint_violation']<1e-4
            if ok:self.previous=solution[7:11]
            record.update(time_s=t,fsm=int(fsm),fallback=not ok,solve_seconds=time.perf_counter()-start)
            point=self.contacts[fsm];world=self.plant.world_frame()
            position=self.plant.CalcPointsPositions(self.pc,point.frame,point.pt,world).ravel()
            record.update(applied_torque_Nm=self.previous.tolist(),stance_position_m=position.tolist(),
                          stance_velocity_m_s=(j@state[7:]).tolist())
            if solution is not None:
                record['candidate_contact_force_N']=solution[-3:].tolist()
            self.logs.append(record);output.SetFromVector(self.previous)

    class Planner(planner_module.LipTrajPlanner):
        def DiscreteStateUpdate(self,context,discrete_state):
            # Original logic, with explicit scalar extraction for NumPy 2 compatibility.
            fsm=fsm_module.get_fsm(context.get_time())
            previous=int(context.get_discrete_state(self.prev_fsm_state_idx).get_value().item())
            if fsm!=previous:
                state=self.EvalVectorInput(context,self.robot_state_input_port_index).value()
                self.plant.SetPositionsAndVelocities(self.plant_context,state)
                point=self.swing_foot_points[fsm]
                position=self.plant.CalcPointsPositions(self.plant_context,point.frame,point.pt,self.plant.world_frame()).ravel()
                discrete_state.get_mutable_vector(self.foot_position_at_liftoff_idx).set_value(position)
            discrete_state.get_mutable_vector(self.prev_fsm_state_idx).set_value([fsm])
            return EventStatus.Succeeded()

    # The supplied planner uses relative model paths. Restrict cwd change to its construction.
    controller=Controller();old=os.getcwd()
    try:
        os.chdir(source);planner=Planner()
    finally:os.chdir(old)
    # The supplied method is not registered as an event in the constructor.
    planner.DeclarePeriodicDiscreteUpdateEvent(.001,0.,planner.DiscreteStateUpdate)
    builder=DiagramBuilder();plant,scene=AddMultibodyPlantSceneGraph(builder,.0005)
    plant.RegisterCollisionGeometry(plant.world_body(),HalfSpace.MakePose(np.array([0.,0.,1.]),np.zeros(3)),
        HalfSpace(),'ground',CoulombFriction(1.,1.))
    Parser(plant).AddModels(str(source/'planar_walker.urdf'))
    plant.WeldFrames(plant.world_frame(),plant.GetBodyByName('base').body_frame(),RigidTransform.Identity());plant.Finalize()
    builder.AddSystem(controller);builder.AddSystem(planner)
    speed=builder.AddSystem(ConstantVectorSource([.5]))
    base=builder.AddSystem(ConstantValueSource(AbstractValue.Make(PiecewisePolynomial(np.zeros(1)))))
    builder.Connect(plant.get_state_output_port(),controller.state_port)
    builder.Connect(plant.get_state_output_port(),planner.get_state_input_port())
    builder.Connect(speed.get_output_port(),planner.get_walking_speed_input_port())
    builder.Connect(planner.get_com_traj_output_port(),controller.ports['com'])
    builder.Connect(planner.get_swing_foot_traj_output_port(),controller.ports['swing'])
    builder.Connect(base.get_output_port(),controller.ports['base'])
    builder.Connect(controller.get_output_port(),plant.get_actuation_input_port())
    diagram=builder.Build();sim=Simulator(diagram);context=sim.get_mutable_context()
    pc=diagram.GetMutableSubsystemContext(plant,context)
    q=np.zeros(7);q[1]=.8;angle=-np.arccos(.8);q[3:]=[angle,-2*angle,angle,-2*angle]
    plant.SetPositions(pc,q)
    # Initialize the supplied planner's swing-foot memory from actual kinematics.
    planner_context=diagram.GetMutableSubsystemContext(planner,context)
    controller.plant.SetPositions(controller.pc,q)
    pt=controller.contacts[1]
    swing=controller.plant.CalcPointsPositions(controller.pc,pt.frame,pt.pt,controller.plant.world_frame()).ravel()
    planner_context.get_mutable_discrete_state(planner.foot_position_at_liftoff_idx).set_value(swing)
    # Check contact Jacobian and directional bias against independent finite differences.
    rng=np.random.default_rng(0);velocity=rng.normal(0,.2,7)
    controller.plant.SetVelocities(controller.pc,velocity);j,bias=controller.contact(0);epsilon=1e-5
    def position(configuration):
        controller.plant.SetPositions(controller.pc,configuration)
        pt=controller.contacts[0]
        return controller.plant.CalcPointsPositions(controller.pc,pt.frame,pt.pt,controller.plant.world_frame()).ravel()
    numeric=np.column_stack([(position(q+epsilon*e)-position(q-epsilon*e))/(2*epsilon) for e in np.eye(7)])
    numeric_bias=(position(q+epsilon*velocity)-2*position(q)+position(q-epsilon*velocity))/epsilon**2
    validation=dict(contact_jacobian_max_error=float(np.max(np.abs(j-numeric))),
        contact_bias_max_error=float(np.max(np.abs(bias-numeric_bias))),actuator_limits_Nm=controller.limits.tolist(),
        dimensions=dict(positions=plant.num_positions(),velocities=plant.num_velocities(),actuators=plant.num_actuators()))
    save(folder/'multibody_checks.json',validation)
    controller.plant.SetPositionsAndVelocities(controller.pc,np.r_[q,np.zeros(7)])
    sim.Initialize();states=[np.r_[q,np.zeros(7)]];times=[0.];start=time.perf_counter();error=None
    try:
        for t in np.arange(.01,duration+.00001,.01):
            sim.AdvanceTo(float(t));states.append(plant.GetPositionsAndVelocities(pc));times.append(float(t))
    except Exception as exception:
        error=f'{type(exception).__name__}: {exception}'
        (folder/'exception.txt').write_text(traceback.format_exc())
    states=np.array(states)
    np.savez_compressed(folder/'trajectory.npz',time=np.array(times),state=states)
    save(folder/'solver_log.json',controller.logs)
    metrics=dict(duration_requested_s=duration,duration_completed_s=times[-1],error=error,
        wall_seconds=time.perf_counter()-start,solver_evaluations=len(controller.logs),
        solver_failures=sum(record['fallback'] for record in controller.logs),
        minimum_base_height_m=float(states[:,1].min()),final_base_height_m=float(states[-1,1]),
        final_base_pitch_rad=float(states[-1,2]),forward_distance_m=float(states[-1,0]-states[0,0]),
        demonstrated_walking=False,note='Execution alone does not establish stable walking; inspect duration, posture, contacts and solver failures.')
    if controller.logs:
        torques=np.array([record['applied_torque_Nm'] for record in controller.logs])
        velocities=np.array([record['stance_velocity_m_s'] for record in controller.logs])
        metrics.update(peak_applied_torque_Nm=np.max(np.abs(torques),axis=0).tolist(),
            max_applied_actuator_violation_Nm=float(max(0.,np.max(np.abs(torques)-controller.limits))),
            stance_velocity_rms_m_s=float(np.sqrt(np.mean(np.sum(velocities**2,axis=1)))),
            max_abs_base_pitch_rad=float(np.max(np.abs(states[:,2]))),
            planner_stance_transitions=int(sum(a['fsm']!=b['fsm'] for a,b in zip(controller.logs[:-1],controller.logs[1:]))))
    save(folder/'simulation_metrics.json',metrics);print(metrics,flush=True)
    return [dict(validation=validation,metrics=metrics)]
