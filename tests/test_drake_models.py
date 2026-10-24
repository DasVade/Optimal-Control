"""Optional independent multibody checks; skipped outside a Drake environment."""
from pathlib import Path
import numpy as np
import pytest
drake=pytest.importorskip('pydrake.all')
from control_portfolio.acrobot import dynamics


def test_portable_acrobot_matches_supplied_urdf():
    plant=drake.MultibodyPlant(0.)
    path=Path(__file__).resolve().parents[1]/'course_sources/hw5/acrobot.urdf'
    drake.Parser(plant).AddModels(str(path));plant.Finalize()
    context=plant.CreateDefaultContext();rng=np.random.default_rng(0)
    assert plant.num_actuators()==1
    for _ in range(5):
        state=rng.normal(0,.4,4);torque=float(rng.uniform(-10,10))
        plant.SetPositionsAndVelocities(context,state)
        acceleration=np.linalg.solve(plant.CalcMassMatrix(context),
            plant.MakeActuationMatrix()@np.array([torque])+plant.CalcGravityGeneralizedForces(context)-plant.CalcBiasTerm(context))
        np.testing.assert_allclose(dynamics(state,torque)[2:],acceleration,rtol=1e-11,atol=1e-11)


def test_biped_task_jacobians_and_biases():
    plant=drake.MultibodyPlant(0.)
    source=Path(__file__).resolve().parents[1]/'course_sources/hw6'
    drake.Parser(plant).AddModels(str(source/'planar_walker.urdf'))
    plant.WeldFrames(plant.world_frame(),plant.GetBodyByName('base').body_frame(),drake.RigidTransform.Identity())
    plant.Finalize();context=plant.CreateDefaultContext();world=plant.world_frame()
    rng=np.random.default_rng(42)
    for _ in range(3):
        q=rng.normal(0,.3,7);q[1]=.8;v=rng.normal(0,.2,7)
        plant.SetPositionsAndVelocities(context,np.r_[q,v])
        # This model has scalar planar joints: qdot=v, checked against Drake's mapping.
        np.testing.assert_allclose(plant.MapVelocityToQDot(context,v),v,atol=1e-12)
        for name in ['left_lower_leg','right_lower_leg','com','joint']:
            if name=='com':
                value=lambda:plant.CalcCenterOfMassPositionInWorld(context).ravel()
                j=plant.CalcJacobianCenterOfMassTranslationalVelocity(context,drake.JacobianWrtVariable.kV,world,world)
                bias=plant.CalcBiasCenterOfMassTranslationalAcceleration(context,drake.JacobianWrtVariable.kV,world,world).ravel()
            elif name=='joint':
                joint=plant.GetJointByName('planar_roty');index=joint.position_start()
                value=lambda index=index:plant.GetPositions(context)[index:index+1]
                j=np.zeros((1,7));j[0,joint.velocity_start()]=1.;bias=np.zeros(1)
            else:
                frame=plant.GetBodyByName(name).body_frame();point=np.array([0.,0.,-.5])
                value=lambda frame=frame:plant.CalcPointsPositions(context,frame,point,world).ravel()
                j=plant.CalcJacobianTranslationalVelocity(context,drake.JacobianWrtVariable.kV,frame,point,world,world)
                bias=plant.CalcBiasTranslationalAcceleration(context,drake.JacobianWrtVariable.kV,frame,point,world,world).ravel()
            def evaluate(configuration):
                plant.SetPositions(context,configuration)
                return value().copy()
            h=1e-4
            numeric=np.column_stack([(evaluate(q+h*e)-evaluate(q-h*e))/(2*h) for e in np.eye(7)])
            numeric_bias=(evaluate(q+h*v)-2*evaluate(q)+evaluate(q-h*v))/h**2
            np.testing.assert_allclose(j,numeric,atol=1e-7,rtol=1e-7)
            np.testing.assert_allclose(bias,numeric_bias,atol=1e-6,rtol=1e-5)
            plant.SetPositions(context,q)
