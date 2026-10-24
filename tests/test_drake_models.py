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
