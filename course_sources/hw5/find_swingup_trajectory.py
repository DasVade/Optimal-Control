import matplotlib.pyplot as plt
import numpy as np
import importlib

from pydrake.all import (
    DiagramBuilder,
    Simulator,
    FindResourceOrThrow,
    MultibodyPlant,
    PiecewisePolynomial,
    SceneGraph,
    Parser,
    JointActuatorIndex,
    MathematicalProgram,
    Solve,
)
from kinematic_constraints import AddBoxCollisionConstraints
from dynamics_constraints import AddCollocationConstraints, EvaluateDynamics


def find_swingup_trajectory(N, initial_state, final_configuration, box_centers, box_width, box_height):
    """
    Parameters:
      N - number of knot points
      initial_state - starting configuration
      distance - target distance to throw the ball

    """

    builder = DiagramBuilder()
    plant = builder.AddSystem(MultibodyPlant(0.0))
    file_name = "acrobot.urdf"
    Parser(plant=plant).AddModels(file_name)
    plant.Finalize()
    acrobot = plant.ToAutoDiffXd()

    plant_context = plant.CreateDefaultContext()
    context = acrobot.CreateDefaultContext()

    # Dimensions specific to the acrobot
    n_q = acrobot.num_positions()
    n_v = acrobot.num_velocities()
    n_x = n_q + n_v
    n_u = acrobot.num_actuators()

    # Store the actuator limits here
    effort_limits = np.zeros(n_u)
    for act_idx in range(n_u):
        effort_limits[act_idx] = acrobot.get_joint_actuator(
            JointActuatorIndex(act_idx)
        ).effort_limit()
    joint_limits = np.pi * np.ones(n_q)
    vel_limits = 15 * np.ones(n_v)

    # Create the mathematical program
    prog = MathematicalProgram()
    x = np.zeros((N, n_x), dtype="object")
    u = np.zeros((N, n_u), dtype="object")
    for i in range(N):
        x[i] = prog.NewContinuousVariables(n_x, "x_" + str(i))
        u[i] = prog.NewContinuousVariables(n_u, "u_" + str(i))

    t0 = 0.0
    dt = 0.35
    timesteps = np.linspace(t0, (N-1)*dt, N)
    x0 = x[0]
    xf = x[-1]

    #############################################################    
    # DO NOT MODIFY THE LINES ABOVE
    #############################################################

    # Add the kinematic constraints (initial state, final state)
    # TODO: Add constraints on the initial state
    
    # TODO: Add the collision constraints to every timestep, for every box

    # Add the collocation aka dynamics constraints
    AddCollocationConstraints(prog, acrobot, context, N, x, u, timesteps)

    # TODO: Add the cost function here

    # TODO: Add bounding box constraints on the inputs and qdot

    # TODO: give the solver an initial guess for x and u using prog.SetInitialGuess(var, value)


    #############################################################
    # DO NOT MODIFY THE LINES BELOW
    #############################################################

    # Set up solver
    result = Solve(prog)

    x_sol = result.GetSolution(x)
    u_sol = result.GetSolution(u)

    print("optimal cost: ", result.get_optimal_cost())
    print("x_sol: ", x_sol)
    print("u_sol: ", u_sol)

    print(result.get_solution_result())

    # Reconstruct the trajectory
    u_sol = np.expand_dims(u_sol, axis=1)
    xdot_sol = np.zeros(x_sol.shape)
    for i in range(N):
        xdot_sol[i] = EvaluateDynamics(plant, plant_context, x_sol[i], u_sol[i])

    x_traj = PiecewisePolynomial.CubicHermite(timesteps, x_sol.T, xdot_sol.T)
    u_traj = PiecewisePolynomial.ZeroOrderHold(timesteps, u_sol.T)

    return x_traj, u_traj, prog, result, prog.GetInitialGuess(x), prog.GetInitialGuess(u)


if __name__ == "__main__":
    N = 11
    initial_state = np.zeros(4)
    final_configuration = np.array([np.pi, 0])
    box_centers = np.array([[1.5, 0.0],[-1.5, 0.0]])
    box_width = 1.0
    box_height = 1.0
    x_traj, u_traj, prog, _, _ = find_swingup_trajectory(
        N, initial_state, final_configuration,
        box_center, box_width, box_height
    )
