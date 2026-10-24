import numpy as np
from pydrake.autodiffutils import AutoDiffXd

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

def box_obstacle(ee_pos, box_center, width, height):
  '''
  Calculate signed (squared) distance to box obstacle
    ee_pos: [x,z] np.array
    box_center: [x,z] np.array
    width and height: from end to end
  '''
  # TODO: Write signed distance formula for box

  # change with your solution
  return signed_dist

def collision_constraint_function(box_params):
  '''
  Impose a constraint such that the EE does not collide with the obstacle
  pos (x_EE, z_EE):
    Matrix([[-l*sin(q0) - l*sin(q0 + q1)], [-l*cos(q0) - l*cos(q0 + q1)]])

  '''
  l = 1
  g = 9.81
  box_center = box_params[0]
  box_width = box_params[1]
  box_height = box_params[2]
  
  def collision_box_fn(vars):
    q = vars[:2]
    constraint_eval = np.zeros((1,), dtype=AutoDiffXd)
    pos = np.array([[-l*sin(q[0]) - l*sin(q[0] + q[1])],
                    [-l*cos(q[0]) - l*cos(q[0] + q[1])]])
    
    # TODO: Express obstacle avoidance constraint as a function of q

    return constraint_eval
    
  return collision_box_fn

def AddBoxCollisionConstraints(prog, q, box_params):
  '''
  q: joint pos (q0, q1)
  box_params: tuple ([center_x, center_z], width, height)
  '''
  coll_constr = collision_constraint_function(box_params)

  # TODO: Add collision constraint as an inequality constraint using prog.AddConstraint
