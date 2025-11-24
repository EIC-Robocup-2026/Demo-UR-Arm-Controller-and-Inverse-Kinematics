import sympy as sp
#links and widths of the Demo UR Arm in milimeters
l1 = 172
l2 = 350
l3 = 350
l4 = 95

a1 = 96.05
a2 = 95.8
a3 = 95
a4 = 50


#initial angles for joints 1 - 6 in rad
initial_angle = [0,sp.pi/4,sp.pi/3,0,sp.pi/2,0]

#gripper parameters
GRIPPER_MAX_ANGLE = 90  # degrees
GRIPPER_MIN_ANGLE = 0   # degrees
GRIPPER_INITIAL_ANGLE = 45  # degrees
GRIPPER_INITIAL_ROLL_ANGLE = 0  # degrees

#control parameters
CONTROL_STEP_SIZE = 1  # step size for each control input in mm 
GRIPPER_ROTATION_STEP = 0.25  # degrees per gripper control input
MAX_ITERATIONS = 5  # maximum iterations for inverse kinematics solver
CONVERGENCE_THRESHOLD = 0.01  # convergence threshold for IK solver in mm
RESET_STEP_SIZE = 0.05  # step size for resetting arm (radians)

THETA_OFFSET_ANGLE = [-135, -135, -135, -135, -135, -90, 0]
