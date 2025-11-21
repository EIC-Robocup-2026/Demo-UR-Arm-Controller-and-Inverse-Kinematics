import const as c
import sympy as sp
import numpy as np
import time

class URArmParameter:
    """Class to hold UR Arm parameters"""
    def __init__(self):
        print("Starting initialization...")
        
        # parameter for links, width, angle of the Demo UR Arm
        # unit in miliimeters and radians
        # reference in Demo Arm image
        self.l1, self.l2, self.l3, self.l4 = c.l1, c.l2, c.l3, c.l4
        self.a1, self.a2, self.a3, self.a4 = c.a1, c.a2, c.a3, c.a4
        self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 = sp.symbols('t1 t2 t3 t4 t5 t6')

        # initial angle for joint 1 - 6 and gripper respectively
        self.joint_initializing_angle = sp.Matrix([c.initial_angle[0], c.initial_angle[1], c.initial_angle[2], c.initial_angle[3], c.initial_angle[4], c.initial_angle[5]])

        # inital theta parameters for joints 1-6
        self.theta_parameters = sp.Matrix([self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]).T

        #initialize thetas, these will be used both controlling the arm and calculating DH parameters
        self.thetas = self.joint_initializing_angle.copy()  # in rad

        #initialize DH parameters
        print("Computing DH parameters...")
        t0 = time.time()
        self.dh_parameters = self.dh()
        print(f"  DH parameters took {time.time() - t0:.2f}s")

        #initialize forward kinematics
        print("Computing forward kinematics...")
        t0 = time.time()
        self.forward_kinematics = self.dh_to_forward_kinematics()
        print(f"  Forward kinematics took {time.time() - t0:.2f}s")

        #initialize jacobian matrix (from end-effector position - last column rows 0-2)
        print("Computing Jacobian matrix...")
        t0 = time.time()
        self.end_effector_position = self.forward_kinematics[:3, 3]
        self.jacobian_matrix = self.end_effector_position.jacobian(self.theta_parameters)
        print(f"  Jacobian matrix took {time.time() - t0:.2f}s")
        
        # Compile numerical functions for fast computation
        print("Compiling numerical functions...")
        t0 = time.time()
        self.fk_func = sp.lambdify((self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6), 
                                    self.end_effector_position, 'numpy')
        self.jac_func = sp.lambdify((self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6), 
                                     self.jacobian_matrix, 'numpy')
        print(f"  Compilation took {time.time() - t0:.2f}s")
        print("Initialization complete!")
    
    #reset thetas to initial position
    def reset_thetas(self):
        self.thetas = self.joint_initializing_angle.copy()

    #return current thetas
    def get_current_thetas(self):
        return self.thetas
    
    #return DH parameters as sp.Matrix
    def dh(self):
        """Return the DH parameters as np.array (d, theta, a, alpha)"""
        dh_parameter = sp.Matrix([[self.l1, self.theta_parameters[0], 0, sp.pi/2],
                              [self.a1, self.theta_parameters[1], 0, -sp.pi/2],
                              [self.l2, 0, 0, -sp.pi/2],
                              [self.a2, self.theta_parameters[2], 0, sp.pi/2],
                              [self.l3, 0, 0, sp.pi/2],
                              [self.l4, self.theta_parameters[3], 0, -sp.pi/2],
                              [self.a3, self.theta_parameters[4], 0, sp.pi/2],
                              [self.a4, self.theta_parameters[5], 0, 0] ])
        
        return dh_parameter
    
    #compute transformation matrix from DH parameters
    def dh_to_transformation(self, d, theta, a, alpha):
        """Convert DH parameters to transformation matrix"""
        T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                       [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                       [0, sp.sin(alpha), sp.cos(alpha), d],
                       [0, 0, 0, 1]])
        return T
    
    #calculate forward kinematics from DH parameters
    def dh_to_forward_kinematics(self):
        """Calculate forward kinematics using DH parameters"""
        dh_params = self.dh_parameters
        T_total = sp.eye(4)
        
        for i in range(dh_params.rows):
            d, theta, a, alpha = dh_params.row(i)
            T = self.dh_to_transformation(d, theta, a, alpha)
            T_total = T_total * T
    
        return T_total
    
    # Substitute theta values into any symbolic matrix
    def substitute_thetas(self, matrix, thetas=None):
        """Substitute theta values into any symbolic matrix
        
        Args:
            matrix: Symbolic matrix to substitute into (e.g., forward_kinematics, jacobian_matrix, etc.)
            thetas: sp.Matrix or list of 6 theta values. If None, uses current self.thetas
            
        Returns:
            Matrix with substituted theta values
        """
        if thetas is None:
            thetas = self.thetas
        
        theta_values = {self.theta1: thetas[0],
                        self.theta2: thetas[1],
                        self.theta3: thetas[2],
                        self.theta4: thetas[3],
                        self.theta5: thetas[4],
                        self.theta6: thetas[5]}
        
        substituted = matrix.subs(theta_values)
        return substituted
    
    #return current end-effector position
    def get_end_effector_position(self):
        """Return the current end-effector position using compiled numerical function"""
        pos = self.fk_func(float(self.thetas[0]), float(self.thetas[1]), float(self.thetas[2]), 
                           float(self.thetas[3]), float(self.thetas[4]), float(self.thetas[5]))
        return sp.Matrix(pos.flatten())
    

    #If movement by unit direction is possible, update thetas and return True, else return False
    def move_by(self, unit_direction):
        """Move the end-effector by a small step in the given unit direction using Jacobian inverse
        
        Args:
            unit_direction: Direction vector to normalize and scale by CONTROL_STEP_SIZE
        """
        
        # Normalize direction and scale by step size
        normalized_dir = unit_direction.normalized()
        step_direction = normalized_dir * c.CONTROL_STEP_SIZE
        
        # Move the end-effector in the given direction
        self.current_pos = self.get_end_effector_position()
        self.target_pos = self.current_pos + step_direction

        #temp theta copy for iteration
        self.theta_copy = self.thetas.copy()

        self.error = self.target_pos - self.current_pos

        #keeping track of iterations
        self.iteration = 0

        while (self.error.norm() > c.CONVERGENCE_THRESHOLD) and (self.iteration < c.MAX_ITERATIONS):
            # Use compiled Jacobian function for speed
            jac_numpy = self.jac_func(float(self.theta_copy[0]), float(self.theta_copy[1]), float(self.theta_copy[2]),
                                      float(self.theta_copy[3]), float(self.theta_copy[4]), float(self.theta_copy[5]))
            dir_numpy = np.array(step_direction.evalf(), dtype=float).flatten()
            
            # Use numerical pseudo-inverse (much faster)
            jac_pinv = np.linalg.pinv(jac_numpy)
            delta_thetas_numpy = jac_pinv @ dir_numpy
            
            # Convert back to sympy
            self.delta_thetas = sp.Matrix(delta_thetas_numpy)

            #update theta copy
            self.theta_copy += self.delta_thetas

            #update current position using compiled function
            pos_numpy = self.fk_func(float(self.theta_copy[0]), float(self.theta_copy[1]), float(self.theta_copy[2]),
                                     float(self.theta_copy[3]), float(self.theta_copy[4]), float(self.theta_copy[5]))
            self.current_pos = sp.Matrix(pos_numpy.flatten())

            #update error
            self.error = self.target_pos - self.current_pos

            #increment iteration
            self.iteration += 1

        if self.iteration < c.MAX_ITERATIONS:
            #update thetas if converged
            self.thetas = self.theta_copy
            return True  # movement successful
        
        else:
            self.thetas = self.theta_copy
            return False  # movement failed due to max iterations



# parameter = URArmParameter()
# print("UR Arm parameters initialized.")

# print("Initial position:")
# init_pos = parameter.get_end_effector_position()
# print(init_pos.evalf(n=5))

# for i in range(100):
#     parameter.move_by(sp.Matrix([1,1,0]))


# print("\nFinal position:")
# final_pos = parameter.get_end_effector_position()
# print(final_pos.evalf(n=5))

# print("\nDiff ", init_pos.evalf(n=5) - final_pos.evalf(n=5))
# print("\nSize ", (init_pos.evalf(n=5) - final_pos.evalf(n=5)).norm())
