#inner loop controller 
#in - measured state (from navigation), time, acceleration command (from guidance)
#out - control inputs - two motors and two flaps
import numpy as np
import math
import config
import quaternion_helpers as qhelp

class Controller:
    def __init__(self, dt):
        """Initialize controller with integrator states"""
        self.dt = dt
        
        # Integrator states for position controller
        self.position_integral = np.zeros(3)  # [x, y, z] integral error
        
        # Integrator states for attitude controller (if needed)
        self.attitude_integral = np.zeros(3)  # [roll, pitch, yaw] integral error
        
        # Anti-windup limits
        self.max_position_integral = 1.0  # Adjust as needed
        self.max_attitude_integral = 0.5  # Adjust as needed
        
    def reset_integrals(self):
        """Reset all integral states (useful when switching modes)"""
        self.position_integral = np.zeros(3)
        self.attitude_integral = np.zeros(3)
    
    def get_control_inputs(self, state, a_com, t):
        """Compute control inputs based on state and commanded acceleration"""
    def get_control_inputs(self, state, a_com, t):
        """Compute control inputs based on state and commanded acceleration"""
        m = config.MASS
        g = config.GRAVITY
        
        T_com = m*np.linalg.norm(a_com) #commanded total thrust
        e3 = np.array([0, 0, 1])
        a_hat = a_com/np.linalg.norm(a_com)
        q_d_r = 1+np.dot(e3,a_hat)
        q_d_i = np.cross(e3,a_hat)
        q_d = np.append([q_d_r], q_d_i)/math.sqrt(2*(1+np.dot(e3,a_hat)))
        q_d = q_d/np.linalg.norm(q_d)
        R_d = qhelp.quat_to_R(q_d)

        # Attitude controller
        q = [state[6], state[7], state[8], state[9]]
        q_d_star = [q_d[0], -q_d[1], -q_d[2], -q_d[3]]
        q_e = qhelp.quat_mult(q_d_star, q)
        
        # Controller Gains
        Kp_att = config.Kp_ATTITUDE
        Kd_att = config.Kd_ATTITUDE
        w = [state[10],state[11],state[12]]
        
        # Control command
        q_e_vector = np.array(q_e[1:])
        
        # Update integral (example - uncomment if you want integral control)
        # self.attitude_integral += q_e_vector * self.dt
        # # Anti-windup: clamp integral
        # self.attitude_integral = np.clip(self.attitude_integral, 
        #                                  -self.max_attitude_integral, 
        #                                   self.max_attitude_integral)
        # Ki_att = np.diag([0.1, 0.1, 0.1])  # Add to config if needed
        
        # Full attitude controller (updated with lambda)
        q_e_dot = 0.5*qhelp.quat_mult(q_e, np.append([0],w))
        lambda_ = config.LAMBDA_ATTITUDE
        
        # PD controller (add integral term if needed)
        tau = -np.sign(q_e[0]) * (Kp_att @ q_e_vector) - (Kd_att @ w) + np.sign(q_e[0])*(lambda_ @ q_e_dot[1:])
        # If using integral: tau += Ki_att @ self.attitude_integral
        
        tau_full = np.append(tau,T_com)
        
        # Convert to forces
        l = config.MOMENT_ARM
        c = config.DRAG_COEFFICIENT
        A = np.array([
            [l, l, -l, -l],
            [-l, l, l, -l],
            [c, -c, c, -c],
            [1, 1, 1 ,1]  # fourth row is to make sure all forces add to thrust
        ])
        A_inv = np.linalg.inv(A)

        ctrl_in = A_inv @ tau_full
        return ctrl_in