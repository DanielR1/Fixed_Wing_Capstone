#inner loop controller 
#in - measured state (from navigation), time, acceleration command (from guidance)
#out - control inputs - two motors and two flaps
import numpy as np
import math
import config
import quaternion_helpers as qhelp
from flight_code import aero_comp as ac

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

        q = [state[6], state[7], state[8], state[9]]
        (qw, qx, qy, qz) = q

        #aero compensation/feedforward
        a_com = ac.aero_comp(a_com, state)

        """Compute control inputs based on state and commanded acceleration"""
        m = config.MASS
        g = config.GRAVITY
        
        #APPROACH - blending hover and fixed-wing desired quaternions based on speed.

        # Hover mode desired quaternion
        T_com_hover = m*np.linalg.norm(a_com) #commanded total thrust
        e3 = np.array([0, 0, 1])
        a_hat = a_com/np.linalg.norm(a_com)
        q_d_r = 1+np.dot(e3,a_hat)
        q_d_i = np.cross(e3,a_hat)
        q_d_hover = np.append([q_d_r], q_d_i)/math.sqrt(2*(1+np.dot(e3,a_hat)))
        q_d_hover = q_d_hover/np.linalg.norm(q_d_hover)

        #Fixed-wing mode desired quaternion
        
        vx = state[3]
        vy = state[4]
        vz = state[5]
        V = np.sqrt(vx**2+vy**2+vz**2)

        if (V>0.1): #guardrail to prevent violent switch in variables

            psi = np.arctan2(vy, vx) #flight path heading angle
            lat_dir_global = np.array([-np.sin(psi), np.cos(psi), 0.0])
            a_lat = np.dot(a_com, lat_dir_global) #lateral component of acceleration (in horizontal plane perp. to motion).

            # {        # # Get heading direction in horizontal plane (body x-axis projected to x-y plane)
            # R_body = qhelp.quat_to_R(q)  # body-to-global rotation matrix
            # x_body = R_body[:, 0]  # forward direction in global frame
            # heading_flat = np.array([x_body[0], x_body[1], 0])  # project to x-y plane
            # heading_flat = heading_flat / np.linalg.norm(heading_flat)  # normalize
            
            # # Perpendicular direction in horizontal plane (rotate 90° in x-y plane)
            # perp_flat = np.array([-heading_flat[1], heading_flat[0], 0])
            
            # # Project a_com onto this perpendicular direction to get lateral acceleration
            # a_lat = np.dot(a_com, perp_flat)

            #compute desired bank angle
            g = config.GRAVITY
            phi_d = np.arctan2(a_lat, g)
            #theta_d = np.asin(2*q_d_hover[1]*q_d_hover[2] + 2*q_d_hover[3]*q_d_hover[0])
            theta_d = qhelp.quat_to_euler_ZXY(q_d_hover)[1]
            q_d_fw = qhelp.euler_ZXY_to_quat(phi_d, theta_d, psi) #fw to be blended
        else:
            q_d_fw = q_d_hover

        #Blending hover and fixed wing quaternions
        #Blending weights:
        V_min = 2.0 #m/s
        V_max = 10.0 #m/s
        w = np.clip((V - V_min) / (V_max - V_min), 0.0, 1.0)
        # 3. Spherical Linear Interpolation (SLERP)
        # When w=0, q_d is 100% q_d_hover. When w=1, q_d is 100% q_d_fw.
        q_d = qhelp.slerp(q_d_hover, q_d_fw, w)

        R_d = qhelp.quat_to_R(q_d)
        T_com =T_com_hover; #might need to change later?

        # Attitude controller

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
        
        #tau_full = np.append(tau,T_com)

        
        # Convert to forces
        l_y = config.THRUST_MOMENT_ARM_Y_m
        cx = config.MOMENT_COEFF_X
        cy = config.MOMENT_COEFF_Y


        #Step 1: Find thrust forces from yaw moment and total thrust
        tau_1 = np.array([tau[2], T_com])  # tau[2] is yaw moment
        A1 = np.array([
            [l_y, -l_y],
            [1,1]
        ])
        A1_inv = np.linalg.inv(A1)

        [T1, T2] = A1_inv @ tau_1
        Tmax_N = config.MAX_THRUST_ONE_MOTOR_N

        # Bound Motors and convert to normal float (min and max thrust)
        T1 = float(min(max(0.1, T1),Tmax_N))
        T2 = float(min(max(0.1, T2),Tmax_N))



        tau_2 = tau[0:2]  # roll and pitch moments (tau[0] and tau[1])
        A2 = np.array([
            [cx*T1, -cx*T2],
            [-cy*T1, -cy*T2]
        ])
        A2_inv = np.linalg.inv(A2)

        [delta_1, delta_2] = A2_inv @ tau_2 #This is in RADIANS
        #convert to normal float and bound
        min_delta = config.MIN_DEFLECTION_TED_RAD
        max_delta = config.MAX_DEFLECTION_TED_RAD

        delta_1 = float(min(max(min_delta, delta_1),max_delta))
        delta_2 = float(min(max(min_delta, delta_2),max_delta))
        #Combine
        ctrl_in = [T1, T2, delta_1, delta_2]

        return ctrl_in