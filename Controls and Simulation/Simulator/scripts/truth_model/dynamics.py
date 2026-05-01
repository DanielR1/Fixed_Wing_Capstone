# Filename: dynamics.py
# Author: 157A Team 1
# Created: 4/20/25
# Description: Dynamics for drone simulator

import numpy as np
import config
from truth_model import aero
import helper_funcs as help
import quaternion_helpers as qhelp

class dynamics: 
	def __init__(self, params, dt):
		# Initialize Params from config
		self.m = config.MASS
		self.g = config.GRAVITY
		self.J = config.INERTIA_TENSOR
		self.l = config.MOMENT_ARM
		self.c = 0#config.DRAG_COEFFICIENT
		

	# This is meant to give the rates of each state. Basically Equations of Motion
	def rates(self, state, ctrl_in):
		# Get rotation matrix from current quaterion
		q_current = state[6:10]
		#R = self.quat_to_rot([state[6], state[7], state[8], state[9]])
		R = qhelp.quat_to_R(q_current)

		#get all forces and moments
		FM_aero = aero.getAeroForcesMoments(state, ctrl_in) #not including contorl deflections
		FM_control = self.get_control_forces_moments_body(ctrl_in)
		FM_Total = FM_aero-FM_control
		Force_body_N = FM_Total[0:3]
		
		# Velocities
		dx = state[3]
		dy = state[4]
		dz = state[5]

		# Accelerations
		# dvx = R[0,0] * T  / self.m
		# dvy = R[1,0] * T  / self.m
		# dvz = R[2,0] * T  / self.m - self.g

		[dvx, dvy, dvz] = R @ Force_body_N #accelerations in global coordinates
		dvz = dvz-self.g #correct for gravity

		# Orientation
		wx, wy, wz = state[10], state[11], state[12]
		omega = np.array([[0, -wx, -wy, -wz], 
				 [wx, 0, wz, -wy], 
				 [wy, -wz, 0, wx], 
				 [wz, wy, -wx, 0]])
		dq = np.matmul(omega, q_current) / 2
		dqw = dq[0]
		dqx = dq[1]
		dqy = dq[2]
		dqz = dq[3]

		# Angular Velocities
		# Torque
		# tau_x = FM_Total[3]
		# tau_y = FM_Total[4]
		# tau_z = FM_Total[5]
		# tau = np.array([tau_x, tau_y, tau_z])
		tau = FM_Total[3:6]
		w = np.array([wx, wy, wz])
		Jinv = np.linalg.inv(self.J)
		Jw = np.matmul(self.J, w)
		w_cross_Jw = np.cross(w, Jw)
		dw = np.matmul(Jinv, (tau - w_cross_Jw))
		dwx = dw[0]
		dwy = dw[1]
		dwz = dw[2]

		res = np.array([dx, dy, dz, dvx, dvy, dvz, dqw, dqx, dqy, dqz, dwx, dwy, dwz])

		return res

	# Numerical integration scheme (can do better than Euler!) Do RK4 later
	def propagate(self, state, ctrl_in, dt):
		state[0:13] += dt * self.rates(state, ctrl_in)

		# GAUSSIAN DISTURBANCES
		# r_noise_stddev = 0.01 # linear position standard dev
		# w_noise_stddev = 0.005 # angular velocity standard dev
		# state[0:3] += np.random.normal(0, r_noise_stddev, size=3)
		# state[10:13] += np.random.normal(0, w_noise_stddev, size=3)
		
		#normalize quaternion
		q = state[6:10]
		quat_norm = np.linalg.norm(q)
		state[6:10] = state[6:10] / quat_norm
		state[13:15] = help.compute_alpha_beta(state)
		return state

	# Helper function that converts a quaternion to 3x3 rotation matrix
	# def quat_to_rot(self, q):
	# 	# q is of form [w, x, y, z]
	# 	# q = [state[6], state[7], state[8], state[9]]
	# 	w, x, y, z = q
	# 	R = np.array([[1-2*(y**2 + z**2),	2*(x*y - w*z),		2*(x*z + w*y)],
	# 				 [2*(x*y + w*z),		1-2*(x**2 + z**2),	2*(y*z - w*x)],
	# 				 [2*(x*z - w*y),		2*(y*z + w*x),		1-2*(x**2 + y**2)]])
	# 	return R

	def get_control_forces_moments_body(self, ctrl_in):
		#takes the state and the control vector and calculates the forces and moments created by the four control inputs (two thrust, two deltas)
		(T1, T2, delta_1, delta_2) = ctrl_in
		cx = config.MOMENT_COEFF_X
		cy = config.MOMENT_COEFF_Y
		l_y = config.THRUST_MOMENT_ARM_Y_m
		tau_x_Nm = cx*T1*delta_1-cx*T2*delta_2 # rolling/x moment
		tau_y_Nm = -cy*T1*delta_1 - cy*T2*delta_2 # pitching/y moment
		tau_z_Nm = T1*l_y-T2*l_y # yawing/z moment
		thrust_force_N = T1+T2
		force_moment_array = np.array([thrust_force_N, 0, 0, tau_x_Nm, tau_y_Nm, tau_z_Nm])
		return force_moment_array
