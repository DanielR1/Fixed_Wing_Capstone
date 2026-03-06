# Filename: dynamics.py
# Author: 157A Team 1
# Created: 4/20/25
# Description: Dynamics for drone simulator

import numpy as np
import config
import aero
import helper_funcs as help

class dynamics: 
	def __init__(self, params, dt):
		# Initialize Params from config
		self.m = config.MASS
		self.g = config.GRAVITY
		self.J = config.INERTIA_TENSOR
		self.l = config.MOMENT_ARM
		self.c = config.DRAG_COEFFICIENT
		

	# This is meant to give the rates of each state
	def rates(self, state, ctrl_in):
		# Get rotation matrix from current quaterion
		R = self.quat_to_rot([state[6], state[7], state[8], state[9]])

		# Get thrust from control inputs
		T = ctrl_in[0:1]

		T_body_coords = np.array([T,0,0])
		

		#get aerodynamic forces (including control input) in body coords
		FM_aero = aero.getAeroForcesMoments(state, ctrl_in)

		Force_body = FM_aero[0:2] + T_body_coords
		
		# Velocities
		dx = state[3]
		dy = state[4]
		dz = state[5]

		# Accelerations
		# dvx = R[0,0] * T  / self.m
		# dvy = R[1,0] * T  / self.m
		# dvz = R[2,0] * T  / self.m - self.g

		[dvx, dvy, dvz] = R @ Force_body #accelerations in global coordinates
		dvz = dvz-self.g #correct for gravity

		# Orientation
		q = np.array([state[6], state[7], state[8], state[9]])
		wx, wy, wz = state[10], state[11], state[12]
		omega = np.array([[0, -wx, -wy, -wz], 
				 [wx, 0, wz, -wy], 
				 [wy, -wz, 0, wx], 
				 [wz, wy, -wx, 0]])
		dq = np.matmul(omega, q) / 2
		dqw = dq[0]
		dqx = dq[1]
		dqy = dq[2]
		dqz = dq[3]

		# Angular Velocities
		# Torque
		tau_x = self.l * (f[0] - f[2])
		tau_y = self.l * (f[1] - f[3])
		tau_z = self.c * (f[0] - f[1] + f[2] - f[3])
		tau = np.array([tau_x, tau_y, tau_z])
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
	# def rates_old(self, state, f):
	# 	# Get rotation matrix from current quaterion
	# 	R = self.quat_to_rot([state[6], state[7], state[8], state[9]])

	# 	# Get thrust from motor forces f
	# 	T = f[0] + f[1] + f[2] + f[3]
		
	# 	# Velocities
	# 	dx = state[3]
	# 	dy = state[4]
	# 	dz = state[5]

	# 	# Accelerations
	# 	dvx = R[0,2] * T  / self.m
	# 	dvy = R[1,2] * T  / self.m
	# 	dvz = R[2,2] * T  / self.m - self.g

	# 	# Orientation
	# 	q = np.array([state[6], state[7], state[8], state[9]])
	# 	wx, wy, wz = state[10], state[11], state[12]
	# 	omega = np.array([[0, -wx, -wy, -wz], 
	# 			 [wx, 0, wz, -wy], 
	# 			 [wy, -wz, 0, wx], 
	# 			 [wz, wy, -wx, 0]])
	# 	dq = np.matmul(omega, q) / 2
	# 	dqw = dq[0]
	# 	dqx = dq[1]
	# 	dqy = dq[2]
	# 	dqz = dq[3]

	# 	# Angular Velocities
	# 	# Torque
	# 	tau_x = self.l * (f[0] - f[2])
	# 	tau_y = self.l * (f[1] - f[3])
	# 	tau_z = self.c * (f[0] - f[1] + f[2] - f[3])
	# 	tau = np.array([tau_x, tau_y, tau_z])
	# 	w = np.array([wx, wy, wz])
	# 	Jinv = np.linalg.inv(self.J)
	# 	Jw = np.matmul(self.J, w)
	# 	w_cross_Jw = np.cross(w, Jw)
	# 	dw = np.matmul(Jinv, (tau - w_cross_Jw))
	# 	dwx = dw[0]
	# 	dwy = dw[1]
	# 	dwz = dw[2]

	# 	res = np.array([dx, dy, dz, dvx, dvy, dvz, dqw, dqx, dqy, dqz, dwx, dwy, dwz])

	# 	return res
	# Numerical integration scheme (can do better than Euler!)
	def propagate(self, state, ctrl_in, dt):
		state += dt * self.rates(state, ctrl_in)

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
	def quat_to_rot(self, q):
		# q is of form [w, x, y, z]
		# q = [state[6], state[7], state[8], state[9]]
		w, x, y, z = q
		R = np.array([[1-2*(y**2 + z**2),	2*(x*y - w*z),		2*(x*z + w*y)],
					 [2*(x*y + w*z),		1-2*(x**2 + z**2),	2*(y*z - w*x)],
					 [2*(x*z - w*y),		2*(y*z + w*x),		1-2*(x**2 + y**2)]])
		return R

