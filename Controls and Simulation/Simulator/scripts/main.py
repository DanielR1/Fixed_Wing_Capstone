# Filename: main.py
# Author: 157A Team 1
# Created: 4/20/25
# Description: Drone simulator
import math

### Import python packages ###
import numpy as np
import matplotlib.pyplot as plt
import datetime

from numpy.f2py.symbolic import as_complex
trp = np.transpose

### Import custom modules and classes ###
from truth_model import dynamics
import config
import quaternion_helpers as qhelp
import helper_funcs as help
from flight_code.guidance import guidance
from flight_code import control
from flight_code import navigation
##########################################
############ Drone Simulation ############
##########################################

# Save data flag
save_data = config.SAVE_DATA

# Initial conditions
t = 0.

state = np.zeros(13) # 13 without trajectory
f = np.zeros(4)

# Initial position
x0, y0, z0 = config.INITIAL_POSITION

# x, y, z
state[0] = x0
state[1] = y0
state[2] = z0

# vx, vy, vz
state[3:6] = config.INITIAL_VELOCITY

# qw, qx, qy, qz
state[6:10] = config.INITIAL_QUATERNION

# wx, wy, wz
state[10:13] = config.INITIAL_ANGULAR_VELOCITY

#alpha, beta
state[13:15] = help.compute_alpha_beta(state)

# Final state
tf = config.FINAL_TIME
xf, yf, zf = config.FINAL_POSITION

# Simulation rate
rate = config.SIMULATION_RATE
dt = config.DT

# Gravity
g = config.GRAVITY

# Mass
m = config.MASS

# Initialize dynamics
dyn = dynamics.dynamics(np.array([g]), dt)

# Initialize controller
controller = control.Controller(dt)

# Initialize guidance
guid = guidance.Guidance(dt, "basic_guidance")

# Initialize navigation
nav = navigation.Navigation(dt)

# Initialize data array that contains useful info (probably should add more)
data = np.append(t,state)
data = np.append(data,f) # data array has time, state (13 deg) and motor forces (4 deg)
data = np.append(data,[x0,y0, z0]) #append trajectory



#TO SWITCH BETWEEN ATTITUDE ONLY AND ATTITUDE + TRAJECTORY CONTROL, COMMENT/UNCOMMENT LINES: 49-52, 169, 190, 200
# Simulation loop
running = True
while running:
    # Navigation: estimate state from sensors (currently perfect)
    estimated_state = nav.get_estimated_states(state, None)
    
    # Guidance: compute desired acceleration
    a_com = guid.get_a_com(estimated_state, t)
    
    # Control: compute control commands [T1, T2, delta1, delta2], (L=1, R=2)
    ctrl_in = controller.get_control_inputs(estimated_state, a_com, t)


    #f = f + np.array([m*g/4, m*g/4,m*g/4,m*g/4]) #add to also counter mass weight (nvm don't use this)
    # Run inner-loop controller to get motor forces 
   # f = [3, 3, 3, 3]

    # Propagate dynamics with control inputs
    state = dyn.propagate(state, ctrl_in, dt)
 
    # If z to low then indicate crash and end simulation
    if state[2] < config.MIN_ALTITUDE and t > config.CRASH_CHECK_TIME:
        print("CRASH!!!")
        break
        #override
    if t > config.CRASH_CHECK_TIME:
        print("sim stop")
        break

    # Update data array (this can probably be done in a much cleaner way...)
    tmp = np.append(t,state)
    tmp = np.append(tmp,f)
    tmp = np.append(tmp, r_d)
    data = np.vstack((data,tmp))

    # Update time
    t += dt 

    # If time exceeds final time then stop simulator
    if t >= tf:
        running = False

# Will delete this... for trajectory plotting

# If save_data flag is true then save data
if save_data:
    now = datetime.datetime.now()
    date_time_string = now.strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"data_{date_time_string}.csv"
    np.savetxt(config.DATA_DIRECTORY + file_name, data, delimiter=",")

