import numpy as np
import math
import config
import quaternion_helpers as qhelp
import config

def get_a_com_bg(state, t):
    ##Basic guidance algorithm. for this case, accelerates in x direction for 1 seconds, then moves at constant velocity for 3 seconds, then decelerates for 1 second to come to a stop
    #generates position/velocity trajectories accordingly
    a_des = np.zeros(3)
    v_des = np.zeros(3)
    pos_des = np.zeros(3)
    

    if t >= 0 and t < 1:
        # Phase 1: acceleration = 1
        a_des = np.array([1, 0, 0])
        v_des = np.array([t, 0, 0])  # v = a*t = 1*t
        pos_des = np.array([0.5*t**2, 0, 0])  # pos = 0.5*a*t^2
        
    elif t >= 1 and t < 4:
        # Phase 2: acceleration = 0 (constant velocity)
        a_des = np.array([0, 0, 0])
        v_des = np.array([1, 0, 0])  # v = v_final from phase 1 = 1
        pos_des = np.array([0.5 + 1*(t-1), 0, 0])  # pos = pos_at_t1 + v*(t-1)
        
    elif t >= 4 and t < 5:
        # Phase 3: acceleration = -1
        a_des = np.array([-1, 0, 0])
        v_des = np.array([1 - 1*(t-4), 0, 0])  # v = v_at_t4 + a*(t-4) = 1 - (t-4)
        pos_des = np.array([3.5 + 1*(t-4) - 0.5*(t-4)**2, 0, 0])  # pos = pos_at_t4 + v*(t-4) + 0.5*a*(t-4)^2
        
    else:
        # t >= 5 or t < 0: stationary
        a_des = np.zeros(3)
        v_des = np.zeros(3)
        if t >= 5:
            pos_des = np.array([4, 0, 0])  # final position
        else:
            pos_des = np.zeros(3)

    pos_error = state[0:3]-pos_des
    vel_error = state[3:6]-v_des

    a_com_bg = a_des - config.Kp_BG @ pos_error - config.Kd_BG @ vel_error
    return a_com_bg

    


    

