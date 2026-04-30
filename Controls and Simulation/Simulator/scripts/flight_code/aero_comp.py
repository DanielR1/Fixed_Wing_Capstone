import numpy as np
import config
import quaternion_helpers as qhelp


#This function takes an acceleration command from guidance, and subtracts the predicted acceleration from lift and drag at the current state, 
#essentially returning commanded acceleration to be done by the actuators/motor. \

def aero_comp(a_com_old, state):
    vx = state[3]
    vy = state[4]
    vz = state[5]
    V = np.sqrt(vx**2+vy**2+vz**2)
    rho = config.AIR_DENSITY
    S = config.WING_AREA
    CDMax = config.CD_MAX_FLAT
    CD0 = config.CD_0_FLAT
    q = state[6:10]

    #Flat plate approximations for feedforward, with parasitic drag term CD0
    #assuming no sideslip aero due to slim body
    (alpha,beta) = state[13:15]
    CX = CD0*np.cos(alpha)
    CZ = (CDMax+CD0)*np.sin(alpha)
    Fx_body = -0.5*rho*V**2*S*CX
    Fz_body = 0.5*rho*V**2*S*CZ #CONVENTION FOR THIS: z points up. might revise later, just add negative
    a_aero_body = np.array([Fx_body, 0, Fz_body])
    a_aero_global = qhelp.quat_to_R(q) @ a_aero_body
    a_com_new = a_com_old - a_aero_global
    return a_com_new