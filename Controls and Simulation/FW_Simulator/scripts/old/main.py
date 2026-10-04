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
import dynamics

##########################################
############ Drone Simulation ############
##########################################

# Save data flag
save_data = True

# Initial conditions
t = 0.

state = np.zeros(13) # 13 without trajectory
f = np.zeros(4)


x0, y0, z0 = [0, 0, 1] 

# x, y, z
state[0] = x0
state[1] = y0
state[2] = z0

# vx, vy, vz
state[3] = 0.
state[4] = 0.
state[5] = 0.

# qw, qx, qy, qz
state[6] = 1.
state[7] = 0.
state[8] = 0.
state[9] = 0.

# wx, wy, wz
state[10] = 0.
state[11] = 0.
state[12] = 0.

# Final state
tf = 3.4
xf = 1
yf = -1
zf = 0.6 


# Simulation rate
rate = 500
dt = 1./rate

# Gravity
g = 9.8

# Other parameters?
m = 0.829

# Initialize dynamics
dyn = dynamics.dynamics(np.array([g]), dt)

# Initialize data array that contains useful info (probably should add more)
data = np.append(t,state)
data = np.append(data,f) # data array has time, state (13 deg) and motor forces (4 deg)
data = np.append(data,[x0,y0, z0]) #append trajectory

# Trajectory Planner
theta = 60*3.14159265/180 # radians, gate angle
V_xg = 2 # m/s, velocity through gate
h_g = 1 # meters, height off the ground
# tg is time to gate
tg = 1.2 #PATH1
#tg = 2 #PATH2
xg, yg, zg = [0, 0, h_g] #PATH1
#xg, yg, zg = [0, 1.25, h_g] #PATH2
# APPROACH TRAJECTORY (1st half)
A_app = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 2, 0, 0, 0, 0, 0],
              [0, 0, 0, 6, 0, 0, 0, 0],
              [1, tg, tg**2, tg**3, tg**4, tg**5, tg**6, tg**7,],
              [0, 1, 2*tg, 3*tg**2, 4*tg**3, 5*tg**4, 6*tg**5, 7*tg**6],
              [0, 0, 2, 6*tg, 12*tg**2, 20*tg**3, 30*tg**4, 42*tg**5], 
              [0, 0, 0, 6, 24*tg, 60*tg**2, 120*tg**3, 210*tg**4]])
b_app = np.array([[x0, y0, z0],
              [0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],
              [xg,yg, zg],
              [V_xg, 0, 0],
              [0, -g*np.tan(theta), 0], #outdated, needs to be neg y
              [0, 0, 0]])
Ainv_app = np.linalg.inv(A_app)
c_app = np.matmul(Ainv_app, b_app)

# DEPARTURE TRAJECTORY (2nd half)
A_dep = np.array([[1, tg, tg**2, tg**3, tg**4, tg**5, tg**6, tg**7],
              [0, 1, 2*tg, 3*tg**2, 4*tg**3, 5*tg**4, 6*tg**5, 7*tg**6],
              [0, 0, 2, 6*tg, 12*tg**2, 20*tg**3, 30*tg**4, 42*tg**5], 
              [0, 0, 0, 6, 24*tg, 60*tg**2, 120*tg**3, 210*tg**4],
              [1, tf, tf**2, tf**3, tf**4, tf**5, tf**6, tf**7,],
              [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4, 6*tf**5, 7*tf**6],
              [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3, 30*tf**4, 42*tf**5], 
              [0, 0, 0, 6, 24*tf, 60*tf**2, 120*tf**3, 210*tf**4]])
b_dep = np.array([[xg,yg, zg],
              [V_xg, 0, 0],
              [0, -g*np.tan(theta), 0], #outdated, need to be in negative y
              [0, 0, 0],
              [xf, yf, zf],
              [0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],])
Ainv_dep = np.linalg.inv(A_dep)
c_dep = np.matmul(Ainv_dep, b_dep)


c1T, c2T, c3T, c4T, c5T, c6T, c7T, c8T = trp(c_app[0,:]), trp(c_app[1,:]), trp(c_app[2,:]), trp(c_app[3,:]), trp(c_app[4,:]), trp(c_app[5,:]), trp(c_app[6,:]), trp(c_app[7,:])
c9T, c10T, c11T, c12T, c13T, c14T, c15T, c16T = trp(c_dep[0,:]), trp(c_dep[1,:]), trp(c_dep[2,:]), trp(c_dep[3,:]), trp(c_dep[4,:]), trp(c_dep[5,:]), trp(c_dep[6,:]), trp(c_dep[7,:])

def quat_multi(q1,q0):
    w1, x1, y1, z1 = q1
    w0, x0, y0, z0 = q0
    #w_prod = w1*w2-np.matmul(np.array([x1, y1, z1]).transpose(), np.array([x2, y2, z2]))
   # xyzprod = w1*np.array([x2, y2, z2])
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def quat_to_rot(q):
    # q is of form [w, x, y, z]
    # q = [state[6], state[7], state[8], state[9]]
    w, x, y, z = q
    R = np.array([[1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]])
    return R

#TO SWITCH BETWEEN ATTITUDE ONLY AND ATTITUDE + TRAJECTORY CONTROL, COMMENT/UNCOMMENT LINES: 49-52, 169, 190, 200
# Simulation loop
running = True
while running:
    #Trajectory gets calculated for each timestep, at the current time
    if t<=tg:
        r_d = c8T * t ** 7 + c7T * t ** 6 + c6T * t ** 5 + c5T * t ** 4 + c4T * t ** 3 + c3T * t ** 2 + c2T * t + c1T  # Position Trajectory (3 dimensions)
        r_d_dot = 7 * c8T * t ** 6 + 6 * c7T * t ** 5 + 5 * c6T * t ** 4 + 4 * c5T * t ** 3 + 3 * c4T * t ** 2 + 2 * c3T * t + c2T  # Velocity Trajectory
        r_d_2dot = 42 * c8T * t ** 5 + 30 * c7T * t ** 4 + 20 * c6T * t ** 3 + 12 * c5T * t ** 2 + 6 * c4T * t + 2 * c3T  # Acceleration Trajectory
        r_d_3dot = 210 * c8T * t ** 4 + 120 * c7T * t ** 3 + 60 * c6T * t ** 2 + 24 * c5T * t + 6 * c4T  # Jerk Trajectory
    if t>tg:
        r_d = c16T * t ** 7 + c15T * t ** 6 + c14T * t ** 5 + c13T * t ** 4 + c12T * t ** 3 + c11T * t ** 2 + c10T * t + c9T  # Position Trajectory (3 dimensions)
        r_d_dot = 7 * c16T * t ** 6 + 6 * c15T * t ** 5 + 5 * c14T * t ** 4 + 4 * c13T * t ** 3 + 3 * c12T * t ** 2 + 2 * c11T * t + c10T  # Velocity Trajectory
        r_d_2dot = 42 * c16T * t ** 5 + 30 * c15T * t ** 4 + 20 * c14T * t ** 3 + 12 * c13T * t ** 2 + 6 * c12T * t + 2 * c11T  # Acceleration Trajectory
        r_d_3dot = 210 * c16T * t ** 4 + 120 * c15T * t ** 3 + 60 * c14T * t ** 2 + 24 * c13T * t + 6 * c12T  # Jerk Trajectory

 # For testing:
   # r_d = [0,0,0.1*t+0.5]
  #  r_d_dot = [0,0,0.1]
   # r_d_2dot = [0, 0, 0]
    ## POSITION CONTROLLER
    if t>1:
        x = 0 #breakpoint

    r_e = np.array([state[0], state[1], state[2]])-r_d
    r_e_dot = np.array([state[3], state[4], state[5]])-r_d_dot
    Kp_pos = np.diag([12, 12, 12])
    Kd_pos = np.diag([4, 4, 4])
    a_com = r_d_2dot-(Kp_pos @ r_e)-(Kd_pos @ r_e_dot)+[0, 0, g] #commanded acceleration
   # a_com = r_d_2dot+[0,0,g] #no feedback control, just feed forward
    T_com = m*np.linalg.norm(a_com) #commanded total thrust
    e3 = np.array([0, 0, 1])
    a_hat = a_com/np.linalg.norm(a_com)
    q_d_r = 1+np.dot(e3,a_hat)
    q_d_i = np.cross(e3,a_hat)
    q_d = np.append([q_d_r], q_d_i)/math.sqrt(2*(1+np.dot(e3,a_hat)))
    q_d = q_d/np.linalg.norm(q_d)
    R_d = quat_to_rot(q_d)

    # Get new desired state from trajectory planner
    # xd, yd, zd, ... = get_desired_state(t)
    xd, yd, zd = c8T*t**7 + c7T*t**6 + c6T*t**5 + c5T*t**4 + c4T*t**3 + c3T*t**2 + c2T*t + c1T # Position Trajectory
    vxd, vyd, vzd = 7*c8T*t**6 + 6*c7T*t**5 + 5*c6T*t**4 + 4*c5T*t**3 + 3*c4T*t**2 + 2*c3T*t + c2T # Velocity Trajectory
    axd, ayd, azd = 42*c8T*t**5 + 30*c7T*t**4 + 20*c6T*t**3 + 12*c5T*t**2 + 6*c4T*t + 2*c3T # Acceleration Trajectory
    jxd, jyd, jzd = 210*c8T*t**4 + 120*c7T*t**3 + 60*c6T*t**2+ 24*c5T*t + 6*c4T # Jerk Trajectory
    print(jxd)

    # Run outer-loop controller to get thrust and references for inner loop 
    # T, ... 
 
    # #can update later, just upright to test
    q = [state[6], state[7], state[8], state[9]]
    q_d_star = [q_d[0], -q_d[1], -q_d[2], -q_d[3]]
    q_e = quat_multi(q_d_star, q)
    #Controller Gains
    Kp_att = np.diag([3.7, 3.7, 3.7]) #3.4
    Kd_att = np.diag([.19, .19, .19]) #0.17
    w = [state[10],state[11],state[12]]
    #control command
    q_e_vector = np.array(q_e[1:])
    #T_com = m*g #placeholder, commanded total thrust
    #older controller
   # tau = -q_e[0]*(Kp_att @ q_e_vector) - (Kd_att @ w)
    #full attitude controller (updated with lambda)
    q_e_dot = 0.5*quat_multi(q_e, np.append([0],w))
    lambda_ = np.diag([.2,.2,.2])
    #newer controller
    tau = -np.sign(q_e[0]) * (Kp_att @ q_e_vector) - (Kd_att @ w) + np.sign(q_e[0])*(lambda_ @ q_e_dot[1:])
    tau_full = np.append(tau,T_com)
    #convert to forces
    l = 0.14 #meters, approximation of moment arm of motor
    c = 0.1 #placeholder, not sure if zero or not? (edit: 0.1 ish)
    A = np.array([
    [l, l, -l, -l],
    [-l, l, l, -l],
    [c, -c, c, -c],
    [1, 1, 1 ,1] #fourth row is to make sure all forces add to thrust
])
#%A_pinv = np.linalg.pinv(A)
    A_inv = np.linalg.inv(A)

    f = A_inv @ tau_full
    #f = f + np.array([m*g/4, m*g/4,m*g/4,m*g/4]) #add to also counter mass weight (nvm don't use this)
    # Run inner-loop controller to get motor forces 
   # f = [3, 3, 3, 3]

    # Propagate dynamics with control inputs
    state = dyn.propagate(state, f, dt)
 
    # If z to low then indicate crash and end simulation
    if state[2] < 0.1 and t > 3:
        print("CRASH!!!")
        break
        #override
    if t > 3:
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
#np.savetxt("C:/Users/ncv72/Documents/"+file_name, data, delimiter=",")
   # np.savetxt("C:/Users/danielr1/Documents/157A/SimSaves/" + file_name, data, delimiter=",")
    # need to fix this ----
np.savetxt("../data/"+file_name, data, delimiter=",")

