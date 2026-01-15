
import config
# Trajectory Planner
theta = config.GATE_ANGLE
V_xg = config.GATE_VELOCITY
h_g = config.GATE_HEIGHT
tg = config.TIME_TO_GATE
xg, yg, zg = config.GATE_POSITION
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

