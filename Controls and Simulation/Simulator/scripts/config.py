#Configuration constant for simulator

import numpy as np

# ==================== Physical Constants ====================
GRAVITY = 9.8  # m/s^2
AIR_DENSITY = 1.23 #kg/m^3
# ==================== Drone Parameters ====================
# Mass and Inertia
MASS = 0.829  # kg
INERTIA_TENSOR = np.array([
    [0.002814, 0, 0],
    [0, 0.003882, 0],
    [0, 0, 0.002334]
])  # kg*m^2

# Geometry
MOMENT_ARM = 0.14  # meters, distance from center to motor (use for FW)
THRUST_MOMENT_ARM_Y_m = 0.14 # Distance from center to a propeller in the y direction. used for yaw moment calculation
MOMENT_COEFF_X = 0.144 # coefficient for A2 allocation matrix. multiplied by thrust and deflection to get x roll moment. DERIVED GEMINI 4/28. to be used for radian deflections temp for now, change when CG
MOMENT_COEFF_Y = 0.0616 # coefficient for A2 allocation matrix. multiplied by thrust and deflection to get y roll moment DERIVED GEMINI 4/28
WING_AREA = 0.0923 #m^2
MEAN_AERO_CHORD = 0.1759 #m

# Prop and Control limits
MAX_THRUST_ONE_MOTOR_N = 6.62 #N
MAX_DEFLECTION_TED_RAD = 20*np.pi/180 #20 degrees down max
MIN_DEFLECTION_TED_RAD = -20*np.pi/180 #20 degrees up max
# ==================== Simulation Parameters ====================
SIMULATION_RATE = 500  # Hz
DT = 1.0 / SIMULATION_RATE  # time step
FINAL_TIME = 3.4  # seconds

# ==================== Initial Conditions ====================
INITIAL_POSITION = [0, 0, 1]  # [x, y, z] in meters
INITIAL_VELOCITY = [0, 0, 0]  # [vx, vy, vz] in m/s
INITIAL_QUATERNION = [1, 0, 0, 0]  # [qw, qx, qy, qz] - upright orientation
INITIAL_ANGULAR_VELOCITY = [0, 0, 0]  # [wx, wy, wz] in rad/s

# ==================== Target/Waypoint Parameters ====================
# Final target position
FINAL_POSITION = [1, -1, 0.6]  # [x, y, z] in meters

# Gate parameters
GATE_ANGLE = 60 * np.pi / 180  # radians
GATE_VELOCITY = 2  # m/s, velocity through gate
GATE_HEIGHT = 1  # meters, height off the ground
TIME_TO_GATE = 1.2  # seconds
GATE_POSITION = [0, 0, GATE_HEIGHT]  # [x, y, z] in meters


# ==================== Guidance Gains ====================

#PD gains on basic guidance:
Kp_BG = np.diag([12, 12, 12])
Kd_BG = np.diag([4,4,4])
# ==================== Controller Gains ====================
# Position controller
Kp_POSITION = np.diag([12, 12, 12])
Kd_POSITION = np.diag([4, 4, 4])

# Attitude controller
Kp_ATTITUDE = np.diag([3.7, 3.7, 3.7])
Kd_ATTITUDE = np.diag([0.19, 0.19, 0.19])
LAMBDA_ATTITUDE = np.diag([0.2, 0.2, 0.2])
#=========== AERO Approximations ==========
CD_MAX_FLAT = 1.2; #max CD from flat plate 
CD_0_FLAT = 0.05 #base CD


# ==================== Safety Limits ====================
MIN_ALTITUDE = 0.1  # meters, crash detection threshold
CRASH_CHECK_TIME = 3.0  # seconds, don't check for crash before this time

# ==================== Data Saving ====================
SAVE_DATA = True
DATA_DIRECTORY = "data/"
