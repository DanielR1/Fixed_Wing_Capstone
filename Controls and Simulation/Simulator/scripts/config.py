#Configuration constant for simulator

import numpy as np

# ==================== Physical Constants ====================
GRAVITY = 9.8  # m/s^2

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
MOMENT_COEFF_X = 1 # coefficient for A2 allocation matrix. multiplied by thrust and deflection to get x roll moment
MOMENT_COEFF_Y = 1 # coefficient for A2 allocation matrix. multiplied by thrust and deflection to get y roll moment

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

# ==================== Safety Limits ====================
MIN_ALTITUDE = 0.1  # meters, crash detection threshold
CRASH_CHECK_TIME = 3.0  # seconds, don't check for crash before this time

# ==================== Data Saving ====================
SAVE_DATA = True
DATA_DIRECTORY = "../data/"
