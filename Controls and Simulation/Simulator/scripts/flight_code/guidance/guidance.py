#Outer guidance loop
#Inputs - current estimated state(from navigation), time
#Outputs - acceleration command to controller (converted in control class)

import numpy as np
import config

class Guidance:
    def __init__(self, dt):
        """Initialize guidance system"""
        self.dt = dt
        
        # Trajectory history (can store previous waypoints, errors, etc.)
        self.trajectory_history = []
        self.previous_target = None
        
        # Trajectory parameters (if needed for state-dependent planning)
        self.current_waypoint_index = 0
        self.waypoints = []  # List of waypoints to follow
        
    def set_waypoints(self, waypoints):
        """Set a list of waypoints for the guidance system to follow"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
    def reset(self):
        """Reset guidance state"""
        self.trajectory_history = []
        self.previous_target = None
        self.current_waypoint_index = 0
    
    def get_a_com(self, state, t):
        """Compute commanded acceleration based on state and time"""
        # Store current state in history if needed
        # self.trajectory_history.append((t, state.copy()))
        
        # Placeholder - implement your guidance law here
        a_com = np.zeros(3)
        
        return a_com






## old code:

# while running:
#     #Trajectory gets calculated for each timestep, at the current time
#     if t<=tg:
#         r_d = c8T * t ** 7 + c7T * t ** 6 + c6T * t ** 5 + c5T * t ** 4 + c4T * t ** 3 + c3T * t ** 2 + c2T * t + c1T  # Position Trajectory (3 dimensions)
#         r_d_dot = 7 * c8T * t ** 6 + 6 * c7T * t ** 5 + 5 * c6T * t ** 4 + 4 * c5T * t ** 3 + 3 * c4T * t ** 2 + 2 * c3T * t + c2T  # Velocity Trajectory
#         r_d_2dot = 42 * c8T * t ** 5 + 30 * c7T * t ** 4 + 20 * c6T * t ** 3 + 12 * c5T * t ** 2 + 6 * c4T * t + 2 * c3T  # Acceleration Trajectory
#         r_d_3dot = 210 * c8T * t ** 4 + 120 * c7T * t ** 3 + 60 * c6T * t ** 2 + 24 * c5T * t + 6 * c4T  # Jerk Trajectory
#     if t>tg:
#         r_d = c16T * t ** 7 + c15T * t ** 6 + c14T * t ** 5 + c13T * t ** 4 + c12T * t ** 3 + c11T * t ** 2 + c10T * t + c9T  # Position Trajectory (3 dimensions)
#         r_d_dot = 7 * c16T * t ** 6 + 6 * c15T * t ** 5 + 5 * c14T * t ** 4 + 4 * c13T * t ** 3 + 3 * c12T * t ** 2 + 2 * c11T * t + c10T  # Velocity Trajectory
#         r_d_2dot = 42 * c16T * t ** 5 + 30 * c15T * t ** 4 + 20 * c14T * t ** 3 + 12 * c13T * t ** 2 + 6 * c12T * t + 2 * c11T  # Acceleration Trajectory
#         r_d_3dot = 210 * c16T * t ** 4 + 120 * c15T * t ** 3 + 60 * c14T * t ** 2 + 24 * c13T * t + 6 * c12T  # Jerk Trajectory

#  # For testing:
#    # r_d = [0,0,0.1*t+0.5]
#   #  r_d_dot = [0,0,0.1]
#    # r_d_2dot = [0, 0, 0]
#     ## POSITION CONTROLLER
#     if t>1:
#         x = 0 #breakpoint

#     r_e = np.array([state[0], state[1], state[2]])-r_d
#     r_e_dot = np.array([state[3], state[4], state[5]])-r_d_dot
#     Kp_pos = config.Kp_POSITION
#     Kd_pos = config.Kd_POSITION
#     a_com = r_d_2dot-(Kp_pos @ r_e)-(Kd_pos @ r_e_dot)+[0, 0, g] #commanded acceleration