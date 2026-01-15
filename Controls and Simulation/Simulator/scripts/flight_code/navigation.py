import numpy as np
import config

class Navigation:
    def __init__(self, dt):
        """Initialize navigation/state estimation system"""
        self.dt = dt
        
        # State estimation history
        self.state_estimate = np.zeros(13)  # Same structure as main state
        self.covariance = np.eye(13)  # State covariance for Kalman filter
        
        # Sensor bias estimates (if using sensor fusion)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        
        # Filter memory (for complementary filter, etc.)
        self.previous_measurement = None

        #NAVIGATION OVERRIDE: just use real states
        self.real_states = True
        
    def reset(self):
        """Reset navigation state"""
        self.state_estimate = np.zeros(13)
        self.covariance = np.eye(13)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        
    def get_estimated_states(self, states, sensor_measurements):
        """Estimate true state from noisy sensor measurements"""
        # Placeholder - implement state estimation here

        
        # For now, just pass through (perfect sensing)
        if self.real_states:
            self.state_estimate = states.copy()
        else:
            #placeholder for actual navigation
            pass
        return self.state_estimate
