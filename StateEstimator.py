import numpy as np

from utils import State, CrazyflieParams

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """

    params: CrazyflieParams = CrazyflieParams()
    state: State = State()

    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """

        # your code here
        self.params = params
        self.state = init_state
        self.P = np.diag([0.1**2 , 0.1]) # sigma^2_z comes from the simulated sensor noise. Unsure what to use for sigma^2_zdot

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        filtered_state = State()

        # Recreate state vector as an np vector
        X = np.array(   [
            [self.state.z_pos],
            [self.state.z_vel]
        ])

        # Double integrator dynamics in discrete time
        A = np.array([
            [1, time_delta],
            [0, 1]
        ])

        # Input force vector: Effects of gravity and control thrust
        Bu = np.array([
            [0],
            [-time_delta*self.params.g + (time_delta / self.params.mass) * U]
        ])

        # Observer model: Only the z_pos is observable
        C = np.array([[1, 0]])
                
        Q = np.diag([0.01, 0.01])      # Process covariance
        R = 0.1      # Measurement covariance

        # Predict:
        X_predicted = A @ X + Bu
        P_predicted = A @ self.P @ A.T + Q

        # Update:
        K = P_predicted @ C.T @ np.linalg.inv(C @ P_predicted @ C.T + R)
        X_next = X_predicted + K @ (z_meas - C @ X_predicted)
        P_next = (np.eye(2) - K @ C) @ P_predicted

        X_next = X_next.flatten()
        filtered_state.z_pos = X_next[0]
        filtered_state.z_vel = X_next[1]

        self.state = filtered_state
        self.P = P_next

        print("P matrix:")
        print(self.P)
        
        return filtered_state
