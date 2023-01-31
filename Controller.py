
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        U = 0
        
        control_mode = "PDFF"
        if control_mode == "P":
            U = self.kp_z * (state.z_pos - setpoint.z_pos)

        elif control_mode == "PD":
            # Example gains: Kp = 25, Kd = 15
            z_pos_error = setpoint.z_pos - state.z_pos
            z_vel_error = setpoint.z_vel - state.z_vel
            U = self.kp_z * z_pos_error + self.kd_z * z_vel_error

        elif control_mode == "PDFF":
            # Example gains: 
            z_pos_error = setpoint.z_pos - state.z_pos
            z_vel_error = setpoint.z_vel - state.z_vel
            force_ff = self.params.mass * self.params.g # Force = mg
            U = self.kp_z * z_pos_error + self.kd_z * z_vel_error + force_ff

        return U
