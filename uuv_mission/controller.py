# control.py module

class PDController:
    def __init__(self, kp=0.15, kd=0.6):
        """
        Initialize the PD controller with default values for KP and KD.
        
        :param kp: Proportional gain (default is 0.15)
        :param kd: Derivative gain (default is 0.6)
        """
        self.KP = kp  # Proportional gain
        self.KD = kd  # Derivative gain
        self.previous_error = 0  # To store the previous error e[t-1]

    def compute_control_action(self, reference, output):
        """
        Compute the control action u[t] based on the reference and the output.
        
        :param reference: r[t], the desired reference value (set point)
        :param output: y[t], the actual output value (e.g., depth)
        :return: u[t], the control action at time t
        """
        # Calculate the current error
        error = reference - output  # e[t] = r[t] - y[t]
        
        # Calculate the control action based on the PD formula:
        # u[t] = KP * e[t] + KD * (e[t] - e[t-1])
        control_action = (self.KP * error) + (self.KD * (error - self.previous_error))
        
        # Update the previous error for the next time step
        self.previous_error = error
        
        return control_action
