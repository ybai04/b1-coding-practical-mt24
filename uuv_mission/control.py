# control.py

class PDController:
    def __init__(self, KP=0.15, KD=0.6):
        self.KP = KP  # Proportional gain
        self.KD = KD  # Derivative gain
        self.previous_error = 0  # To store e[t-1] for derivative term

    def compute(self, reference: float, observation: float) -> float:
        """Compute control action using PD controller."""
        # Calculate the error (e[t] = r[t] - y[t])
        error = reference - observation

        # Calculate derivative of the error (e[t] - e[t-1])
        error_derivative = error - self.previous_error

        # PD control law: u[t] = KP * e[t] + KD * (e[t] - e[t-1])
        control_action = self.KP * error + self.KD * error_derivative

        # Update previous error for the next time step
        self.previous_error = error

        return control_action
