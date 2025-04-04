import math

def normalize_angle(angle):
    """Normalize angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

# ----------------------------
# PID Controller Class
# ----------------------------
class PIDController:
    def __init__(self, kp, ki=0.0, kd=0.0, output_limits=(None, None), integral_limit=None, deadband=0.0, output_deadband=0.0, inverse_output=False):
        """
        Initialize the PID controller.
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple): (min_output, max_output). Use None for no limit.
            integral_limit (float): Maximum absolute value for the integral term (anti-windup).
            deadband (float): Size of the deadband region. Error smaller than this will be compensated.
            output_deadband (float): Deadband applied to the output to overcome physical system deadband.
            inverse_output (bool): When True, the output will be multiplied by -1.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output, self.max_output = output_limits
        self.integral_limit = integral_limit
        self.output_deadband = output_deadband
        self.deadband = deadband
        self.integral = 0.0
        self.prev_error = 0.0
        self.inverse_output = inverse_output

    def update(self, error, dt):
        """Compute the PID output with anti-windup, output deadband compensation and output saturation."""
        # Update integral term with windup protection.
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        
        # Compute derivative term.
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        if abs(error) < self.deadband:
            # Prevent integral windup by not increasing integral term when error is small.
            self.integral = 0.0
            derivative = 0.0
        
        # Compute raw output.
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Apply deadband compensation to the output
        output = self._apply_output_deadband_compensation(output)
        
        # Apply output limits if specified.
        if self.max_output is not None:
            output = min(self.max_output, output)
        if self.min_output is not None:
            output = max(self.min_output, output)
        
        self.prev_error = error
        if self.inverse_output:
            return -output
        return output
    
    def _apply_output_deadband_compensation(self, output):
        """
        Apply deadband compensation to the output.
        
        This simply adds the deadband value to the magnitude of the output
        while preserving the sign, ensuring we overcome the physical deadband.
        """
        if self.output_deadband == 0.0 or output == 0.0:
            return output
            
        if output > self.max_output * 0.05:
            # For positive output, add the deadband
            return output + self.output_deadband
        elif output < self.min_output * 0.05:
            # For negative output, subtract the deadband
            return output - self.output_deadband
        else:
            return output
    
    def _apply_deadband_compensation(self, error):
        """
        Apply deadband compensation to the error.
        
        This maintains the original error value, as the deadband compensation
        will be applied to the output, not the error.
        """
        return error

# ----------------------------
# Visual Servoing Controller Class
# ----------------------------
class VisualServoingController:
    def __init__(self, distance_pid_params, angle_pid_params):
        """
        Initialize the visual servoing controller using enhanced PID controllers.
        
        Args:
            distance_pid_params (tuple): (kp, ki, kd, output_limits, integral_limit, deadband) for distance.
            angle_pid_params (tuple): (kp, ki, kd, output_limits, integral_limit, deadband) for angle.
        """
        self.distance_pid = PIDController(*distance_pid_params)
        self.angle_pid = PIDController(*angle_pid_params)
        self.prev_measured_angle = 0.0  # Used for angular feed-forward damping

    def compute_control(self, measured_distance, measured_angle, desired_distance, desired_angle, dt):
        """
        Compute the forward (x) and angular (z) commands.
        
        Args:
            measured_distance (float): Current distance to target (from camera).
            measured_angle (float): Current angular offset to target (radians).
            desired_distance (float): Desired distance to target.
            desired_angle (float): Desired angular offset (e.g., 0 for centered).
            dt (float): Timestep.
        
        Returns:
            tuple: (forward_command, angular_command)
        """
        # Compute the errors.
        error_distance = measured_distance - desired_distance
        error_angle = normalize_angle(measured_angle - desired_angle)
        
        # Get raw PID outputs.
        forward_command_raw = self.distance_pid.update(error_distance, dt)
        angular_command_raw = self.angle_pid.update(error_angle, dt)

        #print("forward: {} angular: {}".format(forward_command_raw, angular_command_raw))
    
        angular_command = angular_command_raw

        # Couple forward command to angular error:
        # scale the forward command smoothly.
        scaling_factor = max(0.0, min(1.0, math.exp(-2.0 * abs(error_angle))))
        forward_command = forward_command_raw * scaling_factor

        return forward_command, angular_command