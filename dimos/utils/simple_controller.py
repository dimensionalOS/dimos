# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math


def normalize_angle(angle: float):
    """Normalize angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ----------------------------
# PID Controller Class
# ----------------------------
class PIDController:
    def __init__(
        self,
        kp,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits=(None, None),
        integral_limit=None,
        deadband: float = 0.0,
        output_deadband: float = 0.0,
        inverse_output: bool = False,
    ) -> None:
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
