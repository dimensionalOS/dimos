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

from dataclasses import dataclass
from typing import Protocol

from dimos.core import In, Out
from dimos.msgs.geometry_msgs import WrenchStamped
from dimos.msgs.sensor_msgs import JointCommand, JointState


@dataclass
class RobotState:
    """Custom message containing full robot state (deprecated - use RobotStateMsg)."""

    state: int = 0  # Robot state (0: ready, 3: paused, 4: stopped, etc.)
    mode: int = 0  # Control mode (0: position, 1: servo, 4: joint velocity, 5: cartesian velocity)
    error_code: int = 0  # Error code
    warn_code: int = 0  # Warning code
    cmdnum: int = 0  # Command queue length
    mt_brake: int = 0  # Motor brake state
    mt_able: int = 0  # Motor enable state


class ArmDriverSpec(Protocol):
    """Protocol specification for xArm manipulator driver.

    Compatible with xArm5, xArm6, and xArm7 models.
    """

    # Input topics (commands)
    joint_position_command: In[JointCommand]  # Desired joint positions (radians)
    joint_velocity_command: In[JointCommand]  # Desired joint velocities (rad/s)

    # Output topics
    joint_state: Out[JointState]  # Current joint positions, velocities, and efforts
    robot_state: Out[RobotState]  # Full robot state (errors, modes, etc.)
    ft_ext: Out[WrenchStamped]  # External force/torque (compensated)
    ft_raw: Out[WrenchStamped]  # Raw force/torque sensor data

    # RPC Methods
    def set_joint_angles(self, angles: list[float]) -> tuple[int, str]: ...

    def set_joint_velocities(self, velocities: list[float]) -> tuple[int, str]: ...

    def get_joint_state(self) -> JointState: ...

    def get_robot_state(self) -> RobotState: ...

    def enable_servo_mode(self) -> tuple[int, str]: ...

    def disable_servo_mode(self) -> tuple[int, str]: ...
