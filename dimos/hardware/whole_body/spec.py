# Copyright 2025-2026 Dimensional Inc.
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

"""WholeBodyAdapter protocol for joint-level motor control.

Lightweight protocol for robots that expose per-motor
position/velocity/torque control (as opposed to TwistBaseAdapter which
only exposes velocity commands).

Supports any number of motors — quadrupeds (12 DOF), humanoids (29 DOF), etc.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

# Sentinel values from Unitree SDK — used to signal "no command" for a DOF.
POS_STOP: float = 2.146e9
VEL_STOP: float = 16000.0


@dataclass(frozen=True)
class MotorCommand:
    """Command for a single motor."""

    q: float = POS_STOP  # target position (rad)
    dq: float = VEL_STOP  # target velocity (rad/s)
    kp: float = 0.0  # position gain
    kd: float = 0.0  # velocity gain
    tau: float = 0.0  # feedforward torque (Nm)


@dataclass(frozen=True)
class MotorState:
    """Feedback from a single motor."""

    q: float = 0.0  # position (rad)
    dq: float = 0.0  # velocity (rad/s)
    tau: float = 0.0  # estimated torque (Nm)


@dataclass(frozen=True)
class IMUState:
    """IMU feedback."""

    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    gyroscope: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accelerometer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@runtime_checkable
class WholeBodyAdapter(Protocol):
    """Protocol for joint-level whole-body motor IO.

    Implement this per vendor SDK.  All methods use SI units:
    - Position: radians
    - Velocity: rad/s
    - Torque: Nm
    - Force: N
    """

    # --- Connection ---

    def connect(self) -> bool:
        """Connect to hardware. Returns True on success."""
        ...

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        ...

    def is_connected(self) -> bool:
        """Check if connected."""
        ...

    # --- State Reading ---

    def read_motor_states(self) -> list[MotorState]:
        """Read motor states for all joints."""
        ...

    def read_imu(self) -> IMUState:
        """Read IMU state."""
        ...

    # --- Control ---

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        """Write motor commands for all joints. Returns success."""
        ...


__all__ = [
    "POS_STOP",
    "VEL_STOP",
    "IMUState",
    "MotorCommand",
    "MotorState",
    "WholeBodyAdapter",
]
