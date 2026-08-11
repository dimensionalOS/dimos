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

"""WholeBodyAdapter protocol for joint-level (q/dq/kp/kd/tau) motor control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

# Unitree SDK sentinels meaning "no command" for that DOF.
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


@dataclass(frozen=True)
class WholeBodyConfig:
    """Whole-body-specific component config.

    Lives on ``HardwareComponent.wb_config`` for components of type WHOLE_BODY.
    Keeps PD gains (and any future whole-body-only knobs) off the generic
    HardwareComponent shared by all hardware kinds.

    Attributes:
        kp: Per-joint position gains used by ConnectedWholeBody when
            translating position commands to MotorCommand. Length must
            match the component's ``joints`` list when set.
        kd: Per-joint velocity gains. Same length constraint.
    """

    kp: tuple[float, ...] | None = None
    kd: tuple[float, ...] | None = None
    # Gravity feedforward: fills MotorCommand.tau for ``gravity_ff_joints``
    # from RNEA on ``gravity_ff_model`` (URDF or MJCF; in sim, the sim's own
    # MJCF makes the model masses match exactly) at the measured
    # configuration. ``gravity_ff_joint_map`` maps component joint names to
    # model joint names (all joints that should shape the model's q, not
    # just the ff set); ``gravity_ff_payloads`` adds (model_frame, kg) point
    # masses, e.g. a carried jug at the palm.
    gravity_ff_model: str | None = None
    gravity_ff_joint_map: tuple[tuple[str, str], ...] = ()
    gravity_ff_joints: tuple[str, ...] = ()
    gravity_ff_payloads: tuple[tuple[str, float], ...] = ()
    # Global multiplier on the model torques; calibrate against measured
    # sag since model masses rarely match the plant (sim or hardware).
    gravity_ff_scale: float = 1.0


@runtime_checkable
class WholeBodyAdapter(Protocol):
    """Joint-level whole-body motor IO. SI units (rad, rad/s, Nm)."""

    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def read_motor_states(self) -> list[MotorState]: ...
    def has_motor_states(self) -> bool: ...
    def read_imu(self) -> IMUState: ...

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool: ...
