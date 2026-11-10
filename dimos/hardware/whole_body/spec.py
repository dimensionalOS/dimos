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


from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from dimos.hardware.spec import JointLimits

POS_STOP: float = 2.146e9
VEL_STOP: float = 16000.0


@dataclass(frozen=True)
class MotorCommand:
    q: float = POS_STOP
    dq: float = VEL_STOP
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0


@dataclass(frozen=True)
class MotorState:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0


@dataclass(frozen=True)
class IMUState:
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    gyroscope: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accelerometer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class WholeBodyConfig:
    kp: tuple[float, ...] | None = None
    kd: tuple[float, ...] | None = None


@runtime_checkable
class WholeBodyAdapter(Protocol):
    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def read_motor_states(self) -> list[MotorState]: ...
    def has_motor_states(self) -> bool: ...
    def motor_states_ts(self) -> float | None:
        return None

    def read_imu(self) -> IMUState: ...
    def get_limits(self) -> JointLimits | None: ...

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool: ...
