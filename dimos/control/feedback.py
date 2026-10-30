# Copyright 2026 Dimensional Inc.
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

"""Coherent robot feedback and the motor command computed from that sample.

The epoch changes when state is discontinuous (for example a simulator reset).
Ticks are opaque sequence numbers, not wall-clock timestamps. A connection must
preserve the reply's identity instead of attaching its newest sample to old work.
"""

from dataclasses import dataclass
from typing import Protocol

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.spec.utils import Spec


@dataclass(frozen=True)
class ControlFeedback:
    epoch: str
    tick: int
    time: float  # source clock, simulation seconds or monotonic hardware time
    dt: float
    joints: JointState
    imu: Imu | None = None


@dataclass(frozen=True)
class MotorControl:
    epoch: str
    tick: int
    joints: tuple[str, ...]
    motors: MotorCommandArray


class VelocityControllerSpec(Spec, Protocol):
    def set_velocity_command(self, twist: Twist, duration: float = 0.0) -> None: ...
