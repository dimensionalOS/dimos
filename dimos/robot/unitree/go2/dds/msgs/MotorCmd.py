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

"""unitree_go::msg::dds_::MotorCmd_ — one joint's command (target + gains)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MotorCmd:
    mode: int
    q: float  # target position (rad)
    dq: float  # target velocity (rad/s)
    tau: float  # feedforward torque (N·m)
    kp: float  # position gain
    kd: float  # velocity gain
    reserve: np.ndarray  # u32[3]

    __cdr_fields__ = [
        ("mode", "u8"),
        ("q", "f32"),
        ("dq", "f32"),
        ("tau", "f32"),
        ("kp", "f32"),
        ("kd", "f32"),
        ("reserve", ("array", "u32", 3)),
    ]
