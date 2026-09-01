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

"""unitree_go::msg::dds_::MotorState_"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MotorState:
    mode: int
    q: float
    dq: float
    ddq: float
    tau_est: float
    q_raw: float
    dq_raw: float
    ddq_raw: float
    temperature: int
    lost: int
    reserve: np.ndarray  # u32[2]

    __cdr_fields__ = [
        ("mode", "u8"),
        ("q", "f32"),
        ("dq", "f32"),
        ("ddq", "f32"),
        ("tau_est", "f32"),
        ("q_raw", "f32"),
        ("dq_raw", "f32"),
        ("ddq_raw", "f32"),
        ("temperature", "u8"),
        ("lost", "u32"),
        ("reserve", ("array", "u32", 2)),
    ]
