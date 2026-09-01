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

"""unitree_go::msg::dds_::IMUState_"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class IMUState:
    quaternion: np.ndarray  # f32[4], w x y z
    gyroscope: np.ndarray  # f32[3]
    accelerometer: np.ndarray  # f32[3]
    rpy: np.ndarray  # f32[3]
    temperature: int

    __cdr_fields__ = [
        ("quaternion", ("array", "f32", 4)),
        ("gyroscope", ("array", "f32", 3)),
        ("accelerometer", ("array", "f32", 3)),
        ("rpy", ("array", "f32", 3)),
        ("temperature", "u8"),
    ]
