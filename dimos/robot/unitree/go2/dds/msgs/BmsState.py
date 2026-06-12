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

"""unitree_go::msg::dds_::BmsState_ (battery management system)"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BmsState:
    version_high: int
    version_low: int
    status: int
    soc: int  # state of charge, %
    current: int
    cycle: int
    bq_ntc: np.ndarray  # u8[2], °C
    mcu_ntc: np.ndarray  # u8[2], °C
    cell_vol: np.ndarray  # u16[15], mV

    __cdr_fields__ = [
        ("version_high", "u8"),
        ("version_low", "u8"),
        ("status", "u8"),
        ("soc", "u8"),
        ("current", "i32"),
        ("cycle", "u16"),
        ("bq_ntc", ("array", "u8", 2)),
        ("mcu_ntc", ("array", "u8", 2)),
        ("cell_vol", ("array", "u16", 15)),
    ]
