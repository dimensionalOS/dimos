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

"""unitree_go::msg::dds_::LowCmd_ — low-level motor commands (rt/lowcmd).

The actuation counterpart to :class:`~dimos.robot.unitree.go2.dds.msgs.LowState.LowState`:
20 per-joint targets (position/velocity/torque + kp/kd gains) sent to the robot.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.robot.unitree.go2.dds.msgs.base import PrettyMsg
from dimos.robot.unitree.go2.dds.msgs.BmsCmd import BmsCmd
from dimos.robot.unitree.go2.dds.msgs.MotorCmd import MotorCmd


@dataclass(repr=False)
class LowCmd(PrettyMsg):
    head: np.ndarray  # u8[2]
    level_flag: int
    frame_reserve: int
    sn: np.ndarray  # u32[2]
    version: np.ndarray  # u32[2]
    bandwidth: int
    motor_cmd: list[MotorCmd]  # [20]
    bms_cmd: BmsCmd
    wireless_remote: np.ndarray  # u8[40]
    led: np.ndarray  # u8[12]
    fan: np.ndarray  # u8[2]
    gpio: int
    reserve: int
    # NOTE: the SDK's trailing `crc` (uint32) is absent on this Go2's firmware
    # wire format — verified against the recording (body ends after `reserve`),
    # matching the same omission in LowState.

    __cdr_fields__ = [
        ("head", ("array", "u8", 2)),
        ("level_flag", "u8"),
        ("frame_reserve", "u8"),
        ("sn", ("array", "u32", 2)),
        ("version", ("array", "u32", 2)),
        ("bandwidth", "u16"),
        ("motor_cmd", ("array", MotorCmd, 20)),
        ("bms_cmd", BmsCmd),
        ("wireless_remote", ("array", "u8", 40)),
        ("led", ("array", "u8", 12)),
        ("fan", ("array", "u8", 2)),
        ("gpio", "u8"),
        ("reserve", "u32"),
    ]
