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

"""unitree_go::msg::dds_::LowState_ — full low-level robot state (rt/lowstate)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.robot.unitree.go2dds.msgs.base import PrettyMsg
from dimos.robot.unitree.go2dds.msgs.BmsState import BmsState
from dimos.robot.unitree.go2dds.msgs.IMUState import IMUState
from dimos.robot.unitree.go2dds.msgs.MotorState import MotorState


@dataclass(repr=False)
class LowState(PrettyMsg):
    head: np.ndarray  # u8[2]
    level_flag: int
    frame_reserve: int
    sn: np.ndarray  # u32[2]
    version: np.ndarray  # u32[2]
    bandwidth: int
    imu_state: IMUState
    motor_state: list[MotorState]  # [20]
    bms_state: BmsState
    foot_force: np.ndarray  # i16[4]
    foot_force_est: np.ndarray  # i16[4]
    tick: int
    wireless_remote: np.ndarray  # u8[40]
    bit_flag: int
    adc_reel: float
    temperature_ntc1: int
    temperature_ntc2: int
    power_v: float
    power_a: float
    fan_frequency: np.ndarray  # u16[4]
    reserve: int
    # NOTE: the SDK's trailing `crc` (uint32) is absent on this Go2's firmware
    # wire format — verified against the recording (body ends after `reserve`).

    __cdr_fields__ = [
        ("head", ("array", "u8", 2)),
        ("level_flag", "u8"),
        ("frame_reserve", "u8"),
        ("sn", ("array", "u32", 2)),
        ("version", ("array", "u32", 2)),
        ("bandwidth", "u16"),
        ("imu_state", IMUState),
        ("motor_state", ("array", MotorState, 20)),
        ("bms_state", BmsState),
        ("foot_force", ("array", "i16", 4)),
        ("foot_force_est", ("array", "i16", 4)),
        ("tick", "u32"),
        ("wireless_remote", ("array", "u8", 40)),
        ("bit_flag", "u8"),
        ("adc_reel", "f32"),
        ("temperature_ntc1", "u8"),
        ("temperature_ntc2", "u8"),
        ("power_v", "f32"),
        ("power_a", "f32"),
        ("fan_frequency", ("array", "u16", 4)),
        ("reserve", "u32"),
    ]
