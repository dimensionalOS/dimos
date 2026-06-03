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

"""unitree_go::msg::dds_::SportModeState_ — high-level sport state (rt/sportmodestate)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.robot.unitree.go2dds.msgs.base import PrettyMsg
from dimos.robot.unitree.go2dds.msgs.IMUState import IMUState
from dimos.robot.unitree.go2dds.msgs.PathPoint import PathPoint
from dimos.robot.unitree.go2dds.msgs.TimeSpec import TimeSpec


@dataclass(repr=False)
class SportModeState(PrettyMsg):
    stamp: TimeSpec
    error_code: int
    imu_state: IMUState
    mode: int
    progress: float
    gait_type: int
    foot_raise_height: float
    position: np.ndarray  # f32[3]
    body_height: float
    velocity: np.ndarray  # f32[3]
    yaw_speed: float
    range_obstacle: np.ndarray  # f32[4]
    foot_force: np.ndarray  # i16[4]
    foot_position_body: np.ndarray  # f32[12]
    foot_speed_body: np.ndarray  # f32[12]
    path_point: list[PathPoint]  # [10]

    __cdr_fields__ = [
        ("stamp", TimeSpec),
        ("error_code", "u32"),
        ("imu_state", IMUState),
        ("mode", "u8"),
        ("progress", "f32"),
        ("gait_type", "u8"),
        ("foot_raise_height", "f32"),
        ("position", ("array", "f32", 3)),
        ("body_height", "f32"),
        ("velocity", ("array", "f32", 3)),
        ("yaw_speed", "f32"),
        ("range_obstacle", ("array", "f32", 4)),
        ("foot_force", ("array", "i16", 4)),
        ("foot_position_body", ("array", "f32", 12)),
        ("foot_speed_body", ("array", "f32", 12)),
        ("path_point", ("array", PathPoint, 10)),
    ]

    def to_rerun(self) -> Any:
        """Sport-mode pose as a rerun Transform3D (position + body orientation)."""
        import rerun as rr

        w, x, y, z = (float(v) for v in self.imu_state.quaternion)  # Unitree order: wxyz
        return rr.Transform3D(
            translation=[float(v) for v in self.position],
            rotation=rr.Quaternion(xyzw=[x, y, z, w]),
        )
