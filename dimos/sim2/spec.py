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

"""Hardware-interface bindings for upstream robosuite models.

Robot anatomy, joints, actuators and placement belong to the upstream model.
Only public motor names, units and connection startup/watchdog settings live here.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math

from robosuite.models.robots.robot_model import RobotModel


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    model: Callable[[], RobotModel]
    joint_names: tuple[str, ...]
    frame_id: str
    control_imu: str | None = None
    startup_kp: tuple[float, ...] = ()
    startup_kd: tuple[float, ...] = ()
    wait_for_first_command: bool = False
    command_timeout: float = 0.25
    halt_damping: float | None = None
    position_scale: tuple[float, ...] = ()
    position_offset: tuple[float, ...] = ()
    position_only_joints: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.robot_id.isidentifier() or self.robot_id == "world":
            raise ValueError("robot_id must be an identifier; 'world' is reserved")
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("public motor names must be nonempty and unique")
        if not set(self.position_only_joints) <= set(self.joint_names):
            raise ValueError("position-only joints must name public motors")
        for label, values in (
            ("startup_kp", self.startup_kp),
            ("startup_kd", self.startup_kd),
            ("position_scale", self.position_scale),
            ("position_offset", self.position_offset),
        ):
            if values and (
                len(values) != len(self.joint_names) or not all(map(math.isfinite, values))
            ):
                raise ValueError(f"{label} must contain one finite value per motor")
        if any(v < 0 for v in (*self.startup_kp, *self.startup_kd)):
            raise ValueError("startup gains must be nonnegative")
        if any(v == 0 for v in self.position_scale):
            raise ValueError("motor coordinate scales must be nonzero")
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError("command_timeout must be finite and positive")
        if self.halt_damping is not None and (
            not math.isfinite(self.halt_damping) or self.halt_damping < 0
        ):
            raise ValueError("halt_damping must be finite and nonnegative")
