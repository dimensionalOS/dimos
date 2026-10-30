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

"""Robot instance descriptions and the connection discovery contract.

These descriptions contain data and policy factories, never a robosuite model
or a running simulator. The world imports each physical model before starting.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Literal, Protocol, runtime_checkable

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.physics import RobotState
from dimos.spec.utils import Spec


class VelocityController(Protocol):
    """A robot-local locomotion policy; the world owns timing and motor writes."""

    def reset(self) -> None: ...

    def step(self, state: RobotState, command: Twist) -> MotorCommandArray: ...


VelocityFactory = Callable[[float], VelocityController]


@dataclass(frozen=True)
class ModelAsset:
    """Source asset compiled by a backend; its file format is explicit."""

    path: Path
    format: Literal["mjcf", "urdf"] = "mjcf"
    meshdir: Path | None = None
    initial_qpos: tuple[float, ...] = ()
    control_imu: str | None = None
    keyframe: str | None = None


@dataclass(frozen=True)
class RobotInstanceSpec:
    robot_id: str
    model: ModelAsset
    joint_names: tuple[str, ...]
    frame_id: str
    command_kind: Literal["motor", "velocity"] = "motor"
    base_name: str = ""
    velocity_controller: VelocityFactory | None = None
    default_scene: str = "logistics.xml"
    spawn: str = "default"
    position: tuple[float, float, float] | None = None
    rpy: tuple[float, float, float] | None = None
    timestep: float = 0.005
    startup_kp: tuple[float, ...] = ()
    startup_kd: tuple[float, ...] = ()
    wait_for_first_command: bool = False
    command_timeout: float = 0.25
    halt_damping: float | None = None
    position_scale: tuple[float, ...] = ()
    position_offset: tuple[float, ...] = ()
    position_only_joints: tuple[str, ...] = ()

    @property
    def control_imu(self) -> str | None:
        return self.model.control_imu

    def __post_init__(self) -> None:
        if (
            not self.robot_id
            or self.robot_id == "world"
            or any(not part.isidentifier() for part in self.robot_id.split("/"))
        ):
            raise ValueError(
                "robot_id must contain identifier path components; 'world' is reserved"
            )
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("public motor names must be nonempty and unique")
        if self.command_kind == "velocity" and self.velocity_controller is None:
            raise ValueError("velocity control requires a locomotion controller")
        if self.command_kind == "velocity" and not self.base_name:
            raise ValueError("velocity control requires the public base name")
        if self.command_kind == "motor" and self.velocity_controller is not None:
            raise ValueError("external motor control cannot also own a velocity controller")
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
        if not math.isfinite(self.timestep) or self.timestep <= 0:
            raise ValueError("timestep must be finite and positive")
        if self.halt_damping is not None and (
            not math.isfinite(self.halt_damping) or self.halt_damping < 0
        ):
            raise ValueError("halt_damping must be finite and nonnegative")
        for label, pose in (("position", self.position), ("rpy", self.rpy)):
            if pose is not None and (len(pose) != 3 or not all(map(math.isfinite, pose))):
                raise ValueError(f"{label} must contain three finite values")


@runtime_checkable
class RobotConnectionSpec(Spec, Protocol):
    """Build-time description; called before any robot or simulator is started."""

    @staticmethod
    def describe_simulated_robot(resolved_config: Any) -> RobotInstanceSpec: ...
