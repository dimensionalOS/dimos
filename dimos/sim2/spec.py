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

"""Robot definitions contain device bindings; instances contain world placement."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
from typing import Any, ClassVar, Literal, Protocol, runtime_checkable

import mujoco
from pydantic import BaseModel

from dimos.sim2.sensors.composition import Sensor, mounted_sensors
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.spec import Mount as Mount


@dataclass(frozen=True)
class Joint:
    name: str
    model_name: str
    actuator: str
    # Fixed servo gains for position requests to effort actuators. PD motor
    # commands supply their own gains; controller gains do not belong here.
    kp: float = 0.0
    kd: float = 0.0
    # Native position servos retain their own gains. Motor actuators use PD once.
    mode: Literal["effort", "position"] = "effort"
    # Public joint coordinate -> MJCF joint coordinate, including gripper units.
    scale: float = 1.0
    offset: float = 0.0
    ctrl_scale: float = 1.0
    ctrl_offset: float = 0.0
    lower: float = -6.283185307179586
    upper: float = 6.283185307179586
    max_velocity: float = 3.141592653589793
    actuator_time_constant: float = 0.0

    def __post_init__(self) -> None:
        if self.scale == 0 or not all(
            math.isfinite(v)
            for v in (
                self.scale,
                self.offset,
                self.ctrl_scale,
                self.ctrl_offset,
                self.kp,
                self.kd,
                self.lower,
                self.upper,
                self.max_velocity,
                self.actuator_time_constant,
            )
        ):
            raise ValueError(f"joint {self.name!r} requires finite parameters and nonzero scale")
        if self.lower > self.upper or self.max_velocity < 0 or min(self.kp, self.kd) < 0:
            raise ValueError(f"joint {self.name!r} has invalid limits or gains")
        if self.actuator_time_constant < 0 or (
            self.actuator_time_constant and self.mode != "effort"
        ):
            raise ValueError(
                "actuator lag requires an effort motor and a nonnegative time constant"
            )


@dataclass(frozen=True)
class RobotConfig:
    model: Path
    root_body: str
    joints: tuple[Joint, ...]
    sensors: tuple[Sensor, ...] = ()
    meshdir: Path | None = None
    floating: bool = False
    # Root height above a scene's support pose; explicit instances stay absolute.
    spawn_height: float = 0.0
    # Preserve calibrated dynamics when a fit was applied to a compiled model.
    configure_physics: Callable[[mujoco.MjModel, str], None] | None = None
    max_timestep: float = 0.005

    def __post_init__(self) -> None:
        if not math.isfinite(self.spawn_height) or self.spawn_height < 0:
            raise ValueError("robot spawn height must be finite and nonnegative")
        if not math.isfinite(self.max_timestep) or self.max_timestep <= 0:
            raise ValueError("robot max_timestep must be finite and positive")
        if not self.joints:
            raise ValueError("robot must declare its controlled joints")
        for label, names in (
            ("joint", [j.name for j in self.joints]),
            ("actuator", [j.actuator for j in self.joints]),
            ("sensor", [s.name for s in self.mounted_sensors]),
        ):
            if len(set(names)) != len(names) or any(not name for name in names):
                raise ValueError(f"robot {label} names must be nonempty and unique")
        if any(not math.isfinite(s.rate_hz) or s.rate_hz <= 0 for s in self.mounted_sensors):
            raise ValueError("sensor rates must be finite and positive")

    @property
    def mounted_sensors(self) -> tuple[Sensor, ...]:
        """Physical mounts, including IMUs supplied as part of a lidar device."""
        return mounted_sensors(self.sensors)

    def with_sensor(self, sensor: Sensor) -> RobotConfig:
        sensors = tuple(s for s in self.sensors if s.name != sensor.name)
        return replace(self, sensors=(*sensors, sensor))


@dataclass(frozen=True)
class RobotInstance:
    config: RobotConfig
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_positions: dict[str, float] = field(default_factory=dict)
    # Explicit controller feedback and startup/watchdog settings, not anatomy.
    control_imu: str | None = None
    wait_for_first_command: bool = False
    hold_gains: dict[str, tuple[float, float]] = field(default_factory=dict)
    # Identity/configuration of the external controller; never executed by physics.
    controller: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        joints = {joint.name: joint for joint in self.config.joints}
        unknown = (self.initial_positions.keys() | self.hold_gains.keys()) - joints.keys()
        if unknown:
            raise ValueError(f"unknown initial/holding joints: {sorted(unknown)}")
        for name, position in self.initial_positions.items():
            joint = joints[name]
            if not math.isfinite(position) or not joint.lower <= position <= joint.upper:
                raise ValueError(f"initial position for {name!r} is outside joint limits")
        if any(
            len(gains) != 2 or any(not math.isfinite(v) or v < 0 for v in gains)
            for gains in self.hold_gains.values()
        ):
            raise ValueError("holding gains require finite nonnegative kp/kd pairs")
        imus = {sensor.name for sensor in self.config.mounted_sensors if isinstance(sensor, Imu)}
        if self.control_imu is not None and self.control_imu not in imus:
            raise ValueError(f"unknown control IMU {self.control_imu!r}")
        if any(s.name == self.control_imu and s.latency for s in self.config.mounted_sensors):
            raise ValueError("coherent control IMU feedback does not support a delayed measurement")


@runtime_checkable
class SimRobotSpec(Protocol):
    """A connection describes its simulated device without opening real hardware."""

    default_sim_scene: ClassVar[str]

    @staticmethod
    def describe_simulated_robot(config: BaseModel, scene: Path) -> RobotInstance: ...


@dataclass(frozen=True)
class WorldConfig:
    scene: Path
    robots: dict[str, RobotInstance]
    timestep: float = 0.005
    snapshot_hz: float = 60.0
    sensors: tuple[Sensor, ...] = ()
    # Parameter path -> evidence URI/revision. Empty means no calibration claim.
    calibration_evidence: dict[str, str] = field(default_factory=dict)
    # Caller-sampled variation, including its distribution, seed and sampled value.
    # This records sampling; it does not change parameters or seed external RNGs.
    sampled_variation: dict[str, Any] = field(default_factory=dict)

    @property
    def devices(self) -> tuple[tuple[str | None, Sensor], ...]:
        """Independently acquired devices; the selected control IMU travels with motor feedback."""
        return (
            *((None, sensor) for sensor in self.sensors),
            *(
                (robot_id, sensor)
                for robot_id, instance in self.robots.items()
                for sensor in instance.config.sensors
                if not (isinstance(sensor, Imu) and sensor.name == instance.control_imu)
            ),
        )

    def __post_init__(self) -> None:
        if any(
            not name or name.split("/")[0] == "world" or any(not part for part in name.split("/"))
            for name in self.robots
        ):
            raise ValueError("world requires nonempty robot ID segments; 'world' is reserved")
        if any(not math.isfinite(v) or v <= 0 for v in (self.timestep, self.snapshot_hz)):
            raise ValueError("world timestep and snapshot rate must be finite and positive")
        sensors = mounted_sensors(self.sensors)
        names = [sensor.name for sensor in sensors]
        if len(set(names)) != len(names) or any(not name for name in names):
            raise ValueError("world sensor names must be nonempty and unique")
        if any(not math.isfinite(s.rate_hz) or s.rate_hz <= 0 for s in sensors):
            raise ValueError("sensor rates must be finite and positive")
