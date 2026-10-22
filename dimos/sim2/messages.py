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

"""Internal simulator messages. Joint order and units come from RobotConfig.

These are Python messages over DimOS transport, not a shared-memory ABI.
World snapshots are privileged MuJoCo integration state, not robot observations.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.spec import RobotInstance

Lifecycle = Literal["starting", "ready", "faulted", "closed"]


@dataclass(frozen=True)
class StreamDescriptor:
    sim_id: str
    robot_id: str
    generation: str
    physics_dt: float
    control_decimation: int = 1
    observation_slots: int = 2
    joint_names: tuple[str, ...] = ()
    actuator_modes: tuple[Literal["effort", "position"], ...] = ()
    position_gains: tuple[tuple[float, float], ...] = ()
    control_imu: str | None = None

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def topic(self) -> str:
        suffix = f"robots/{self.robot_id}" if self.dof else "world"
        return f"sim2/{self.generation}/{suffix}"

    @property
    def fingerprint(self) -> str:
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> StreamDescriptor:
        return cls(
            **{
                **value,
                "joint_names": tuple(value["joint_names"]),
                "actuator_modes": tuple(value["actuator_modes"]),
                "position_gains": tuple(tuple(g) for g in value["position_gains"]),
            }
        )


def robot_descriptor(
    sim_id: str, robot_id: str, generation: str, instance: RobotInstance, timestep: float
) -> StreamDescriptor:
    return StreamDescriptor(
        sim_id,
        robot_id,
        generation,
        timestep,
        joint_names=tuple(j.name for j in instance.config.joints),
        actuator_modes=tuple(j.mode for j in instance.config.joints),
        position_gains=tuple((j.kp, j.kd) for j in instance.config.joints),
        control_imu=instance.control_imu,
    )


@dataclass(frozen=True)
class FrameMetadata:
    """Internal transport stamp; episode_id is the command/sensor generation.

    Unlike SceneState.episode, it also changes on an edit or resume.
    """

    sequence: int
    episode_id: int
    physics_tick: int
    control_tick: int
    sim_time: float
    applied_action_sequence: int = 0


@dataclass(frozen=True)
class StateFrame:
    metadata: FrameMetadata
    values: dict[str, NDArray[np.float64]]
    lifecycle: Lifecycle = "ready"
    reset_at: float = 0.0


@dataclass(frozen=True)
class MotorFrame:
    metadata: FrameMetadata
    values: dict[str, NDArray[np.float64]]
    writer: str
    source_ts: float
