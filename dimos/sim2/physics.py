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

"""Rigid-body physics contract, independent of an engine and its checkpoint format.

Positions are metres, angles radians, forces newtons and torques newton-metres.
Body poses and velocities use the world frame and the body origin; quaternions
are xyzw. Stable instance/link and instance/joint names never contain engine IDs.
Commands are active for one backend step; the module owns expiry and repetition.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Protocol

from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.world.scene_types import SceneDescription

if TYPE_CHECKING:
    from dimos.sim2.visuals import VisualScene

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class BodyState:
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3
    angular_velocity: Vec3


@dataclass(frozen=True)
class JointState:
    """Scalar q/dq, or ball xyzw q and local angular dq.

    Actuator effort is the generalized effort applied in the last integration
    interval, excluding contacts/reactions; None means the backend cannot report it.
    """

    position: tuple[float, ...]
    velocity: tuple[float, ...]
    actuator_effort: tuple[float, ...] | None = None


@dataclass(frozen=True)
class InertialState:
    """World orientation and sensor-local velocities/specific acceleration."""

    orientation: Quat
    angular_velocity: Vec3
    linear_velocity: Vec3
    linear_acceleration: Vec3


@dataclass(frozen=True)
class RobotState:
    """Hardware-unit projection in the connection's declared motor order."""

    joint_names: tuple[str, ...]
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    effort: tuple[float, ...]
    base: BodyState
    imu: InertialState | None = None


@dataclass(frozen=True)
class WorldState:
    metadata: FrameMetadata
    bodies: dict[str, BodyState]
    joints: dict[str, JointState]
    robots: dict[str, RobotState]
    scene_revision: int = 0
    wall_time: float = 0.0


@dataclass(frozen=True)
class BodyDescription:
    entity: str
    motion: Literal["dynamic", "static", "kinematic"]


@dataclass(frozen=True)
class JointDescription:
    parent: str
    child: str
    kind: Literal["revolute", "prismatic", "spherical", "fixed"]
    actuated: bool = False


@dataclass(frozen=True)
class WorldDescription:
    scene: SceneDescription
    bodies: dict[str, BodyDescription]
    joints: dict[str, JointDescription]
    robot_roots: dict[str, str]
    robot_joints: dict[str, tuple[str, ...]]
    capabilities: frozenset[str]


@dataclass(frozen=True)
class JointDrive:
    """Drive a declared scalar actuator; q/dq/kp/kd/effort use joint units."""

    joint: str
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    effort: float = 0.0


@dataclass(frozen=True)
class BodyWrench:
    """One-step wrench; point=None applies at CoM, otherwise in the chosen frame."""

    body: str
    force: Vec3 = (0.0, 0.0, 0.0)
    torque: Vec3 = (0.0, 0.0, 0.0)
    frame: Literal["world", "body"] = "world"
    point: Vec3 | None = None


@dataclass(frozen=True)
class KinematicTarget:
    """World pose for a declared kinematic body, never a dynamic-body teleport."""

    body: str
    position: Vec3
    orientation: Quat


PhysicsCommand = JointDrive | BodyWrench | KinematicTarget


@dataclass(frozen=True)
class CommandBatch:
    command_id: str
    epoch: int
    commands: tuple[PhysicsCommand, ...]
    duration: float = 0.0  # zero means exactly one step; positive uses sim seconds


@dataclass(frozen=True)
class CommandReport:
    command_id: str
    epoch: int
    tick: int
    status: Literal["accepted", "applied", "rejected", "expired"]
    reason: str = ""


@dataclass(frozen=True)
class BackendStep:
    """Optional task/sensor results; neither determines the physical state schema."""

    observations: dict[str, Any] = field(default_factory=dict)
    reward: float = 0.0
    done: bool = False
    info: dict[str, Any] = field(default_factory=dict)


class PhysicsBackend(Protocol):
    """One owner thread builds, resets, commands, reads and closes the backend."""

    @property
    def name(self) -> str: ...
    @property
    def step_seconds(self) -> float: ...
    @property
    def physics_dt(self) -> float: ...
    @property
    def sim_time(self) -> float: ...
    @property
    def done(self) -> bool: ...
    @property
    def description(self) -> WorldDescription: ...

    def build(self) -> None: ...
    def reset(self) -> BackendStep: ...
    def validate(self, commands: tuple[PhysicsCommand, ...]) -> None: ...
    def step(self, commands: tuple[PhysicsCommand, ...]) -> BackendStep: ...
    def read_state(self, metadata: FrameMetadata) -> WorldState: ...
    def set_task_action(self, action: list[float]) -> None: ...
    def describe_visuals(self) -> VisualScene: ...
    def sensor_description(self) -> dict[str, Any]: ...
    def checkpoint(self, metadata: FrameMetadata, wall_time: float) -> StateFrame | None: ...
    def close(self) -> None: ...
