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

"""Proposed sim2 interface for review, not an implemented replacement API.

This file defines the values and contracts an implementation must support.
Simulation is the running module's application API. PhysicsBackend is the
engine adapter's synchronous API. The examples below send no commands at import.

Coordinate, timing, validation and lifecycle rules are in sim2-core-review.md.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal, Protocol
from uuid import UUID, uuid4

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.spec.utils import Spec

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # xyzw, unit quaternion
DriveKind = Literal["pd_effort", "native_position"]


@dataclass(frozen=True)
class StateId:
    epoch: UUID  # new on build/reset; old-run inputs cannot be reused
    tick: int  # completed public steps in this epoch; starts at zero


@dataclass(frozen=True)
class BodyState:
    pose: Pose  # world pose of the authored body/link origin
    twist: Twist  # world-expressed velocities AT THAT SAME ORIGIN


@dataclass(frozen=True)
class ScalarJointState:
    position: float  # rad for hinge, m for slider
    velocity: float  # rad/s or m/s
    actuator_effort: float | None  # Nm or N, final internal integration step


@dataclass(frozen=True)
class BallJointState:
    orientation: Quat  # child joint frame in parent joint frame
    angular_velocity: Vec3  # relative velocity, in CHILD joint axes
    actuator_torque: Vec3 | None  # Nm, same axes


@dataclass(frozen=True)
class WorldState:
    id: StateId
    sim_time: float  # seconds since this epoch began
    model_revision: str  # identifies the shared compiled description
    bodies: Mapping[str, BodyState]
    joints: Mapping[str, ScalarJointState | BallJointState]


@dataclass(frozen=True)
class JointInfo:
    kind: Literal["revolute", "prismatic", "spherical", "fixed"]
    drive: DriveKind | None  # None means unactuated in this physics profile


@dataclass(frozen=True)
class WorldDescription:
    model_revision: str
    step_dt: float  # one public step in simulated seconds; fixed while loaded
    bodies: Mapping[str, Literal["dynamic", "static", "kinematic"]]
    joints: Mapping[str, JointInfo]
    # IDs refer to the shared model; no second copy of origins, axes or mounts.


@dataclass(frozen=True)
class JointPD:
    joint: str
    position: float
    velocity: float
    kp: float
    kd: float
    feedforward: float


@dataclass(frozen=True)
class JointPosition:
    joint: str
    position: float  # native servo retains authored gains and dynamics


@dataclass(frozen=True)
class BodyWrench:
    body: str
    force_world: Vec3 = (0.0, 0.0, 0.0)
    torque_world: Vec3 = (0.0, 0.0, 0.0)
    point_world: Vec3 | None = None  # None means the body's current CoM


@dataclass(frozen=True)
class StepInput:
    from_state: StateId
    drives: tuple[JointPD | JointPosition, ...]
    wrenches: tuple[BodyWrench, ...] = ()


class PhysicsBackend(Protocol):
    """Engine implementer API, called only by the simulation owner.

    The existing composition factory supplies model instances and scene assets
    before build. This proposal does not introduce another model-file format.
    """

    def build(self) -> WorldDescription: ...
    def read_state(self) -> WorldState: ...
    def step(self, inputs: StepInput) -> WorldState: ...
    def reset(self) -> WorldState: ...
    def close(self) -> None: ...


PhysicsCommand = JointPD | JointPosition | BodyWrench


@dataclass(frozen=True)
class CommandBatch:
    command_id: str
    epoch: UUID
    commands: tuple[PhysicsCommand, ...]
    duration: float  # simulated seconds; 0 means one public physics step


@dataclass(frozen=True)
class CommandReport:
    command_id: str
    state: StateId  # state at acceptance/rejection, or completed affected tick
    status: Literal["accepted", "applied", "rejected", "expired", "cancelled"]
    reason: str = ""


class Simulation(Spec, Protocol):
    """Application-facing RPCs of a running SimulationModule.

    The module additionally publishes world_state: Out[WorldState] and
    command_reports: Out[CommandReport] through normal DimOS streams.
    Applications never call PhysicsBackend.step or build a complete StepInput.
    """

    def describe_world(self) -> WorldDescription: ...
    def read_state(self) -> WorldState: ...
    def submit(self, batch: CommandBatch) -> CommandReport: ...
    def set_paused(self, paused: bool) -> None: ...
    def reset(self) -> None: ...


def push_g1(sim: Simulation) -> CommandReport:
    """Push the pelvis with 20 N along world +X for 100 ms of simulation time.

    G1's existing controller continues supplying all of its motor targets.
    """
    state = sim.read_state()
    return sim.submit(
        CommandBatch(
            command_id=uuid4().hex,
            epoch=state.id.epoch,
            commands=(BodyWrench("g1/pelvis", force_world=(20.0, 0.0, 0.0)),),
            duration=0.1,
        )
    )


def push_box_and_open_passive_door(sim: Simulation) -> CommandReport:
    """For the G1/box/door conformance scene, not the current static store assets."""
    state = sim.read_state()
    return sim.submit(
        CommandBatch(
            command_id=uuid4().hex,
            epoch=state.id.epoch,
            commands=(
                BodyWrench("box", force_world=(20.0, 0.0, 0.0)),
                BodyWrench("door_leaf", torque_world=(0.0, 0.0, 3.0)),
            ),
            duration=0.1,
        )
    )


def g1_pd_drives(
    motor_joint_ids: tuple[str, ...],
    command: MotorCommandArray,
) -> tuple[JointPD, ...]:
    """Inside the G1 adapter: map all 29 GR00T motor commands into model IDs.

    The mapping is resolved once from the connection's hardware motor order.
    The current G1 profile uses identity joint scale/offset. Other robot adapters
    must convert hardware coordinates and effort into model units/signs first.
    This is not called by an application submitting a scene force.
    """
    if len(motor_joint_ids) != 29 or command.num_joints != 29:
        raise ValueError("G1 GR00T requires a complete 29-motor command")
    return tuple(
        JointPD(joint, q, dq, kp, kd, tau)
        for joint, q, dq, kp, kd, tau in zip(
            motor_joint_ids,
            command.q,
            command.dq,
            command.kp,
            command.kd,
            command.tau,
            strict=True,
        )
    )


def g1_position_drives(
    motor_joint_ids: tuple[str, ...],
    command: MotorCommandArray,
) -> tuple[JointPosition, ...]:
    """Inside the stock G1 adapter: its policy's targets drive native servos."""
    if len(motor_joint_ids) != 29 or command.num_joints != 29:
        raise ValueError("Stock G1 requires a complete 29-motor command")
    return tuple(
        JointPosition(joint, q) for joint, q in zip(motor_joint_ids, command.q, strict=True)
    )
