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

"""Dynamics contract: commands accepted by an in-process physics owner."""

from dataclasses import dataclass
from typing import Protocol

from dimos.sim2.state import Frame, Layout


@dataclass(frozen=True)
class JointEfforts:
    epoch: int
    joints: tuple[str, ...]
    efforts: tuple[float, ...]
    ttl: float = 0.25


@dataclass(frozen=True)
class JointTargets:
    epoch: int
    joints: tuple[str, ...]
    positions: tuple[float, ...]
    ttl: float = 0.25  # simulation seconds; expiration holds measured position
    # Empty fields use zero velocity/feedforward and the world's Drive gains.
    # effort = kp*(position-q) + kd*(velocity-dq) + feedforward, at each physics step.
    velocities: tuple[float, ...] = ()
    kp: tuple[float, ...] = ()
    kd: tuple[float, ...] = ()
    feedforward: tuple[float, ...] = ()
    tick: int | None = None  # the feedback tick answered by a paced controller


@dataclass(frozen=True)
class Wrench:
    epoch: int
    body: str
    force: tuple[float, float, float]
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
    point: tuple[float, float, float] | None = None  # world point; None = link origin
    duration: float = 0.1  # simulation seconds, not a persistent external force


class PhysicsBackend(Protocol):
    """One in-process dynamics owner. The caller serializes these operations.

    Position servos and direct joint effort are the actuation capabilities of v1.
    A future backend must satisfy the behavior tests, not just these signatures.
    """

    layout: Layout
    dt: float

    def targets(self, command: JointTargets) -> None:
        """Validate every entry before changing any drive, then latch targets."""
        ...

    def efforts(self, command: JointEfforts) -> None: ...
    def hold(self, joints: tuple[str, ...]) -> None: ...
    def validate_wrench(self, wrench: Wrench) -> None: ...
    def step(self, wrenches: tuple[Wrench, ...]) -> None: ...
    def snapshot(self, epoch: int, tick: int) -> Frame: ...
    def reset(self) -> None: ...
    def place(self, body: str, pose: tuple[float, ...]) -> None:
        """Privileged scene edit: free root only, zero its velocity; not robot control."""
        ...
