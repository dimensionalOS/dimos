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

"""Synthetic bottle-cap commands for the simulation; no hardware or policy model."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import suppress
import math
import time

from pydantic import Field, FiniteFloat

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.takeover import Weights6, distance_to_target, quadratic


def bottle_cap_takeover(pose: Transform) -> Weights6:
    """Human lifts along world Z; policy mainly rotates about world Z.

    Approaching the example cap at (0.4, 0, 0.4) increases policy authority.
    These are velocity weights, independent of the TCP's quaternion components.
    """
    alpha = quadratic(distance_to_target(pose, (0.4, 0.0, 0.4), outer_radius=0.5))
    return (alpha, alpha, 0.1 * alpha, alpha, alpha, 0.8 + 0.2 * alpha)


def bottle_cap_commands(
    elapsed: float, duration_s: float, world_frame: str
) -> tuple[TwistStamped, TwistStamped]:
    """One gentle lift-and-return and twist-and-return, then zero velocities."""
    speed = math.sin(2 * math.pi * elapsed / duration_s) if 0 <= elapsed < duration_s else 0.0
    return (
        TwistStamped(
            frame_id=world_frame, linear=[0, 0, 0.02 * speed], angular=[0, 0, 0.02 * speed]
        ),
        TwistStamped(
            frame_id=world_frame, linear=[0, 0, 0.002 * speed], angular=[0, 0, 0.3 * speed]
        ),
    )


class SyntheticVelocityConfig(ModuleConfig):
    world_frame: str = Field(default="world", min_length=1)
    control_hz: FiniteFloat = Field(default=30.0, gt=0)
    duration_s: FiniteFloat = Field(default=8.0, gt=0)


class SyntheticVelocitySources(Module):
    """Publish both sources after measured feedback first arrives; stop after one cycle."""

    config: SyntheticVelocityConfig
    current_pose: In[Transform]
    teleop_command: Out[TwistStamped]
    policy_command: Out[TwistStamped]
    _started_at: float | None = None

    async def handle_current_pose(self, pose: Transform) -> None:
        if self._started_at is None:
            self._started_at = time.monotonic()

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            deadline = loop.time() + 1 / self.config.control_hz
            elapsed = -1.0 if self._started_at is None else time.monotonic() - self._started_at
            teleop, policy = bottle_cap_commands(
                elapsed, self.config.duration_s, self.config.world_frame
            )
            self.teleop_command.publish(teleop)
            self.policy_command.publish(policy)
            await asyncio.sleep(max(0, deadline - loop.time()))

    async def main(self) -> AsyncIterator[None]:
        self._started_at = None
        task = asyncio.create_task(self._run())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self.teleop_command.publish(TwistStamped(frame_id=self.config.world_frame))
            self.policy_command.publish(TwistStamped(frame_id=self.config.world_frame))
