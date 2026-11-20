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

"""Blend world-frame TCP velocity sources at a fixed rate."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
from contextlib import suppress
import math
import time

import numpy as np
from pydantic import Field, FiniteFloat

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import twist_to_numpy

logger = setup_logger()

Weights6 = tuple[float, float, float, float, float, float]
TakeoverFn = Callable[[Transform], float | Weights6]


def policy_only(pose: Transform) -> float:
    """Give the policy all six axes."""
    return 1.0


class PolicyTeleopConfig(ModuleConfig):
    world_frame: str = "world"
    tcp_frame: str = "link_tcp"
    control_hz: FiniteFloat = Field(default=30.0, gt=0)
    timeout_s: FiniteFloat = Field(default=0.25, gt=0)
    takeover_fn: TakeoverFn = policy_only


class PolicyTeleopModule(Module):
    """Publish the blended twist, or zero while required inputs are unusable."""

    config: PolicyTeleopConfig
    current_pose: In[Transform]
    policy_command: In[TwistStamped]
    teleop_command: In[TwistStamped]
    ee_twist_command: Out[TwistStamped]

    _fault: str | None = None
    _pose: Transform | None = None
    _policy: TwistStamped | None = None
    _teleop: TwistStamped | None = None

    async def handle_current_pose(self, msg: Transform) -> None:
        self._pose = msg

    async def handle_policy_command(self, msg: TwistStamped) -> None:
        self._policy = msg

    async def handle_teleop_command(self, msg: TwistStamped) -> None:
        self._teleop = msg

    def _command(self, now: float) -> TwistStamped:
        pose = self._pose
        if (
            pose is None
            or pose.frame_id != self.config.world_frame
            or pose.child_frame_id != self.config.tcp_frame
            or not 0 <= now - pose.ts <= self.config.timeout_s
            or not np.isfinite(pose.translation.to_numpy()).all()
            or not math.isclose(np.linalg.norm(pose.rotation.to_numpy()), 1.0, abs_tol=1e-3)
        ):
            raise ValueError("missing, stale or invalid TCP pose")

        weights = np.asarray(self.config.takeover_fn(pose), dtype=float)
        if weights.shape not in ((), (6,)) or not np.all((weights >= 0) & (weights <= 1)):
            raise ValueError("expected one or six policy weights between zero and one")

        result = np.zeros(6)
        for name, msg, gain in (
            ("policy", self._policy, weights),
            ("teleop", self._teleop, 1 - weights),
        ):
            if not np.any(gain):
                continue
            if (
                msg is None
                or msg.frame_id != self.config.world_frame
                or not 0 <= now - msg.ts <= self.config.timeout_s
            ):
                raise ValueError(f"missing, stale or wrong-frame {name} twist")
            values = twist_to_numpy(msg)
            if not np.isfinite(values).all():
                raise ValueError(f"nonfinite {name} twist")
            result += gain * values
        return TwistStamped(ts=now, frame_id=pose.frame_id, linear=result[:3], angular=result[3:])

    def _tick(self, now: float) -> None:
        fault = None
        try:
            cmd = self._command(now)
        except Exception as exc:
            fault = f"{type(exc).__name__}: {exc}"
            cmd = TwistStamped(ts=now, frame_id=self.config.world_frame)
        self.ee_twist_command.publish(cmd)
        if fault != self._fault:
            if fault is None:
                logger.info("Policy teleop inputs ready")
            else:
                logger.warning("Policy teleop holding position", reason=fault)
            self._fault = fault

    async def _run(self) -> None:
        while True:
            self._tick(time.time())
            await asyncio.sleep(1 / self.config.control_hz)

    async def main(self) -> AsyncIterator[None]:
        task = asyncio.create_task(self._run())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self.ee_twist_command.publish(TwistStamped(frame_id=self.config.world_frame))
