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
from collections.abc import AsyncIterator
from contextlib import suppress
import math
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field, FiniteFloat

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.blending import blend_twist, normalize_weights
from dimos.teleop.policy.takeover import TakeoverFn, policy_only
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PolicyTeleopConfig(ModuleConfig):
    """Timeouts use input Unix timestamps; future timestamps are refused.

    Attributes:
        timeout_s: Common freshness deadline for pose and both velocity inputs.
        control_hz: Output frequency, including while faulted.
        world_frame: All commands and measured translations use this frame.
        tcp_frame: The single TCP controlled by both sources.
        takeover_fn: Final policy weights, evaluated once on every valid pose tick.
    """

    world_frame: str = Field(default="world", min_length=1)
    tcp_frame: str = Field(default="link_tcp", min_length=1)
    control_hz: FiniteFloat = Field(default=30.0, gt=0)
    timeout_s: FiniteFloat = Field(default=0.25, gt=0)
    takeover_fn: TakeoverFn = policy_only


class PolicyTeleopModule(Module):
    """Publish the blended twist, or zero while required inputs are unusable.

    Input handlers and the control loop all run on the module's asyncio loop.
    """

    config: PolicyTeleopConfig
    current_pose: In[Transform]
    policy_command: In[TwistStamped]
    teleop_command: In[TwistStamped]
    ee_twist_command: Out[TwistStamped]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._pose: Transform | None = None
        self._policy: TwistStamped | None = None
        self._teleop: TwistStamped | None = None
        self._fault: str | None = None
        self._fault_lock = threading.Lock()

    @rpc
    def get_fault(self) -> str | None:
        """Return the latest holding reason, or None when inputs are usable."""
        with self._fault_lock:
            return self._fault

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
            or not np.isfinite(pose.rotation.to_numpy()).all()
            or not math.isclose(float(np.linalg.norm(pose.rotation.to_numpy())), 1.0, abs_tol=1e-3)
        ):
            raise ValueError("missing, stale or invalid TCP pose")

        # User-supplied code may raise arbitrary exceptions. _tick fails closed.
        weights = self.config.takeover_fn(pose)
        alpha = normalize_weights(weights)
        for name, msg, gain in (
            ("policy", self._policy, alpha),
            ("teleop", self._teleop, 1 - alpha),
        ):
            if not np.any(gain):
                continue
            if (
                msg is None
                or msg.frame_id != self.config.world_frame
                or not 0 <= now - msg.ts <= self.config.timeout_s
            ):
                raise ValueError(f"missing, stale or wrong-frame {name} twist")
        command = blend_twist(self._teleop, self._policy, weights)
        command.ts = now
        return command

    def _tick(self, now: float) -> None:
        fault = None
        try:
            cmd = self._command(now)
        except Exception as exc:
            # This is the boundary around user code and untrusted input samples.
            fault = f"{type(exc).__name__}: {exc}"
            cmd = TwistStamped(ts=now, frame_id=self.config.world_frame)
        with self._fault_lock:
            previous, self._fault = self._fault, fault
        self.ee_twist_command.publish(cmd)
        if fault != previous:
            if fault is None:
                logger.info("Policy teleop inputs ready")
            else:
                logger.warning("Policy teleop holding position", reason=fault)

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        period = 1 / self.config.control_hz
        while True:
            deadline = loop.time() + period
            self._tick(time.time())
            await asyncio.sleep(max(0, deadline - loop.time()))

    async def main(self) -> AsyncIterator[None]:
        task = asyncio.create_task(self._run())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self.ee_twist_command.publish(TwistStamped(frame_id=self.config.world_frame))
            self._pose = None
            self._policy = None
            self._teleop = None
