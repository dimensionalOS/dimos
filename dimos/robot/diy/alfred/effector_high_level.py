# Copyright 2025-2026 Dimensional Inc.
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

"""Alfred high-level control via Portal RPC.

Subscribes to ``cmd_vel`` and forwards each Twist to the Alfred controller
as a holonomic target velocity. The controller performs the wheel-level
kinematics on-board, so this module hands off ``(vx, vy, wz)`` rather than
computing per-wheel speeds locally.

Frame convention: Alfred uses an inverted Y-axis vs. ROS, so ``vy`` and
``wz`` are negated before being sent to the hardware.

  Standard (ROS):     Alfred:
      +Y                -Y
      ↑                  ↑
   ───┼──→ +X         ───┼──→ +X
      |                  |
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
from typing import Any

import numpy as np
import portal

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred.config import DEFAULT_ADDRESS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AlfredHighLevelConfig(ModuleConfig):
    address: str = DEFAULT_ADDRESS
    cmd_vel_timeout: float = 0.2


class AlfredHighLevel(Module):
    cmd_vel: In[Twist]
    config: AlfredHighLevelConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._client: portal.Client | None = None
        self._stop_task: asyncio.Task[None] | None = None
        self._last_velocities = [0.0, 0.0, 0.0]

    async def main(self) -> AsyncGenerator[None, None]:
        self._client = portal.Client(self.config.address)
        logger.info(f"Connected to Alfred at {self.config.address}")
        try:
            yield
        finally:
            if self._stop_task is not None and not self._stop_task.done():
                self._stop_task.cancel()
            try:
                await self._send_velocity(0.0, 0.0, 0.0)
            except Exception as e:
                logger.error(f"Error stopping Alfred: {e}")
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None
            logger.info("Alfred high-level connection stopped")

    async def handle_cmd_vel(self, msg: Twist) -> None:
        await self.move(msg)

    @rpc
    async def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a Twist as a holonomic velocity command.

        With ``duration > 0`` the command runs for that many seconds before
        auto-stop. With ``duration == 0`` each call rearms a ``cmd_vel_timeout``
        watchdog; if the stream stalls, the platform stops automatically.
        """
        if self._client is None:
            logger.warning("Alfred not connected; ignoring move")
            return False

        vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z

        if self._stop_task is not None and not self._stop_task.done():
            self._stop_task.cancel()

        # Negate vy and wz for Alfred's inverted Y-axis frame.
        # Send before scheduling the watchdog — otherwise it could fire first.
        if not await self._send_velocity(vx, -vy, -wz):
            return False

        self._last_velocities = [vx, vy, wz]
        timeout = duration if duration > 0 else self.config.cmd_vel_timeout
        self._stop_task = asyncio.create_task(self._auto_stop_movement(timeout))
        return True

    async def _auto_stop_movement(self, delay: float) -> None:
        try:
            await asyncio.sleep(delay)
        except asyncio.CancelledError:
            return
        try:
            await self._send_velocity(0.0, 0.0, 0.0)
            self._last_velocities = [0.0, 0.0, 0.0]
        except Exception as e:
            logger.error(f"Auto-stop failed: {e}")

    @rpc
    async def get_state(self) -> str:
        if self._client is None:
            return "DISCONNECTED"
        moving = any(abs(v) > 1e-6 for v in self._last_velocities)
        return "MOVING" if moving else "STOPPED"

    @skill
    async def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move the Alfred at the given velocity for ``duration`` seconds."""
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        await self.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    async def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send a raw velocity (already in Alfred frame) via Portal RPC."""
        if self._client is None:
            return False
        try:
            command = {
                "target_velocity": np.array([vx, vy, wz]),
                "frame": "local",
            }
            future = self._client.set_target_velocity(command)
            await asyncio.to_thread(future.result)
            return True
        except Exception as e:
            logger.error(f"Error sending Alfred velocity: {e}")
            return False


__all__ = ["AlfredHighLevel", "AlfredHighLevelConfig"]
