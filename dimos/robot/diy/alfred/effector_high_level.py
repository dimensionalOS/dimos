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
import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.diy.alfred.config import DEFAULT_ADDRESS
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    # An optional hardware dependency (the `misc` extra); imported lazily at
    # connect time so merely importing this module doesn't require it.
    import portal

logger = setup_logger()


class AlfredHighLevelConfig(ModuleConfig):
    address: str = DEFAULT_ADDRESS
    cmd_vel_timeout: float = 0.2
    wheel_odometry_hz: float = 50.0
    # Not "odom": Point-LIO publishes odom->base_link, and a second publisher on
    # that edge gives base_link two parents, which is a malformed tf tree rather
    # than a redundant one. Wheel odometry gets its own root and stays a plain
    # message stream for a consumer to fuse.
    wheel_odom_frame_id: str = "wheel_odom"
    base_frame_id: str = "base_link"


class AlfredHighLevel(Module):
    cmd_vel: In[Twist]
    wheel_odometry: Out[Odometry]
    config: AlfredHighLevelConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._client: portal.Client | None = None
        self._stop_task: asyncio.Task[None] | None = None
        self._odometry_task: asyncio.Task[None] | None = None
        self._last_velocities = [0.0, 0.0, 0.0]
        self._client_lock = asyncio.Lock()

    async def main(self) -> AsyncGenerator[None, None]:
        # The controller multiplexes one connection; the lock keeps a velocity
        # command from interleaving with an in-flight odometry poll. Recreated
        # each run so a restart binds it to the new event loop.
        self._client_lock = asyncio.Lock()
        import portal

        client = portal.Client(self.config.address)
        self._client = client
        logger.info(f"Connected to Alfred at {self.config.address}")
        self._odometry_task = asyncio.create_task(self._poll_wheel_odometry())
        try:
            yield
        finally:
            if self._odometry_task is not None and not self._odometry_task.done():
                self._odometry_task.cancel()
            if self._stop_task is not None and not self._stop_task.done():
                self._stop_task.cancel()
            try:
                await self._send_velocity(0.0, 0.0, 0.0)
            except Exception as e:
                logger.error(f"Error stopping Alfred: {e}")
            try:
                client.close()
            except Exception:
                pass
            # A restart can overlap: this run's teardown must not null out a
            # newer run's client.
            if self._client is client:
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
            if await self._send_velocity(0.0, 0.0, 0.0):
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
            async with self._client_lock:
                future = self._client.set_target_velocity(command)
                await asyncio.to_thread(future.result)
            return True
        except Exception as e:
            logger.error(f"Error sending Alfred velocity: {e}")
            return False

    async def _poll_wheel_odometry(self) -> None:
        """Publish the controller's on-board integrated pose as `wheel_odometry`."""
        period = 1.0 / self.config.wheel_odometry_hz
        previous: tuple[float, float, float, float] | None = None
        while True:
            start = asyncio.get_running_loop().time()
            try:
                async with self._client_lock:
                    future = self._client.get_odometry({})  # type: ignore[union-attr]
                    reading = await asyncio.to_thread(future.result)
                ts = time.time()
                x, y = (float(v) for v in reading["translation"])
                yaw = float(reading["rotation"])
            except asyncio.CancelledError:
                return
            except Exception as error:
                logger.error(f"Alfred wheel odometry read failed: {error}")
                await asyncio.sleep(period)
                continue

            # The controller integrates in the same inverted-Y frame move()
            # sends into; negate back so consumers see the ROS convention.
            y, yaw = -y, -yaw
            # The controller reports no velocity, so difference consecutive
            # poses and rotate the world-frame displacement into the base frame.
            twist = Twist()
            if previous is not None:
                last_ts, last_x, last_y, last_yaw = previous
                dt = ts - last_ts
                if dt > 0.0:
                    forward = math.cos(yaw) * (x - last_x) + math.sin(yaw) * (y - last_y)
                    left = -math.sin(yaw) * (x - last_x) + math.cos(yaw) * (y - last_y)
                    turn = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
                    twist = Twist(
                        linear=Vector3(forward / dt, left / dt, 0.0),
                        angular=Vector3(0.0, 0.0, turn / dt),
                    )
            previous = (ts, x, y, yaw)

            self.wheel_odometry.publish(
                Odometry(
                    ts=ts,
                    frame_id=self.config.wheel_odom_frame_id,
                    child_frame_id=self.config.base_frame_id,
                    pose=Pose(
                        position=Vector3(x, y, 0.0),
                        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                    ),
                    twist=twist,
                )
            )
            await asyncio.sleep(max(0.0, period - (asyncio.get_running_loop().time() - start)))
