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

The controller does the wheel-level kinematics on-board, so this hands off a
holonomic ``(vx, vy, wz)`` rather than computing per-wheel speeds locally.

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
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from pydantic import Field, FiniteFloat

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
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
    import portal

logger = setup_logger()

# these combined need to be less than DEFAULT_THREAD_JOIN_TIMEOUT so the teardown join outlasts them.
STOP_COMMAND_TIMEOUT_SECONDS = DEFAULT_THREAD_JOIN_TIMEOUT / 2
CONNECTION_CLOSE_TIMEOUT_SECONDS = DEFAULT_THREAD_JOIN_TIMEOUT / 4


class AlfredHighLevelConfig(ModuleConfig):
    address: str = DEFAULT_ADDRESS
    cmd_vel_timeout: float = 0.2
    wheel_odometry_hz: FiniteFloat = Field(50.0, gt=0.0)
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
        self._odometry_stop = asyncio.Event()

    async def main(self) -> AsyncGenerator[None, None]:
        # Recreated each run so a restart binds it to the new event loop.
        self._odometry_stop = asyncio.Event()
        # portal is optional; importing this module for its blueprint must work without it.
        import portal

        client = portal.Client(self.config.address)
        self._client = client
        logger.info(f"Connected to Alfred at {self.config.address}")
        self._odometry_task = asyncio.create_task(self._poll_wheel_odometry(client))
        try:
            yield
        finally:
            if self._stop_task is not None and not self._stop_task.done():
                self._stop_task.cancel()
            # Stop before draining: portal matches replies by request number, so a
            # wedged poll cannot delay the stop.
            stopper = threading.Thread(target=_stop_and_close, args=(client,), daemon=True)
            stopper.start()
            self._odometry_stop.set()
            if self._odometry_task is not None:
                done, _ = await asyncio.wait(
                    {self._odometry_task}, timeout=DEFAULT_THREAD_JOIN_TIMEOUT
                )
                if not done:
                    logger.warning("Alfred wheel odometry poll is still running; cancelling it")
                    self._odometry_task.cancel()
            await asyncio.to_thread(stopper.join, DEFAULT_THREAD_JOIN_TIMEOUT)
            if stopper.is_alive():
                logger.error("Alfred has not taken the stop command yet; it may still be moving")
            # A restart can overlap: this teardown must not null out a newer run's client.
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
            await _rpc_result(self._client.set_target_velocity(command))
            return True
        except Exception as e:
            logger.error(f"Error sending Alfred velocity: {e}")
            return False

    async def _poll_wheel_odometry(self, client: portal.Client) -> None:
        period = 1.0 / self.config.wheel_odometry_hz
        previous: tuple[float, float, float, float] | None = None
        last_error_log: float | None = None
        while not self._odometry_stop.is_set():
            start = asyncio.get_running_loop().time()
            try:
                # Stamped before the call so a slow reply cannot drag the stamp forward.
                ts = time.time()
                reading = await _rpc_result(client.get_odometry({}))
                x, y = (float(v) for v in reading["translation"])
                yaw = float(reading["rotation"])

                # Odometry comes back in the inverted-Y frame move() sends into.
                y, yaw = -y, -yaw
                # The controller reports no velocity, so difference consecutive poses.
                twist = Twist()
                if previous is not None:
                    last_ts, last_x, last_y, last_yaw = previous
                    dt = ts - last_ts
                    if dt > 0.0:
                        turn = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
                        # Either endpoint yaw biases the forward/left split while turning.
                        heading = last_yaw + turn / 2.0
                        dx, dy = x - last_x, y - last_y
                        forward = math.cos(heading) * dx + math.sin(heading) * dy
                        left = -math.sin(heading) * dx + math.cos(heading) * dy
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
            except asyncio.CancelledError:
                return
            except Exception as error:
                now = asyncio.get_running_loop().time()
                odometry_error_log_interval_seconds = 10.0
                if (
                    last_error_log is None
                    or now - last_error_log >= odometry_error_log_interval_seconds
                ):
                    last_error_log = now
                    logger.error(f"Alfred wheel odometry poll failed: {error}")
                await self._wait_or_stop(period)
                continue

            await self._wait_or_stop(max(0.0, period - (asyncio.get_running_loop().time() - start)))

    async def _wait_or_stop(self, seconds: float) -> None:
        try:
            await asyncio.wait_for(self._odometry_stop.wait(), seconds)
        except asyncio.TimeoutError:
            pass


PORTAL_RPC_POLL_SECONDS = 0.002


async def _rpc_result(future: portal.Future) -> Any:
    """portal.Future's are kinda dumb. We have to wait on them to unwrap."""
    while not future.done():
        await asyncio.sleep(PORTAL_RPC_POLL_SECONDS)
    return future.result()


def _stop_and_close(client: portal.Client) -> None:
    """Portal answers on a worker thread, so a close chained to the stop through the
    event loop is skipped outright once the loop shuts down, leaving the client open.
    """
    try:
        client.set_target_velocity({"target_velocity": np.zeros(3), "frame": "local"}).result(
            timeout=STOP_COMMAND_TIMEOUT_SECONDS
        )
    except Exception as e:
        logger.error(f"Error stopping Alfred: {e!r}")
    # Closing fails every request the connection still owes, so it goes after the stop.
    try:
        # Left unbounded, portal waits out a send queue that a dead peer never drains.
        client.close(timeout=CONNECTION_CLOSE_TIMEOUT_SECONDS)
    except Exception as e:
        logger.error(f"Error closing the Alfred connection: {e!r}")
