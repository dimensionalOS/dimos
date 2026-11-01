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
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import twist_from_values, twist_to_values
from dimos.control.contract.limits import BASE_NAME
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.drive_trains.flowbase import portal_compat
from dimos.hardware.drive_trains.flowbase.config import FLOWBASE_MAX_LINEAR_CMD_M_S
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.diy.alfred.config import DEFAULT_ADDRESS
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    import portal

logger = setup_logger()

# these combined need to be less than DEFAULT_THREAD_JOIN_TIMEOUT so the teardown join outlasts them.
STOP_COMMAND_TIMEOUT_SECONDS = DEFAULT_THREAD_JOIN_TIMEOUT / 2
FLOWBASE_MAX_TURN_CMD_RAD_S = 3.0
MOVE_VELOCITY_PERIOD_S = 0.1  # a teleop stream's rate: inside the velocity task's timeout
CONNECTION_CLOSE_TIMEOUT_SECONDS = DEFAULT_THREAD_JOIN_TIMEOUT / 4


def flowbase_range() -> Twist:
    """The fastest the FlowBase may be told to go: the firmware's cap on translation. It states
    none on turning, so this one is only wide enough to leave teleop as it was."""
    return Twist(
        linear=Vector3(FLOWBASE_MAX_LINEAR_CMD_M_S, FLOWBASE_MAX_LINEAR_CMD_M_S, 0.0),
        angular=Vector3(0.0, 0.0, FLOWBASE_MAX_TURN_CMD_RAD_S),
    )


class AlfredHighLevelConfig(ConnectionConfig):
    source: str = "flowbase"
    base_range: Twist | None = Field(default_factory=flowbase_range)
    # A stack with a coordinator commands every tick and can run the default 0.1 s; a navigation
    # stack's MovementManager streams far slower than that.
    deadman_timeout_s: float = 0.2
    address: str = DEFAULT_ADDRESS
    wheel_odometry_hz: FiniteFloat = Field(default=50.0, gt=0.0)
    wheel_odom_frame_id: str = "wheel_odom"
    base_frame_id: str = "base_link"
    # The controller's odometry counts from when its own driver started, not from when this
    # stack did (36 m and two turns into a day): rebased, the run's first reading is the
    # origin, like Point-LIO's. Off, the poses come through as the controller reports them.
    rebase_odometry: bool = True


class AlfredHighLevel(ConnectionModule):
    """The FlowBase over Portal. Its only writer: speeds arrive on ``cmd_vel``, and the deadman
    zeroes them when the stream stops."""

    cmd_vel: In[Twist]
    # Where move_velocity drives from: into whatever arbitrates the base, so a hold stops it too.
    tele_cmd_vel: Out[Twist]
    wheel_odometry: Out[Odometry]
    config: AlfredHighLevelConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._client: portal.Client | None = None
        self._odometry_task: asyncio.Task[None] | None = None
        self._send_task: asyncio.Task[None] | None = None
        self._drive_task: asyncio.Task[None] | None = None
        self._velocity = (0.0, 0.0, 0.0)
        self._velocity_ready = asyncio.Event()
        self._last_velocities = [0.0, 0.0, 0.0]
        self._odometry_stop = asyncio.Event()

    @property
    def _base(self) -> str:
        return f"{self.config.source}/{BASE_NAME}"

    async def main(self) -> AsyncGenerator[None, None]:
        # Only the running module needs the SDK; importing it at module scope would make
        # every blueprint that composes Alfred unimportable without the misc extra.
        import portal

        # Portal's socket thread otherwise spins a whole core once the 50 Hz odometry poll
        # races its wake-up (see portal_compat).
        portal_compat.install()
        # Recreated each run so a restart binds it to the new event loop.
        self._odometry_stop = asyncio.Event()
        self._velocity_ready = asyncio.Event()
        client = portal.Client(self.config.address)
        self._client = client
        logger.info(f"Connected to Alfred at {self.config.address}")
        self._odometry_task = asyncio.create_task(self._poll_wheel_odometry(client))
        self._send_task = asyncio.create_task(self._send_velocities())
        try:
            yield
        finally:
            self._send_task.cancel()
            if self._drive_task is not None:
                self._drive_task.cancel()
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

    def connect(self) -> None:
        """The Portal client opens in ``main``, on the module's event loop."""

    def write(self, values: dict[str, float]) -> None:
        twist = twist_from_values(values, self._base)
        self._request_velocity((twist.linear.x, twist.linear.y, twist.angular.z))

    def halt(self) -> None:
        self._request_velocity((0.0, 0.0, 0.0))

    def _request_velocity(self, velocity: tuple[float, float, float]) -> None:
        self._velocity = velocity
        self._on_loop(self._velocity_ready.set)

    async def handle_cmd_vel(self, msg: Twist) -> None:
        self.command(twist_to_values(msg, self._base))

    async def _send_velocities(self) -> None:
        """Send the newest velocity, never a backlog of them, and a stop only when it is news:
        a coordinator tells a base at rest 0 every tick, and Portal is a round trip to the Pi."""
        while True:
            await self._velocity_ready.wait()
            self._velocity_ready.clear()
            velocity = self._velocity
            if not any(velocity) and not any(self._last_velocities):
                continue
            vx, vy, wz = velocity
            if await self._send_velocity(vx, -vy, -wz):
                self._last_velocities = [vx, vy, wz]

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
        if self._drive_task is not None:
            self._drive_task.cancel()
        self._drive_task = asyncio.create_task(self._drive_for(twist, duration))
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    async def _drive_for(self, twist: Twist, duration: float) -> None:
        """Keep a teleop stream going for ``duration`` seconds, then stop it."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + duration
        self.tele_cmd_vel.publish(twist)
        while loop.time() < deadline:
            await asyncio.sleep(MOVE_VELOCITY_PERIOD_S)
            self.tele_cmd_vel.publish(twist)
        self.tele_cmd_vel.publish(Twist())

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
        origin: tuple[float, float, float] | None = None
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
                if self.config.rebase_odometry:
                    if origin is None:
                        origin = (x, y, yaw)
                        logger.info(
                            f"Wheel odometry origin is the controller's ({x:.3f}, {y:.3f}, "
                            f"{yaw:.3f} rad); poses are relative to it from here"
                        )
                    x, y, yaw = _rebase(origin, x, y, yaw)
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


def _rebase(
    origin: tuple[float, float, float], x: float, y: float, yaw: float
) -> tuple[float, float, float]:
    """The pose in the frame whose origin is ``origin``: the travel from it turned into its
    heading, the yaw relative to it (an SE(2) change of origin, not a plain subtraction)."""
    origin_x, origin_y, origin_yaw = origin
    dx, dy = x - origin_x, y - origin_y
    cos_yaw, sin_yaw = math.cos(origin_yaw), math.sin(origin_yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        angle_diff(yaw, origin_yaw),
    )


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
