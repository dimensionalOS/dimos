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

"""The FlowBase for a simulation: AlfredHighLevel's ports, with no Pi behind them.

It takes speeds on ``cmd_vel``, moves through the plant it is given (the real base yaws 2.85x and
translates 0.77x what it is told, so a plan's base segment runs on the same command gains as on the
robot), and publishes the result on ``wheel_odometry`` in AlfredHighLevel's message, frames and
rate, so the relay and everything after it run unchanged.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
import math
import time

from pydantic import Field, FiniteFloat

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import twist_from_values, twist_to_values
from dimos.control.contract.limits import BASE_NAME
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevelConfig, flowbase_range

_ALFRED = AlfredHighLevelConfig()


class SimFlowBaseConfig(ConnectionConfig):
    source: str = _ALFRED.source
    base_range: Twist | None = Field(default_factory=flowbase_range)
    deadman_timeout_s: float = _ALFRED.deadman_timeout_s
    # What the base does per unit it is told: vx, vy, wz.
    plant_gains: tuple[float, float, float] = (1.0, 1.0, 1.0)
    wheel_odometry_hz: FiniteFloat = Field(default=_ALFRED.wheel_odometry_hz, gt=0.0)
    wheel_odom_frame_id: str = _ALFRED.wheel_odom_frame_id
    base_frame_id: str = _ALFRED.base_frame_id


class SimFlowBase(ConnectionModule):
    """``cmd_vel`` in, the pose it integrates to as ``wheel_odometry`` out."""

    config: SimFlowBaseConfig

    cmd_vel: In[Twist]
    wheel_odometry: Out[Odometry]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._velocity = (0.0, 0.0, 0.0)
        self._pose = (0.0, 0.0, 0.0)

    @property
    def _base(self) -> str:
        return f"{self.config.source}/{BASE_NAME}"

    def connect(self) -> None:
        """There is nothing to open."""

    def write(self, values: dict[str, float]) -> None:
        twist = twist_from_values(values, self._base)
        self._velocity = (twist.linear.x, twist.linear.y, twist.angular.z)

    def halt(self) -> None:
        self._velocity = (0.0, 0.0, 0.0)

    async def handle_cmd_vel(self, msg: Twist) -> None:
        self.command(twist_to_values(msg, self._base))

    async def main(self) -> AsyncGenerator[None, None]:
        task = asyncio.create_task(self._run())
        try:
            yield
        finally:
            task.cancel()

    async def _run(self) -> None:
        period = 1.0 / self.config.wheel_odometry_hz
        while True:
            self.step(period)
            await asyncio.sleep(period)

    def step(self, dt: float) -> None:
        """Move the base ``dt`` seconds at its current speed and publish where that left it."""
        forward, left, turn = (
            told * gain for told, gain in zip(self._velocity, self.config.plant_gains, strict=True)
        )
        x, y, yaw = self._pose
        heading = yaw + turn * dt / 2.0
        x += (math.cos(heading) * forward - math.sin(heading) * left) * dt
        y += (math.sin(heading) * forward + math.cos(heading) * left) * dt
        yaw = math.atan2(math.sin(yaw + turn * dt), math.cos(yaw + turn * dt))
        self._pose = (x, y, yaw)
        self.wheel_odometry.publish(
            Odometry(
                ts=time.time(),
                frame_id=self.config.wheel_odom_frame_id,
                child_frame_id=self.config.base_frame_id,
                pose=Pose(
                    position=Vector3(x, y, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                ),
                twist=Twist(linear=Vector3(forward, left, 0.0), angular=Vector3(0.0, 0.0, turn)),
            )
        )
