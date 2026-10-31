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

"""A robot whose base is driven straight from cmd_vel (SetVelocity), with odom/tf like a real connection's."""

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldChange

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.actions import SetVelocity, world_action
from dimos.sim2.module import Simulation
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_change import WorldMirror


class SimKinematicBaseConfig(ModuleConfig):
    robot_id: str
    model: Path
    root_body: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Seconds a cmd_vel holds before the base stops, as a real base's watchdog does.
    cmd_vel_timeout: float = 0.5
    # Sim-time rate of odom and tf.
    odom_rate_hz: float = 50.0


class SimKinematicBase(Module):
    config: SimKinematicBaseConfig
    cmd_vel: In[Twist]
    world_change: In[WorldChange]
    actions: Out[WorldAction]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._expires: float | None = None

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config = self.config
        return {
            "simulated": SimulatedRobot(
                config.robot_id, config.model, config.root_body, xyz=config.xyz
            )
        }

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_change)
        self._reporting = asyncio.create_task(self._report())
        yield

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self.actions.publish(world_action(self._world, SetVelocity(self.config.robot_id, twist)))
        self._expires = time.monotonic() + self.config.cmd_vel_timeout

    async def _report(self) -> None:
        async for _ in self._world.ticks(self.config.odom_rate_hz):
            if self._expires is not None and time.monotonic() > self._expires:
                self._expires = None
                self.actions.publish(
                    world_action(self._world, SetVelocity(self.config.robot_id, Twist()))
                )
            with self._world.lock:
                entity = self._world.entity(self.config.robot_id)
                stamp = self._world.stamp()
            if entity is None or entity.pose is None:
                continue
            odom = PoseStamped(
                ts=stamp,
                frame_id="world",
                position=entity.pose[:3].tolist(),
                orientation=entity.pose[3:].tolist(),
            )
            self.tf.publish(TFMessage(Transform.from_pose("base_link", odom)))
            self.odom.publish(odom)
