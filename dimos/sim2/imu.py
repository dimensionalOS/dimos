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

"""An IMU at a mount, read from its own view of the world as world_change moves it."""

import asyncio
from collections.abc import AsyncIterator
from typing import Any

from dimos_lcm.world_msgs import WorldChange

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.spec import Mount, SimulatedSensor
from dimos.sim2.view import WorldView, open_view
from dimos.sim2.world_change import WorldMirror


class SimImuConfig(ModuleConfig):
    name: str = "imu"
    mount: Mount = Mount("world")
    robot_id: str | None = None
    frame_id: str | None = "imu_link"
    rate_hz: float = 200.0


class SimImu(Module):
    config: SimImuConfig
    imu: Out[Imu]
    world_change: In[WorldChange]
    _simulation: SimulationSpec

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config = self.config
        return {"simulated": SimulatedSensor(config.name, "imu", config.mount, config.robot_id)}

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_change)
        self._reading = asyncio.create_task(self._read(open_view(self._simulation.describe())))
        yield

    async def _read(self, view: WorldView) -> None:
        sensor = f"{self.config.robot_id or 'world'}/{self.config.name}"
        # rate_hz of sim time: its readings difference velocities over sim time, so they hold at any speed.
        async for _ in self._world.ticks(self.config.rate_hz):
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(sensor)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.frame_id,
                    ts=self._world.stamp(),
                )
            )
