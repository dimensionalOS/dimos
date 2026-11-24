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

import asyncio
from collections.abc import AsyncIterator

from dimos.core.hardware_module import HardwareModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.robot.assets.model import UsdModel
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sensor_usd import frame_usd
from dimos.sim2.sim import until_it_stops


class SimImuConfig(HardwareModuleConfig):
    usd: UsdModel | None = frame_usd("imu_link", name="sim_imu")
    rate_hz: float = 200.0


class SimImu(SimSensor):
    config: SimImuConfig
    imu: Out[Imu]

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        self._reading = asyncio.create_task(until_it_stops(f"{self.frame_id} imu", self._read()))
        yield
        self._reading.cancel()

    async def _read(self) -> None:
        site = self.frame_id
        view = None
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                layer = await asyncio.to_thread(self.world_layer)
                view = open_view(layer, robot_ids(self._world), SensorMount("imu", site, site))
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            stamp = self._world.stamp()
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=site,
                    ts=stamp,
                )
            )
            self.publish_usd_tf(ts=stamp)
