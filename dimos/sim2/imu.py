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
from collections.abc import Coroutine
from typing import Any

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.robot.assets.model import UsdModel
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.sensor import SimSensorConfig
from dimos.sim2.sensor_usd import frame_usd
from dimos.sim2.view_sensor import ViewSensor, imu_message


class SimImuConfig(SimSensorConfig):
    usd: UsdModel = frame_usd("imu_link", name="sim_imu")
    rate_hz: float = 200.0


class SimImu(ViewSensor):
    config: SimImuConfig
    imu: Out[Imu]

    def loops(self) -> list[Coroutine[Any, Any, None]]:
        return [self._read()]

    async def _read(self) -> None:
        site = self.frame_id
        view = None
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                view = await asyncio.to_thread(self.open_view, SensorMount("imu", site, site))
                # Opening took this tick, so the next tick is the first read on time.
                continue
            view.update(self._world)
            stamp = view.time
            if not self.on_time("imu", stamp, 1 / self.config.rate_hz, self.config.stamp_tolerance):
                continue
            self.imu.publish(imu_message(view, site))
            self.publish_moving_tf(stamp)
