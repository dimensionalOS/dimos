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

"""The scene a simulated world is built on: the one unmounted HardwareModule, with a frame per robot spot."""

import asyncio
from collections.abc import AsyncIterator

from dimos.core.hardware_module import HardwareModuleConfig
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sim import until_it_stops


class SimSceneConfig(HardwareModuleConfig):
    # Static frames have no reading to ride on, so they go out at this rate of sim time.
    tf_rate_hz: float = 10.0


class SimScene(SimSensor):
    """Static hardware: a room whose mounts are where robots start. Its tf goes out stamped with the world's time."""

    config: SimSceneConfig

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        self._publishing = asyncio.create_task(until_it_stops("scene tf", self._publish()))
        yield
        self._publishing.cancel()

    async def _publish(self) -> None:
        async for stamp in self._world.ticks(self.config.tf_rate_hz):
            self.publish_usd_tf(ts=stamp)
