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
from functools import cached_property

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.sim2.lfs import pull_model_assets
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sim import until_it_stops


class SimSceneConfig(HardwareModuleConfig):
    # Static frames have no reading to ride on, so they go out at this rate of sim time.
    tf_rate_hz: float = 10.0


class SimScene(SimSensor):
    """Static hardware: a room whose mounts are where robots start. Its tf goes out stamped with the world's time."""

    config: SimSceneConfig

    @rpc
    def start(self) -> None:
        # Before anything opens the room's stage, which opens every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        self._publishing = asyncio.create_task(until_it_stops("scene tf", self._publish()))
        yield
        self._publishing.cancel()

    async def _publish(self) -> None:
        async for stamp in self._world.ticks(self.config.tf_rate_hz):
            self.tfbuffer.publish(*self.mount_transforms(ts=stamp))

    def mount_transforms(self, ts: float | None = None) -> list[Transform]:
        """The edges from the room's root to each mount: a real room publishes no furniture, and the world USD places it."""
        return [
            Transform(
                translation=edge.translation,
                rotation=edge.rotation,
                frame_id=edge.frame_id,
                child_frame_id=edge.child_frame_id,
                ts=ts,
            )
            for edge in self._mount_edges
        ]

    @cached_property
    def _mount_edges(self) -> list[Transform]:
        # A room does not move: its edges are worked out once and only restamped, as working them out holds the GIL for every joint it has.
        transforms = self.usd_transforms()
        parent_of = {t.child_frame_id: t.frame_id for t in transforms}
        needed: set[str] = set()
        for mount in self.mounts:
            frame = self.usd_frame(mount)
            while frame in parent_of and frame not in needed:
                needed.add(frame)
                frame = parent_of[frame]
        return [t for t in transforms if t.child_frame_id in needed]
