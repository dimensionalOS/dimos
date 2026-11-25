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

from collections.abc import AsyncIterator
from functools import cached_property

from dimos.core.hardware_module import HardwareModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.sim2.sensor import SimSensor


class SimSceneConfig(HardwareModuleConfig):
    """A scene's mounts have no reading to ride on: they go out on tf at static_tf_rate_hz of sim time."""


class SimScene(SimSensor):
    """Static hardware: a room whose mounts are where robots start. Its tf goes out stamped with the world's time."""

    config: SimSceneConfig

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        yield

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The edges from the room's root to each mount: a real room publishes no furniture, and the world USD places it."""
        transforms = super()._static_edges
        parent_of = {t.child_frame_id: t.frame_id for t in transforms}
        needed: set[str] = set()
        for mount in self.mounts:
            frame = self.usd_frame(mount)
            while frame in parent_of and frame not in needed:
                needed.add(frame)
                frame = parent_of[frame]
        return [t for t in transforms if t.child_frame_id in needed]
