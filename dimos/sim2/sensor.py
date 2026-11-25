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

"""A simulated sensor: hardware with its own USD, reading the world the physics backend publishes."""

from pathlib import Path

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule
from dimos.core.stream import In
from dimos.sim2.lfs import pull_model_assets
from dimos.sim2.sim import SimulatedWorldSpec
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror


class SimSensor(HardwareModule):
    mount_shared = frozenset({"world_observations"})
    world: SimulatedWorldSpec
    world_observations: In[WorldObservations]

    @rpc
    def start(self) -> None:
        # Before anything opens its stage, which opens every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    @property
    def frame_id(self) -> str:
        """The sensor's frame: its USD's root link, under its namespace."""
        return self.usd_frame(self.config.usd.root_link)

    def mirror_world(self) -> WorldMirror:
        world = WorldMirror()
        world.subscribe(self.world_observations, self.world.snapshot)
        return world

    def world_layer(self) -> Path:
        """The composed world, as a layer file a view can open."""
        return layer_file(self.world.get_world_usd(), "world")
