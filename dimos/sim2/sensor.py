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
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mount
from dimos.core.stream import In
from dimos.robot.assets.usd_utils.lfs import pull_model_assets
from dimos.sim2.sim import SimulatedWorldSpec, on_mount
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = setup_logger()


class SimSensorConfig(HardwareModuleConfig):
    # Seconds a reading's stamp (the world time it read) may be off wall time and still go out; None is one sample period.
    stamp_tolerance: float | None = None


class SimSensor(HardwareModule):
    """Stamps each reading with the world time it read, which is wall time when the world keeps up; a reading too far off it is not sent."""

    world: SimulatedWorldSpec
    world_observations: In[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # The streams whose last reading was too far off wall time, warned about once until one is not.
        self._late: set[str] = set()

    @classmethod
    def attach(cls, mount: Mount, **kwargs: Any) -> "Blueprint":
        """This sensor on ``mount``, as HardwareModule.attach puts it, still reaching the one simulated world."""
        return on_mount(cls.blueprint(**kwargs, attached_to=mount.name), mount)

    @rpc
    def start(self) -> None:
        # Before anything opens its stage, which opens every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    @property
    def frame_id(self) -> str:
        """The sensor's frame: its USD's root link, under its namespace."""
        return self.tf_frames[self.config.usd.root_link]

    def publish_moving_tf(self, ts: float) -> None:
        """Publish its USD's moving edges (at zero: a sensor drives no joint) stamped ``ts``, so a sensor USD with a joint keeps it on tf."""
        transforms = self.moving_transforms(ts=ts)
        if transforms:
            self.tfbuffer.publish(*transforms)

    def on_time(self, stream: str, stamp: float, period: float, tolerance: float | None) -> bool:
        """Whether a ``stream`` reading stamped ``stamp`` is within ``tolerance`` (None: ``period``) of wall time: one that is not, the real sensor would have missed."""
        off = time.time() - stamp
        if abs(off) <= (period if tolerance is None else tolerance):
            self._late.discard(stream)
            return True
        if stream not in self._late:
            self._late.add(stream)
            logger.warning(
                f"{self.frame_id} {stream}: the world is {off:.3f} s off wall time, so its readings are dropped until it is back"
            )
        return False

    def mirror_world(self) -> WorldMirror:
        world = WorldMirror()
        world.subscribe(self.world_observations, self.world.snapshot)
        return world

    def world_layer(self) -> Path:
        """The composed world, as a layer file a view can open."""
        return layer_file(self.world.get_world_usd(), "world")
