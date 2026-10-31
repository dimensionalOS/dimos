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


"""What a physics engine implements to run behind Simulation."""

from collections.abc import Callable, Mapping, Sequence
from importlib import import_module
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from dimos_lcm.world_msgs import WorldChange

from dimos.sim2.actions import Action
from dimos.sim2.spec import Simulated
from dimos.sim2.world_change import WorldWriter

if TYPE_CHECKING:
    from dimos.sim2.module import SimulationConfig

BACKENDS = {"mujoco": "dimos.sim2.mujoco_backend.MujocoBackend"}


class Publisher(Protocol):
    @property
    def busy(self) -> bool: ...
    def offer(self, change: WorldChange) -> None: ...


class SimulationBackend(Protocol):
    # What its frames are written with: the ids actions address its entities by.
    world: WorldWriter

    def __init__(
        self, config: "SimulationConfig", simulated: Mapping[str, Simulated], directory: Path
    ) -> None:
        """Build the world from config.scene and simulated, writing any files into directory."""

    async def run(self, world_change: Publisher) -> None:
        """Step the world forever, offering a frame of what changed whenever world_change is not busy.

        It writes a frame only when it can offer it, so no change is ever written and then dropped.
        """

    def apply(self, actions: Sequence[Action]) -> None:
        """Called from any thread; takes effect before the next step."""

    def assets(self) -> Mapping[str, Path | bytes]:
        """Every file a renderer may request: the world description and each file it names."""

    def scene_data(self) -> str:
        """The world outside the robots as dimos.scene_data.v1 JSON, for a viewer to draw."""

    def describe(self) -> dict[str, Any]:
        """Backend specifics for local clients, like the paths of a shared model and state."""


def load_backend(
    name: str,
) -> Callable[["SimulationConfig", Mapping[str, Simulated], Path], SimulationBackend]:
    module, _, cls = BACKENDS[name].rpartition(".")
    return getattr(import_module(module), cls)  # type: ignore[no-any-return]
