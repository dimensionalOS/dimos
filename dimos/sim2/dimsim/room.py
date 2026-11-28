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

"""DimSim's own room as a scene: what a dimsim world is built on when a blueprint names no other."""

from pathlib import Path

from pydantic import Field

from dimos.robot.assets.model import UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig, SpawnMounts, scene_usd

ROOM = Path(__file__).parent / "assets" / "room.usda"


class DimSimRoomConfig(SimSceneConfig):
    usd: UsdModel = Field(default_factory=lambda: scene_usd(ROOM))


class DimSimRoom(SimScene):
    """A 6 m room with a table, a crate to push, and one spawn point, `go2`, its root body's height above the floor."""

    config: DimSimRoomConfig
    mounts = SpawnMounts(ROOM)
