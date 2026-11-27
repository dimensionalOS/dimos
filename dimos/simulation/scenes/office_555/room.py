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


"""Office 555 as a scene: two floors of offices, a workshop, a kitchen and meeting rooms, with spawn points in every room."""

from pathlib import Path

from pydantic import Field

from dimos.robot.assets.model import UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig, SpawnMounts, scene_usd

# The layer and every mesh it references are Git LFS, so a checkout made with GIT_LFS_SKIP_SMUDGE=1
# has pointer files here; scene_usd fetches them before anything opens a stage of it.
SCENE = Path(__file__).parent / "scene.usda"


class Office555Config(SimSceneConfig):
    usd: UsdModel = Field(default_factory=lambda: scene_usd(SCENE))


class Office555(SimScene):
    """Office 555. Robots spawn on its floor spots (``ground_*``, ``upper_*``, ``main``, ``left``, ``right``), objects on its ``*_on_*`` slots, and a room camera on ``overview``."""

    config: Office555Config
    mounts = SpawnMounts(SCENE)
