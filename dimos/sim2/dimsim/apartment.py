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

"""DimSim's apartment as a scene: its fridge, cabinets, lamps, TV and the rest keep their actions."""

from pydantic import Field

from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.sim2.dimsim.dimsim_scene import dimsim_scene_usd
from dimos.sim2.scene import SimScene, SimSceneConfig

# DimSim's spawn point for this scene (scenes/apartment/index.js), z up, on the rug there.
SPOTS = {"go2": ((3.0, 2.0, 0.12), (0.0, 0.0, 0.0))}
# The DimSim scene, which names its top prim too, so each spot is a frame under it.
_SCENE = "apartment"


def apartment_usd() -> UsdModel:
    return UsdModel.from_layer(dimsim_scene_usd(_SCENE)).with_frames(
        {name: (_SCENE, xyz, rpy) for name, (xyz, rpy) in SPOTS.items()}
    )


class DimSimApartmentConfig(SimSceneConfig):
    # Converted when the scene is made, not imported: its objects' meshes are read out of the scene's GLBs.
    usd: UsdModel = Field(default_factory=apartment_usd)


class DimSimApartment(SimScene):
    config: DimSimApartmentConfig
    mounts = Mounts(*(f"{_SCENE}/{name}" for name in SPOTS))
