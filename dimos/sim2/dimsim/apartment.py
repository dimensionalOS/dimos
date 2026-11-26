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

"""DimSim's apartment as a scene: its structure, furniture and lights."""

from typing import Any

from pydantic import model_validator

from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.sim2.dimsim.dimsim_scene import dimsim_scene_usd
from dimos.sim2.scene import SimScene, SimSceneConfig

# DimSim's spawn point for this scene (scenes/apartment/index.js), z up, on the rug there.
SPOTS = {"go2": ((3.0, 2.0, 0.12), (0.0, 0.0, 0.0))}
# The DimSim scene, which names its top prim too, so each spot is a frame under it.
_SCENE = "apartment"


class DimSimApartmentConfig(SimSceneConfig):
    # The DomeLight radiance standing in for DimSim's environment map (a RoomEnvironment), by eye against DimSim's view.
    environment: float = 1.15
    # What a viewer reading UsdLux as Blender does (Controller) multiplies a normalized DistantLight's intensity by.
    sun_per_intensity: float = 4.0

    @model_validator(mode="before")
    @classmethod
    def _converted(cls, data: Any) -> Any:
        """The scene converted with this config's lighting, unless a usd is given; its meshes are read out of its GLBs."""
        if not isinstance(data, dict) or "usd" in data:
            return data
        lighting = {
            name: data.get(name, cls.model_fields[name].default)
            for name in ("environment", "sun_per_intensity")
        }
        usd = UsdModel.from_layer(dimsim_scene_usd(_SCENE, **lighting)).with_frames(
            {name: (_SCENE, xyz, rpy) for name, (xyz, rpy) in SPOTS.items()}
        )
        return {**data, "usd": usd}


class DimSimApartment(SimScene):
    config: DimSimApartmentConfig
    mounts = Mounts(*(f"{_SCENE}/{name}" for name in SPOTS))
