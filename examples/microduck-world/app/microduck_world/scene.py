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

"""Load project scene artifacts and named navigation locations."""

from pathlib import Path
from typing import Self

from dimos.robot.pollen.microduck.places import RoomSpec
from dimos.simulation.scene_assets.spec import ScenePackage, load_scene_package
from pydantic import BaseModel, ConfigDict, Field, model_validator

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class WorldScene(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, str_strip_whitespace=True)

    id: str = Field(min_length=1)
    spawn_xy: tuple[float, float]
    rooms: dict[str, RoomSpec]
    objects: dict[str, tuple[float, float]]

    @model_validator(mode="after")
    def validate_places(self) -> Self:
        names: set[str] = set()
        for key, room in self.rooms.items():
            if key != room.name:
                raise ValueError(f"Room key {key!r} must match its name {room.name!r}")
            xmin, xmax, ymin, ymax = room.bounds
            if xmin >= xmax or ymin >= ymax:
                raise ValueError(f"Room {key!r} must have increasing x/y bounds")
            if not room.contains(room.target[0], room.target[1]):
                raise ValueError(f"Room {key!r} navigation target must be inside its bounds")
            for name in (room.name, *room.aliases):
                normalized = name.strip().casefold()
                if not normalized or normalized in names:
                    raise ValueError(f"Place name or alias is empty or ambiguous: {name!r}")
                names.add(normalized)
        for name in self.objects:
            normalized = name.strip().casefold()
            if not normalized or normalized in names:
                raise ValueError(f"Place name is empty or ambiguous: {name!r}")
            names.add(normalized)
        return self


def load_world() -> tuple[ScenePackage, WorldScene]:
    package = load_scene_package(PROJECT_ROOT / "assets/scenes/apartment/scene.meta.json")
    if package.mujoco_scene_path is None or package.objects_path is None:
        raise ValueError("World scene requires MuJoCo geometry and place metadata")
    if not package.mujoco_scene_path.is_file():
        raise FileNotFoundError(package.mujoco_scene_path)
    return package, WorldScene.model_validate_json(package.objects_path.read_text())
