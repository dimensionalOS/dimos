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

"""Locate authored arenas and their optional DimOS semantic annotations.

Model assembly is robosuite Task/Arena/RobotModel composition in environment.py.
"""

from pathlib import Path

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.world.scene_types import SceneDescription
from dimos.utils.data import LfsPath


def scene_path(value: str | None, default: str) -> Path:
    if value is None or value == "none":
        return LfsPath("sim2/scenes") / default
    path = Path(value).expanduser()
    if not path.exists() and len(path.parts) == 1:
        path = LfsPath("sim2/scenes") / path
    if path.is_dir():
        path = path / "scene.xml"
    if not path.is_file():
        names = ", ".join(list_scenes())
        raise FileNotFoundError(f"sim2 scene {value!r} is not installed; available: {names}")
    return path.resolve()


def list_scenes() -> list[str]:
    """List installed native scene names without loading or compiling them."""
    root = Path(str(LfsPath("sim2/scenes")))
    return sorted(
        [p.name for p in root.iterdir() if p.is_dir() and (p / "scene.xml").is_file()]
        + [p.name for p in root.glob("*.xml")]
    )


def describe_scene(path: Path) -> SceneDescription:
    """Read the optional semantic sidecar; raw MJCF remains runnable without it."""
    metadata = path.with_suffix(".json")
    if not metadata.is_file():
        return SceneDescription(id=path.parent.name if path.name == "scene.xml" else path.stem)
    return SceneDescription.model_validate_json(metadata.read_text())


def scene_spawn(path: Path, spawn: str = "default") -> Pose:
    """Return a support pose; robosuite's RobotModel supplies its bottom offset."""
    description = describe_scene(path)
    if description.spawns and spawn not in description.spawns:
        raise ValueError(
            f"scene {description.id!r} has no authored {spawn!r} support; "
            f"available: {', '.join(description.spawns)}"
        )
    return description.spawns.get(spawn, Pose(0, 0, 0))
