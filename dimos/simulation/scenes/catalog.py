# Copyright 2025-2026 Dimensional Inc.
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

"""Named scene packages for simulation/runtime viewers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.simulation.scene_assets.spec import ScenePackage, load_scene_package
from dimos.utils.data import get_data

DEFAULT_SCENE = "office"
_DISABLED_SCENE_NAMES = {"", "none", "off", "disabled", "false", "0"}
_ALIASES = {
    "office": DEFAULT_SCENE,
    "dimos-office": DEFAULT_SCENE,
    "dimos_office": DEFAULT_SCENE,
    "supermarket": "supermarket",
    "dimos-supermarket": "supermarket",
    "dimos_supermarket": "supermarket",
}
_PACKAGE_DIRS = {
    DEFAULT_SCENE: "dimos_office",
    "supermarket": "supermarket",
}


def resolve_scene_package(
    scene: str | Path | None = None,
    **_legacy: Any,
) -> ScenePackage | None:
    """Resolve a scene name, metadata path, or package directory.

    ``robot_mjcf_path`` and ``meshdir`` are accepted as legacy keyword
    arguments and ignored; scene packages are now robot-agnostic and the
    runtime composer attaches the robot via ``MjSpec.attach()``.
    """
    if scene is None:
        return None

    scene_text = str(scene).strip()
    if scene_text.lower() in _DISABLED_SCENE_NAMES:
        return None

    candidate = Path(scene_text).expanduser()
    if candidate.exists():
        if candidate.is_dir():
            metadata_path = candidate / "scene.meta.json"
            if not metadata_path.exists():
                raise FileNotFoundError(f"scene package directory has no {metadata_path.name}")
        elif candidate.name == "scene.meta.json" or candidate.suffix.lower() == ".json":
            metadata_path = candidate
        else:
            raise ValueError(
                "scene paths must point to a cooked scene.meta.json or package directory; "
                f"got raw asset path: {candidate}"
            )
        return load_scene_package(metadata_path)

    name = _ALIASES.get(scene_text.lower())
    if name is None:
        known = ", ".join(sorted(_PACKAGE_DIRS))
        raise ValueError(f"unknown scene '{scene_text}'. Known scenes: {known}")

    return _resolve_named_scene_package(name)


def _resolve_named_scene_package(name: str) -> ScenePackage:
    metadata_path = _scene_package_dir() / _PACKAGE_DIRS[name] / "scene.meta.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"scene package '{name}' is not cooked yet: {metadata_path}")
    package = load_scene_package(metadata_path)
    _validate_package_artifacts(package, name, metadata_path)
    return package


def _validate_package_artifacts(
    package: ScenePackage,
    name: str,
    metadata_path: Path,
) -> None:
    rerun_visual_path = package.browser_visual_path("rerun")
    if (
        rerun_visual_path is not None
        and package.mujoco_scene_path is not None
        and rerun_visual_path.exists()
        and package.mujoco_scene_path.exists()
        and (package.browser_collision_path is None or package.browser_collision_path.exists())
    ):
        return

    raise ValueError(
        f"scene package '{name}' is incomplete or has missing artifacts: "
        f"{metadata_path}. Recook it from the source scene."
    )


def _scene_package_dir() -> Path:
    return get_data("scene_packages")
