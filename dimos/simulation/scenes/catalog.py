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

from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.simulation.scene_assets.spec import ScenePackage, load_scene_package
from dimos.simulation.scenes.office import get_dimos_office

SCENE_PACKAGE_CACHE_DIR = Path.home() / ".cache" / "dimos" / "scene_packages"
DEFAULT_SCENE = "dimos-office"
_DISABLED_SCENE_NAMES = {"", "none", "off", "disabled", "false", "0"}
_ALIASES = {
    "default": DEFAULT_SCENE,
    "office": DEFAULT_SCENE,
    "dimos-office": DEFAULT_SCENE,
    "dimos_office": DEFAULT_SCENE,
    "street": "street-lite",
    "street-lite": "street-lite",
    "street_lite": "street-lite",
    "mall": "mall-babylon-nolights",
    "mall_babylon_nolights": "mall-babylon-nolights",
    "mall-babylon-nolights": "mall-babylon-nolights",
    "mall-proxy": "mall-proxy",
    "mall_proxy": "mall-proxy",
    "lowpoly": "lowpoly-tdm",
    "lowpoly-tdm": "lowpoly-tdm",
    "lowpoly_tdm": "lowpoly-tdm",
    "tdm": "lowpoly-tdm",
}
_PACKAGE_DIRS = {
    DEFAULT_SCENE: "dimos_office",
    "street-lite": "street_lite",
    "mall-babylon-nolights": "mall_babylon_nolights",
    "mall-proxy": "mall_proxy",
    "lowpoly-tdm": "lowpoly_tdm",
}


def resolve_scene_package(
    scene: str | Path | None = None,
    *,
    robot_mjcf_path: str | Path | None = None,
    meshdir: str | Path | None = None,
) -> ScenePackage | None:
    """Resolve a scene name, metadata path, or package directory."""
    if scene is None:
        scene = DEFAULT_SCENE

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

    if name == DEFAULT_SCENE:
        return _resolve_dimos_office(robot_mjcf_path=robot_mjcf_path, meshdir=meshdir)

    metadata_path = SCENE_PACKAGE_CACHE_DIR / _PACKAGE_DIRS[name] / "scene.meta.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"scene package '{name}' is not cooked yet: {metadata_path}")
    return load_scene_package(metadata_path)


def _resolve_dimos_office(
    *,
    robot_mjcf_path: str | Path | None,
    meshdir: str | Path | None,
) -> ScenePackage:
    office = get_dimos_office()
    metadata_path = SCENE_PACKAGE_CACHE_DIR / _PACKAGE_DIRS[DEFAULT_SCENE] / "scene.meta.json"
    expected_alignment = SceneMeshAlignment(
        scale=office.scale,
        translation=office.translation,
        rotation_zyx_deg=office.rotation_zyx_deg,
        y_up=office.y_up,
    )
    if not metadata_path.exists():
        raise FileNotFoundError(
            "dimos-office scene package is not cooked yet: "
            f"{metadata_path}. Run dimos.simulation.scene_assets.cook first."
        )

    package = load_scene_package(metadata_path)
    if (
        package.source_path == office.mesh_path
        and _alignment_matches(package.alignment, expected_alignment)
        and package.visual_path is not None
        and package.browser_collision_path is not None
        and (robot_mjcf_path is None or package.mujoco_model_path is not None)
    ):
        return package

    raise ValueError(
        "dimos-office scene package is stale or has incorrect alignment: "
        f"{metadata_path}. Recook it from the bundled office scene."
    )


def _alignment_matches(left: SceneMeshAlignment, right: SceneMeshAlignment) -> bool:
    return (
        left.scale == right.scale
        and tuple(left.translation) == tuple(right.translation)
        and tuple(left.rotation_zyx_deg) == tuple(right.rotation_zyx_deg)
        and left.y_up == right.y_up
    )


__all__ = ["DEFAULT_SCENE", "resolve_scene_package"]
