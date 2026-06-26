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

"""Scene package contracts for offline asset cooking.

Runtime modules consume the artifacts described here; they do not perform
the heavy bake themselves.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

ARTIFACT_FRAMES = {
    "browser_visual": "source",
    "browser_collision": "source",
    "mujoco": "dimos_world",
}


@dataclass(frozen=True)
class SceneMeshAlignment:
    """Transform from a raw scene asset frame into DimOS world frame.

    Cookers use this to produce artifacts; runtime consumers keep it in
    package metadata so viewers and simulators know each artifact's frame.
    """

    scale: float = 1.0
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    y_up: bool = True


@dataclass(frozen=True)
class BrowserVisualSpec:
    """Browser-rendered asset policy."""

    enabled: bool = True
    output_name: str = "visual.glb"
    optimizer: str = "gltfpack"
    simplify_ratio: float = 0.3
    simplify_error: float = 0.02
    texture_format: str | None = None
    max_texture_size: int | None = None
    max_meshes: int = 200
    max_materials: int = 50
    max_textures: int = 750
    max_vertices: int = 750_000
    max_vertex_growth_ratio: float = 1.25


@dataclass(frozen=True)
class BrowserCollisionSpec:
    """Browser raycast/physics collision asset policy."""

    enabled: bool = True
    output_name: str = "collision.glb"
    target_faces: int = 100_000


@dataclass(frozen=True)
class MujocoSceneSpec:
    """MuJoCo collision asset policy."""

    enabled: bool = True
    include_visual_mesh: bool = False


@dataclass(frozen=True)
class SceneCookSpec:
    """Complete cook input for one source scene."""

    source_path: Path
    alignment: SceneMeshAlignment = field(default_factory=SceneMeshAlignment)
    browser_visual: BrowserVisualSpec = field(default_factory=BrowserVisualSpec)
    browser_collision: BrowserCollisionSpec = field(default_factory=BrowserCollisionSpec)
    mujoco: MujocoSceneSpec = field(default_factory=MujocoSceneSpec)


@dataclass(frozen=True)
class ScenePackage:
    """Cooked scene outputs for runtime modules."""

    package_dir: Path
    source_path: Path
    alignment: SceneMeshAlignment
    visual_path: Path | None = None
    browser_collision_path: Path | None = None
    objects_path: Path | None = None
    mujoco_scene_path: Path | None = None
    metadata_path: Path | None = None
    entities: list[dict[str, Any]] = field(default_factory=list)
    stats: dict[str, Any] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        package_dir = self.package_dir.expanduser().resolve()

        return {
            "source_path": str(self.source_path),
            "package_dir": ".",
            "alignment": asdict(self.alignment),
            "artifact_frames": ARTIFACT_FRAMES,
            "artifacts": {
                "browser_visual": _serialize_package_path(self.visual_path, package_dir),
                "browser_collision": _serialize_package_path(
                    self.browser_collision_path,
                    package_dir,
                ),
                "objects": _serialize_package_path(self.objects_path, package_dir),
                "mujoco_scene": _serialize_package_path(self.mujoco_scene_path, package_dir),
            },
            "entities": _serialize_entity_paths(self.entities, package_dir),
            "stats": self.stats,
        }

    def write_metadata(self, path: Path | None = None) -> Path:
        metadata_path = path or self.metadata_path or (self.package_dir / "scene.meta.json")
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(self.to_json_dict(), indent=2, sort_keys=True) + "\n")
        return metadata_path


def load_scene_package(path: str | Path) -> ScenePackage:
    """Load a previously written ``scene.meta.json``."""
    metadata_path = Path(path).expanduser().resolve()
    raw = json.loads(metadata_path.read_text())
    _validate_artifact_frames(raw, metadata_path)
    artifacts = raw.get("artifacts", {})
    align = SceneMeshAlignment(**raw["alignment"])
    package_dir = _resolve_package_dir(raw.get("package_dir"), metadata_path)
    return ScenePackage(
        package_dir=package_dir,
        source_path=Path(raw["source_path"]),
        alignment=align,
        visual_path=_resolve_package_path(artifacts.get("browser_visual"), package_dir),
        browser_collision_path=(
            _resolve_package_path(artifacts.get("browser_collision"), package_dir)
        ),
        objects_path=_resolve_package_path(artifacts.get("objects"), package_dir),
        mujoco_scene_path=_resolve_package_path(artifacts.get("mujoco_scene"), package_dir),
        metadata_path=metadata_path,
        entities=_resolve_entity_paths(raw.get("entities", []), package_dir),
        stats=raw.get("stats", {}),
    )


def _validate_artifact_frames(raw: dict[str, Any], metadata_path: Path) -> None:
    frames = raw.get("artifact_frames")
    if frames is None:
        raise ValueError(
            f"scene package is missing artifact frame metadata: {metadata_path}. "
            "Recook it with dimos.experimental.pimsim.scene.cook."
        )

    artifacts = raw.get("artifacts", {})
    required = {
        "browser_visual": "browser_visual",
        "browser_collision": "browser_collision",
        "mujoco_scene": "mujoco",
    }
    for artifact_name, frame_name in required.items():
        if artifacts.get(artifact_name) and frames.get(frame_name) != ARTIFACT_FRAMES[frame_name]:
            raise ValueError(
                f"scene package artifact frame mismatch in {metadata_path}: "
                f"{frame_name}={frames.get(frame_name)!r}, "
                f"expected {ARTIFACT_FRAMES[frame_name]!r}. Recook the scene package."
            )


def _serialize_package_path(path: Path | None, package_dir: Path) -> str | None:
    if path is None:
        return None
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(package_dir).as_posix()
    except ValueError:
        return str(resolved)


def _resolve_package_dir(raw: str | None, metadata_path: Path) -> Path:
    if raw is None:
        return metadata_path.parent
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return (metadata_path.parent / path).resolve()


def _resolve_package_path(raw: str | None, package_dir: Path) -> Path | None:
    if not raw:
        return None
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return (package_dir / path).resolve()


_ENTITY_PATH_KEYS = ("visual_path", "collision_path", "mesh_path")
_ENTITY_PATH_LIST_KEYS = ("collision_paths",)


def _serialize_entity_paths(
    entities: list[dict[str, Any]], package_dir: Path
) -> list[dict[str, Any]]:
    out = deepcopy(entities)
    for entity in out:
        _rewrite_entity_paths(entity, package_dir, serialize=True)
    return out


def _resolve_entity_paths(
    entities: list[dict[str, Any]], package_dir: Path
) -> list[dict[str, Any]]:
    out = deepcopy(entities)
    for entity in out:
        _rewrite_entity_paths(entity, package_dir, serialize=False)
    return out


def _rewrite_entity_paths(
    entity: dict[str, Any],
    package_dir: Path,
    *,
    serialize: bool,
) -> None:
    def rewrite(value: Any) -> Any:
        if not isinstance(value, str):
            return value
        path = Path(value).expanduser()
        if serialize:
            return _serialize_package_path(path, package_dir)
        if path.is_absolute():
            return str(path)
        return str((package_dir / path).resolve())

    for key in _ENTITY_PATH_KEYS:
        if key in entity:
            entity[key] = rewrite(entity[key])

    for key in _ENTITY_PATH_LIST_KEYS:
        value = entity.get(key)
        if isinstance(value, list):
            entity[key] = [rewrite(item) for item in value]

    artifacts = entity.get("artifacts")
    if isinstance(artifacts, dict):
        for key, value in list(artifacts.items()):
            artifacts[key] = rewrite(value)
