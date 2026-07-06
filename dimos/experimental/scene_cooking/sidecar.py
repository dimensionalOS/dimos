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

"""Authored scene-cook sidecar.

The existing ``<scene>.collision.json`` file remains the low-level collision
contract.  ``<scene>.cook.json`` is the wider authored-scene contract: it can
carry the same collision policy plus a small, explicit list of objects that
should be removed from static cooks and respawned as scene-package entities.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import fnmatch
import json
from pathlib import Path
from typing import Any, Literal

from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec
from dimos.experimental.scene_cooking.source_assets.mesh import ScenePrimMesh
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

CookEntitySpawn = Literal["initial", "manual"]
CookEntityKind = Literal["dynamic", "kinematic", "static"]
EntityGroupMode = Literal["per_prim"]

_COOK_SIDECAR_SUFFIXES = (".cook.json", ".scene.json")


@dataclass(frozen=True)
class InteractableSpec:
    """One hand-authored runtime entity.

    Two flavours:
      * ``source_prim_paths`` set -> matched against scene prims, geometry
        extracted from the source mesh (the chairs flow).
      * ``source_prim_paths`` empty + ``pose`` set -> synthetic entity
        with no scene-mesh source. Geometry comes from ``physics.shape``
        + ``physics.extents``; pose comes from the spec directly. Used for
        manip rigs, test cubes, anything not already in the source asset.
    """

    id: str
    source_prim_paths: tuple[str, ...] = ()
    pose: dict[str, float] | None = None
    remove_from_static: bool = True
    spawn: CookEntitySpawn = "initial"
    kind: CookEntityKind = "dynamic"
    mass: float = 1.0
    tags: tuple[str, ...] = ()
    physics: dict[str, Any] = field(default_factory=dict)
    visual: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> InteractableSpec:
        prims = raw.get("source_prim_paths", raw.get("prim_paths", ()))
        if isinstance(prims, str):
            prims = (prims,)
        pose = raw.get("pose")
        if not prims and pose is None:
            raise ValueError(
                f"interactable {raw.get('id')!r}: needs either source_prim_paths "
                f"(extract from scene) or pose (synthetic entity)"
            )
        tags = raw.get("tags", ())
        if isinstance(tags, str):
            tags = (tags,)
        return cls(
            id=str(raw["id"]),
            source_prim_paths=tuple(str(pattern) for pattern in prims),
            pose=dict(pose) if pose is not None else None,
            remove_from_static=bool(raw.get("remove_from_static", True)),
            spawn=raw.get("spawn", "initial"),
            kind=raw.get("kind", "dynamic"),
            mass=float(raw.get("mass", 1.0)),
            tags=tuple(str(tag) for tag in tags),
            physics=dict(raw.get("physics", {})),
            visual=dict(raw.get("visual", {})),
        )

    @property
    def is_synthetic(self) -> bool:
        return not self.source_prim_paths and self.pose is not None

    def to_json_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["source_prim_paths"] = list(self.source_prim_paths)
        raw["tags"] = list(self.tags)
        return raw

    def matches(self, prim: ScenePrimMesh) -> bool:
        prim_candidates = tuple(
            candidate for candidate in (prim.visual_node_name, prim.prim_path) if candidate
        )
        return any(
            match_prim_pattern(candidate, pattern, include_sanitized=False)
            for candidate in prim_candidates
            for pattern in self.source_prim_paths
        )


@dataclass(frozen=True)
class EntityGroupSpec:
    """Pattern-authored runtime entities expanded from many source prims.

    ``mode="per_prim"`` creates one runtime entity for each matched source
    prim.  The cook plan can then share collision prototypes across repeated
    source mesh names instead of decomposing every instance independently.
    """

    id_prefix: str
    source_prim_paths: tuple[str, ...]
    mode: EntityGroupMode = "per_prim"
    remove_from_static: bool = True
    spawn: CookEntitySpawn = "initial"
    kind: CookEntityKind = "dynamic"
    mass: float = 1.0
    tags: tuple[str, ...] = ()
    physics: dict[str, Any] = field(default_factory=dict)
    visual: dict[str, Any] = field(default_factory=dict)
    prototype_key: str = "mesh_name"

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> EntityGroupSpec:
        prims = raw.get("source_prim_paths", raw.get("prim_paths", ()))
        if isinstance(prims, str):
            prims = (prims,)
        if not prims:
            raise ValueError(f"entity group {raw.get('id_prefix')!r}: source_prim_paths required")
        tags = raw.get("tags", ())
        if isinstance(tags, str):
            tags = (tags,)
        mode = raw.get("mode", "per_prim")
        if mode != "per_prim":
            raise ValueError(f"entity group {raw.get('id_prefix')!r}: unsupported mode {mode!r}")
        return cls(
            id_prefix=str(raw["id_prefix"]),
            source_prim_paths=tuple(str(pattern) for pattern in prims),
            mode=mode,
            remove_from_static=bool(raw.get("remove_from_static", True)),
            spawn=raw.get("spawn", "initial"),
            kind=raw.get("kind", "dynamic"),
            mass=float(raw.get("mass", 1.0)),
            tags=tuple(str(tag) for tag in tags),
            physics=dict(raw.get("physics", {})),
            visual=dict(raw.get("visual", {})),
            prototype_key=str(raw.get("prototype_key", "mesh_name")),
        )

    def to_json_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["source_prim_paths"] = list(self.source_prim_paths)
        raw["tags"] = list(self.tags)
        return raw

    def matches(self, prim: ScenePrimMesh) -> bool:
        prim_candidates = tuple(
            candidate for candidate in (prim.visual_node_name, prim.prim_path) if candidate
        )
        return any(
            match_prim_pattern(candidate, pattern, include_sanitized=False)
            for candidate in prim_candidates
            for pattern in self.source_prim_paths
        )


@dataclass(frozen=True)
class SceneCookSidecar:
    """Authored policy loaded from ``<scene>.cook.json``.

    ``collision`` is exactly the older collision sidecar schema.  Interactables
    add surgical scene knowledge without forcing every object in a scene to
    become semantic metadata.
    """

    path: Path | None = None
    collision: CollisionSpec = field(default_factory=CollisionSpec)
    interactables: tuple[InteractableSpec, ...] = ()
    entity_groups: tuple[EntityGroupSpec, ...] = ()

    @classmethod
    def from_dict(cls, raw: dict[str, Any], *, path: Path | None = None) -> SceneCookSidecar:
        collision_raw = raw.get("collision")
        if isinstance(collision_raw, dict):
            collision = CollisionSpec.from_dict(collision_raw)
        else:
            # Accept old collision keys at top-level so authored sidecars can be
            # promoted without wrapping every existing key manually.
            collision = CollisionSpec.from_dict(raw)
        interactables = tuple(
            InteractableSpec.from_dict(item) for item in raw.get("interactables", ())
        )
        entity_groups = tuple(
            EntityGroupSpec.from_dict(item) for item in raw.get("entity_groups", ())
        )
        return cls(
            path=path,
            collision=collision,
            interactables=interactables,
            entity_groups=entity_groups,
        )

    @classmethod
    def from_json(cls, path: str | Path) -> SceneCookSidecar:
        sidecar_path = Path(path).expanduser().resolve()
        return cls.from_dict(json.loads(sidecar_path.read_text()), path=sidecar_path)

    @classmethod
    def auto_discover(cls, scene_path: str | Path) -> SceneCookSidecar:
        source = Path(scene_path).expanduser().resolve()
        for suffix in _COOK_SIDECAR_SUFFIXES:
            sidecar = source.with_suffix(suffix)
            if sidecar.exists():
                logger.info("loading scene cook sidecar", path=sidecar)
                return cls.from_json(sidecar)

        legacy_collision = source.with_suffix(".collision.json")
        if legacy_collision.exists():
            logger.info("loading legacy collision sidecar", path=legacy_collision)
            return cls(path=legacy_collision, collision=CollisionSpec.from_json(legacy_collision))
        return cls()

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "path": str(self.path) if self.path else None,
            "collision": asdict(self.collision),
            "interactables": [item.to_json_dict() for item in self.interactables],
            "entity_groups": [item.to_json_dict() for item in self.entity_groups],
        }


def match_prim_pattern(
    prim_path: str,
    pattern: str,
    *,
    include_sanitized: bool = True,
) -> bool:
    stripped = prim_path.lstrip("/")
    sanitized = "".join(c if c.isalnum() else "_" for c in stripped)
    basename = stripped.rsplit("/", 1)[-1]
    candidates = [prim_path, stripped, basename]
    if include_sanitized:
        candidates.append(sanitized)
    return any(fnmatch.fnmatchcase(candidate, pattern) for candidate in candidates)
