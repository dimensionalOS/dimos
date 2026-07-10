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

"""Resolved scene cook plan.

The authored sidecar is user intent.  The cook plan is resolved source-scene
membership: exactly which source prims become each runtime entity, which visual
nodes Blender must extract/delete, and which collision policy every downstream
cook must consume.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
import re
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as R

from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec
from dimos.experimental.scene_cooking.sidecar import (
    EntityGroupSpec,
    InteractableSpec,
    SceneCookSidecar,
)
from dimos.experimental.scene_cooking.source_assets.mesh import (
    ScenePrimMesh,
    load_scene_prims,
)
from dimos.simulation.scene_assets.spec import SceneMeshAlignment

_HASH_SUFFIX_RE = re.compile(r"_[0-9a-fA-F]{6,}$")

#: Floor for a prim/entity's AABB extent (metres). Keeps zero-thickness
#: sheets (a floor tile authored with no depth) from producing a zero-size
#: physics extent.
_MIN_EXTENT_M = 1e-4


def _safe_extents(
    aabb_min: NDArray[np.float64], aabb_max: NDArray[np.float64]
) -> NDArray[np.float64]:
    """AABB extents clamped to ``_MIN_EXTENT_M`` so no axis is zero-size."""
    extents: NDArray[np.float64] = np.maximum(aabb_max - aabb_min, _MIN_EXTENT_M).astype(float)
    return extents


@dataclass(frozen=True)
class EntityCookPlan:
    """Resolved authored entity."""

    spec: InteractableSpec
    safe_id: str
    matched_prim_paths: tuple[str, ...]
    visual_node_patterns: tuple[str, ...]
    aabb_min: tuple[float, float, float]
    aabb_max: tuple[float, float, float]
    center: tuple[float, float, float]
    initial_quat: tuple[float, float, float, float]
    descriptor: dict[str, Any]
    visual_path: Path | None
    prototype_id: str | None = None

    def to_metadata(self) -> dict[str, Any]:
        metadata = {
            "id": self.spec.id,
            "tags": list(self.spec.tags),
            "source_prim_paths": list(self.spec.source_prim_paths),
            "matched_prim_paths": list(self.matched_prim_paths),
            "visual_node_patterns": list(self.visual_node_patterns),
            "remove_from_static": self.spec.remove_from_static,
            "spawn": self.spec.spawn,
            "synthetic": self.spec.is_synthetic,
            "aabb": {
                "min": list(self.aabb_min),
                "max": list(self.aabb_max),
            },
            "initial_pose": {
                "x": self.center[0],
                "y": self.center[1],
                "z": self.center[2],
                "qw": self.initial_quat[0],
                "qx": self.initial_quat[1],
                "qy": self.initial_quat[2],
                "qz": self.initial_quat[3],
            },
            "visual_path": str(self.visual_path) if self.visual_path else None,
            "descriptor": self.descriptor,
            "physics": self.spec.physics,
            "visual": self.spec.visual,
        }
        if self.prototype_id is not None:
            metadata["prototype_id"] = self.prototype_id
        return metadata


@dataclass(frozen=True)
class EntityPrototypePlan:
    """Shared source mesh cooked once and instanced by many entities."""

    id: str
    safe_id: str
    source_prim_path: str
    vertices: NDArray[np.float32]
    triangles: NDArray[np.int32]
    collision_dir: Path

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "safe_id": self.safe_id,
            "source_prim_path": self.source_prim_path,
            "vertex_count": len(self.vertices),
            "triangle_count": len(self.triangles),
            "collision_dir": str(self.collision_dir),
        }


@dataclass(frozen=True)
class SceneCookPlan:
    """Resolved plan shared by every artifact writer."""

    source_path: Path
    alignment: SceneMeshAlignment
    sidecar: SceneCookSidecar
    collision_spec: CollisionSpec
    entities: tuple[EntityCookPlan, ...] = ()
    prototypes: tuple[EntityPrototypePlan, ...] = ()
    stats: dict[str, Any] = field(default_factory=dict)

    def entities_metadata(self) -> list[dict[str, Any]]:
        return [entity.to_metadata() for entity in self.entities]

    def to_json_dict(self) -> dict[str, Any]:
        """Cook-stats summary of the plan.

        Entities appear as a count only — their full records already live
        in ``scene.meta.json`` via ``entities_metadata()``.
        """
        return {
            "source_path": str(self.source_path),
            "alignment": {
                "scale": self.alignment.scale,
                "rotation_zyx_deg": list(self.alignment.rotation_zyx_deg),
                "translation": list(self.alignment.translation),
                "y_up": self.alignment.y_up,
            },
            "sidecar_path": str(self.sidecar.path) if self.sidecar.path else None,
            "entities": len(self.entities),
            "prototypes": [prototype.to_json_dict() for prototype in self.prototypes],
            "stats": self.stats,
        }


def build_scene_cook_plan(
    source_path: str | Path,
    *,
    sidecar: SceneCookSidecar,
    alignment: SceneMeshAlignment,
    output_dir: str | Path,
    collision_spec: CollisionSpec | None = None,
) -> SceneCookPlan:
    source = Path(source_path).expanduser().resolve()
    base_collision = collision_spec or sidecar.collision
    if not sidecar.interactables and not sidecar.entity_groups:
        return SceneCookPlan(
            source_path=source,
            alignment=alignment,
            sidecar=sidecar,
            collision_spec=base_collision,
            stats={"source_prims": 0, "entities": 0},
        )

    entities_dir = Path(output_dir).expanduser().resolve() / "entities"
    needs_prims = bool(sidecar.entity_groups) or any(
        item.source_prim_paths for item in sidecar.interactables
    )
    prims = load_scene_prims(source, alignment=alignment) if needs_prims else []
    explicit_entities = tuple(
        (
            _build_synthetic_entity_plan(item, entities_dir)
            if item.is_synthetic
            else _build_matched_entity_plan(item, prims, entities_dir)
        )
        for item in sidecar.interactables
    )
    group_entities, prototypes, group_skip_patterns = _build_entity_group_plans(
        sidecar.entity_groups,
        prims,
        entities_dir,
    )
    entities = (*explicit_entities, *group_entities)
    effective_collision = _collision_spec_with_entity_skips(
        base_collision,
        entities,
        group_skip_patterns=group_skip_patterns,
    )
    return SceneCookPlan(
        source_path=source,
        alignment=alignment,
        sidecar=sidecar,
        collision_spec=effective_collision,
        entities=entities,
        prototypes=prototypes,
        stats={
            "source_prims": len(prims),
            "entities": len(entities),
            "entity_prototypes": len(prototypes),
        },
    )


def _build_matched_entity_plan(
    spec: InteractableSpec,
    prims: list[ScenePrimMesh],
    entities_dir: Path,
) -> EntityCookPlan:
    matched = sorted(
        (prim for prim in prims if spec.matches(prim)),
        key=_prim_sort_key,
    )
    if not matched:
        patterns = ", ".join(spec.source_prim_paths)
        raise ValueError(f"scene interactable {spec.id!r} matched no source prims: {patterns}")

    vertices = np.concatenate([prim.vertices for prim in matched], axis=0)
    aabb_min_np = vertices.min(axis=0).astype(float)
    aabb_max_np = vertices.max(axis=0).astype(float)
    center_np = ((aabb_min_np + aabb_max_np) * 0.5).astype(float)
    extents = _safe_extents(aabb_min_np, aabb_max_np)
    safe_id = _safe_entity_id(spec.id)
    visual_path = entities_dir / safe_id / "visual.glb"

    shape_hint, shape_extents = _resolve_shape(spec, extents)
    descriptor = _make_descriptor(spec, shape_hint, shape_extents, visual_path)

    return EntityCookPlan(
        spec=spec,
        safe_id=safe_id,
        matched_prim_paths=tuple(prim.prim_path or prim.name for prim in matched),
        visual_node_patterns=_visual_node_patterns(matched),
        aabb_min=(float(aabb_min_np[0]), float(aabb_min_np[1]), float(aabb_min_np[2])),
        aabb_max=(float(aabb_max_np[0]), float(aabb_max_np[1]), float(aabb_max_np[2])),
        center=(float(center_np[0]), float(center_np[1]), float(center_np[2])),
        initial_quat=(1.0, 0.0, 0.0, 0.0),
        descriptor=descriptor,
        visual_path=visual_path,
    )


def _build_synthetic_entity_plan(
    spec: InteractableSpec,
    entities_dir: Path,
) -> EntityCookPlan:
    """Synthetic entity: no source-prim extraction, primitive geometry,
    pose from the spec.  Used for manip rigs, test cubes, props you want
    in the scene that aren't in the asset."""
    pose = spec.pose or {}
    center = (
        float(pose.get("x", 0.0)),
        float(pose.get("y", 0.0)),
        float(pose.get("z", 0.0)),
    )
    quat = (
        float(pose.get("qw", 1.0)),
        float(pose.get("qx", 0.0)),
        float(pose.get("qy", 0.0)),
        float(pose.get("qz", 0.0)),
    )
    extents_raw = spec.physics.get("extents")
    if not extents_raw:
        raise ValueError(
            f"synthetic interactable {spec.id!r}: physics.extents required "
            f"(no source mesh to derive bounds from)"
        )
    extents_np = np.asarray([float(v) for v in extents_raw], dtype=float)
    half = extents_np / 2.0 if len(extents_np) == 3 else extents_np
    aabb_half = np.zeros(3, dtype=float)
    aabb_half[: len(half)] = half[:3] if len(half) >= 3 else half
    aabb_min = tuple(c - h for c, h in zip(center, aabb_half, strict=True))
    aabb_max = tuple(c + h for c, h in zip(center, aabb_half, strict=True))

    shape_hint, shape_extents = _resolve_shape(spec, extents_np)
    safe_id = _safe_entity_id(spec.id)
    descriptor = _make_descriptor(spec, shape_hint, shape_extents, visual_path=None)

    return EntityCookPlan(
        spec=spec,
        safe_id=safe_id,
        matched_prim_paths=(),
        visual_node_patterns=(),
        aabb_min=aabb_min,
        aabb_max=aabb_max,
        center=center,
        initial_quat=quat,
        descriptor=descriptor,
        visual_path=None,
    )


def _build_entity_group_plans(
    groups: tuple[EntityGroupSpec, ...],
    prims: list[ScenePrimMesh],
    entities_dir: Path,
) -> tuple[tuple[EntityCookPlan, ...], tuple[EntityPrototypePlan, ...], tuple[str, ...]]:
    entities: list[EntityCookPlan] = []
    prototypes_by_id: dict[str, EntityPrototypePlan] = {}
    group_skip_patterns: list[str] = []

    for group in groups:
        matched = sorted((prim for prim in prims if group.matches(prim)), key=_prim_sort_key)
        if not matched:
            patterns = ", ".join(group.source_prim_paths)
            raise ValueError(
                f"scene entity group {group.id_prefix!r} matched no source prims: {patterns}"
            )
        if group.remove_from_static:
            group_skip_patterns.extend(group.source_prim_paths)

        physics = {"shape": "mesh", **group.physics}
        for index, prim in enumerate(matched):
            entity, prototype = _build_group_entity_plan(
                group,
                prim,
                index=index,
                physics=physics,
                entities_dir=entities_dir,
            )
            entities.append(entity)
            prototypes_by_id.setdefault(prototype.id, prototype)

    return tuple(entities), tuple(prototypes_by_id.values()), tuple(group_skip_patterns)


def _build_group_entity_plan(
    group: EntityGroupSpec,
    prim: ScenePrimMesh,
    *,
    index: int,
    physics: dict[str, Any],
    entities_dir: Path,
) -> tuple[EntityCookPlan, EntityPrototypePlan]:
    prim_path = prim.prim_path or prim.name
    prototype_key = _entity_group_prototype_key(group, prim)
    prototype_safe_id = _safe_entity_id(f"{group.id_prefix}_{prototype_key}")
    entity_id = f"{group.id_prefix}_{index:05d}_{prototype_safe_id}"
    spec = InteractableSpec(
        id=entity_id,
        source_prim_paths=(prim_path,),
        remove_from_static=group.remove_from_static,
        spawn=group.spawn,
        kind=group.kind,
        mass=group.mass,
        tags=group.tags,
        physics=physics,
        visual=group.visual,
    )

    vertices = np.asarray(prim.vertices, dtype=np.float64)
    aabb_min_np = vertices.min(axis=0).astype(float)
    aabb_max_np = vertices.max(axis=0).astype(float)
    extents = _safe_extents(aabb_min_np, aabb_max_np)
    local_vertices, center_np, quat = _localize_prim_mesh(vertices)
    shape_hint, shape_extents = _resolve_shape(spec, extents)
    descriptor = _make_descriptor(spec, shape_hint, shape_extents, visual_path=None)
    descriptor["prototype_id"] = prototype_safe_id

    entity = EntityCookPlan(
        spec=spec,
        safe_id=_safe_entity_id(entity_id),
        matched_prim_paths=(prim_path,),
        visual_node_patterns=(),
        aabb_min=(float(aabb_min_np[0]), float(aabb_min_np[1]), float(aabb_min_np[2])),
        aabb_max=(float(aabb_max_np[0]), float(aabb_max_np[1]), float(aabb_max_np[2])),
        center=(float(center_np[0]), float(center_np[1]), float(center_np[2])),
        initial_quat=quat,
        descriptor=descriptor,
        visual_path=None,
        prototype_id=prototype_safe_id,
    )
    prototype = EntityPrototypePlan(
        id=prototype_safe_id,
        safe_id=prototype_safe_id,
        source_prim_path=prim_path,
        vertices=local_vertices.astype(np.float32),
        triangles=np.asarray(prim.triangles, dtype=np.int32),
        collision_dir=entities_dir / "_prototypes" / prototype_safe_id / "mujoco_collision",
    )
    return entity, prototype


def _resolve_shape(
    spec: InteractableSpec,
    extents_np: NDArray[np.float64],
) -> tuple[str, list[float]]:
    shape_hint = str(spec.physics.get("shape", "box"))
    shape_extents = spec.physics.get("extents")
    if shape_extents is not None:
        return shape_hint, [float(v) for v in shape_extents]
    if shape_hint == "box":
        return shape_hint, [float(v) for v in extents_np[:3]]
    if shape_hint == "sphere":
        return shape_hint, [float(max(extents_np) * 0.5)]
    if shape_hint == "cylinder":
        return shape_hint, [
            float(max(extents_np[0], extents_np[1]) * 0.5),
            float(extents_np[2] if len(extents_np) >= 3 else extents_np[-1]),
        ]
    return shape_hint, []


def _make_descriptor(
    spec: InteractableSpec,
    shape_hint: str,
    shape_extents: list[float],
    visual_path: Path | None,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {
        "entity_id": spec.id,
        "kind": spec.kind,
        "shape_hint": shape_hint,
        "extents": [float(value) for value in shape_extents],
        "mass": float(spec.mass),
    }
    if visual_path is not None:
        safe_id = _safe_entity_id(spec.id)
        descriptor["mesh_ref"] = f"entities/{safe_id}/visual.glb"
    else:
        descriptor["mesh_ref"] = ""
    rgba = spec.visual.get("rgba") if spec.visual else None
    if isinstance(rgba, list | tuple) and len(rgba) == 4:
        descriptor["rgba"] = [float(v) for v in rgba]
    return descriptor


def _entity_group_prototype_key(group: EntityGroupSpec, prim: ScenePrimMesh) -> str:
    prim_path = prim.prim_path or prim.visual_node_name or prim.name
    if group.prototype_key == "prim_path":
        return prim_path
    if group.prototype_key != "mesh_name":
        raise ValueError(
            f"entity group {group.id_prefix!r}: unsupported prototype_key {group.prototype_key!r}"
        )

    basename = prim_path.lstrip("/").rsplit("/", 1)[-1]
    if "__" in basename:
        basename = basename.split("__", 1)[1]
    basename = basename.rsplit("_Mesh", 1)[0]
    repeated = re.match(r"^(.+?)\.\d+_\1$", basename)
    if repeated:
        return repeated.group(1)
    return _HASH_SUFFIX_RE.sub("", basename)


def _localize_prim_mesh(
    vertices: NDArray[np.float64],
) -> tuple[NDArray[np.float64], tuple[float, float, float], tuple[float, float, float, float]]:
    aabb_min = vertices.min(axis=0)
    aabb_max = vertices.max(axis=0)
    center = (aabb_min + aabb_max) * 0.5
    centered = vertices - center
    cov = centered.T @ centered
    _, axes = np.linalg.eigh(cov)
    axes = axes[:, ::-1]
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] *= -1.0
    local_vertices = centered @ axes
    quat_xyzw = R.from_matrix(axes).as_quat()
    quat_wxyz = (
        float(quat_xyzw[3]),
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
    )
    return (
        local_vertices,
        (float(center[0]), float(center[1]), float(center[2])),
        quat_wxyz,
    )


def _visual_node_patterns(prims: list[ScenePrimMesh]) -> tuple[str, ...]:
    names: list[str] = []
    for prim in prims:
        prim_path = prim.visual_node_name or prim.prim_path or prim.name
        basename = prim_path.lstrip("/").rsplit("/", 1)[-1]
        visual_name = _HASH_SUFFIX_RE.sub("", basename)
        if visual_name not in names:
            names.append(visual_name)
    return tuple(names)


def _collision_spec_with_entity_skips(
    collision_spec: CollisionSpec,
    entities: tuple[EntityCookPlan, ...],
    *,
    group_skip_patterns: tuple[str, ...] = (),
) -> CollisionSpec:
    entity_skip_overrides: dict[str, dict[str, Any]] = {}
    for pattern in group_skip_patterns:
        entity_skip_overrides[pattern] = {"type": "skip", "visual": False}
    for entity in entities:
        if entity.prototype_id is not None:
            continue
        if not entity.spec.remove_from_static:
            continue
        for prim_path in sorted(entity.matched_prim_paths):
            entity_skip_overrides[prim_path] = {"type": "skip", "visual": False}

    # CollisionSpec.resolve() is first-match-wins. Entity extraction must
    # take precedence over broad class overrides such as "Grocery_Scatter_*",
    # otherwise extracted dynamic entities are duplicated in static collision.
    prim_overrides: dict[str, dict[str, Any]] = {
        **entity_skip_overrides,
        **collision_spec.prim_overrides,
    }
    return replace(collision_spec, prim_overrides=prim_overrides)


def _prim_sort_key(prim: ScenePrimMesh) -> tuple[str, str]:
    return (prim.prim_path or prim.name, prim.name)


def _safe_entity_id(entity_id: str) -> str:
    safe = "".join(c if c.isalnum() or c in {"-", "_", "."} else "_" for c in entity_id)
    return safe or "entity"
