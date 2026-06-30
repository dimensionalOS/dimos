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

"""Compose ``ScenePackage.entities`` metadata into MuJoCo bodies.

The cook step removes entity prims (chairs, props) from the static
scene bake and writes their per-entity GLBs and metadata to the
package's ``entities/`` directory. At runtime, ``MujocoSimModule``
attaches the robot first, then calls
:func:`add_scene_package_entities_to_spec` so the robot keeps the first
freejoint/qpos block and cooked entities become first-class bodies after it
in the composed model.

Entities with ``kind == "dynamic"`` and positive mass receive a
freejoint (robot can push/grasp them); anything else is welded static.
Collision: primitive shapes (box/sphere/cylinder) use the descriptor
extents; mesh entities load the CoACD hulls cooked into the package
(``collision_paths`` in ``scene.meta.json``, written by
``dimos.experimental.scene_cooking.entities.collision``). There is no
runtime decomposition — a mesh entity without cooked hulls falls back
to its AABB box with a warning to re-cook the package.

A spawn-contact audit can be run on the compiled model to weld
entities that start in deep penetration with the static scene; see
:func:`find_scene_package_entity_spawn_penetrators`. The runtime caller
(``MujocoSimModule``) chooses when to invoke it.

Body naming: ``entity:<entity_id>`` — consumers map MuJoCo bodies back
to entity ids through this prefix.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import mujoco

logger = setup_logger()

ENTITY_BODY_PREFIX = "entity:"

_MIN_HALF_EXTENT = 0.01
# Sliding friction 0.3 ≈ furniture that scoots when bumped. Entity geoms
# carry priority=1 so this wins the contact pair outright (MuJoCo's
# default combine rule is element-wise max, which would let the μ=1.0
# floor override it). Graspable props override via ``physics.friction``.
_DEFAULT_FRICTION = (0.3, 0.05, 0.001)
_DEFAULT_RGBA = (0.62, 0.62, 0.68, 1.0)
# Same geom group as the baked static scene so depth-based lidar renders
# (which hide robot groups 0/1) still see entities.
_ENTITY_GEOM_GROUP = 3
# Spawn-contact audit: deeper penetration than this at t=0 demotes the
# entity to static instead of letting MuJoCo eject it.
_SPAWN_PENETRATION_LIMIT_M = 0.02


def scene_package_entity_body_name(entity_id: str) -> str:
    return f"{ENTITY_BODY_PREFIX}{entity_id}"


def _entity_id_from_geom_name(name: str) -> str | None:
    if not name.startswith(ENTITY_BODY_PREFIX):
        return None
    rest = name[len(ENTITY_BODY_PREFIX) :]
    marker = ":geom"
    marker_idx = rest.rfind(marker)
    if marker_idx < 0:
        return None
    return rest[:marker_idx]


def _initial_entities(entities: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [e for e in entities if e.get("spawn", "initial") == "initial"]


def _box_size_and_offset(entity: dict[str, Any]) -> tuple[list[float], list[float]] | None:
    """Half-extents + geom offset (body frame) for an entity's collision box."""
    descriptor = entity.get("descriptor", {})
    shape = descriptor.get("shape_hint", "mesh")
    extents = [float(x) for x in descriptor.get("extents", [])]

    if shape == "box" and len(extents) == 3:
        half = [max(x / 2.0, _MIN_HALF_EXTENT) for x in extents]
        return half, [0.0, 0.0, 0.0]
    if shape == "sphere" and len(extents) == 1:
        r = max(extents[0], _MIN_HALF_EXTENT)
        return [r, r, r], [0.0, 0.0, 0.0]
    if shape == "cylinder" and len(extents) == 2:
        r = max(extents[0], _MIN_HALF_EXTENT)
        h = max(extents[1] / 2.0, _MIN_HALF_EXTENT)
        return [r, r, h], [0.0, 0.0, 0.0]

    # Mesh entities: box from the cooked world-frame AABB. Cook poses are
    # identity-rotation, so the AABB is axis-aligned in the body frame too.
    aabb = entity.get("aabb")
    pose = entity.get("initial_pose")
    if not aabb or not pose:
        return None
    lo = [float(x) for x in aabb["min"]]
    hi = [float(x) for x in aabb["max"]]
    half = [max((h - low) / 2.0, _MIN_HALF_EXTENT) for low, h in zip(lo, hi, strict=True)]
    center = [(h + low) / 2.0 for low, h in zip(lo, hi, strict=True)]
    origin = [float(pose.get(k, 0.0)) for k in ("x", "y", "z")]
    offset = [c - o for c, o in zip(center, origin, strict=True)]
    return half, offset


def _entity_collision_hulls(entity: dict[str, Any]) -> list[Path]:
    """Cooked collision hulls for a mesh entity, from package metadata.

    ``collision_paths`` is written by the cooker (CoACD decomposition of
    the entity's visual GLB) and resolved to absolute paths by
    ``load_scene_package``. The cooked per-entity GLBs are entity-local
    (origin = initial_pose, world axes), so hulls drop in with zero geom
    offset. All-or-nothing: a partially missing hull set falls back to
    the AABB box rather than colliding with half a chair.
    """
    raw = entity.get("collision_paths")
    if not isinstance(raw, list) or not raw:
        return []
    paths = [Path(item) for item in raw if isinstance(item, str) and item]
    missing = [path for path in paths if not path.exists()]
    if missing or not paths:
        logger.warning(
            "entity %s: %d/%d cooked collision hulls missing (e.g. %s); "
            "falling back to AABB box — re-cook the scene package",
            entity.get("id"),
            len(missing),
            len(paths),
            missing[0] if missing else "<none listed>",
        )
        return []
    return paths


def _entity_mesh_asset_name(path: Path) -> str:
    safe_stem = "".join(c if c.isalnum() else "_" for c in path.stem).strip("_")
    digest = hashlib.sha256(str(path).encode()).hexdigest()[:12]
    return f"entity_hull_{safe_stem}_{digest}"


def _entity_friction(entity: dict[str, Any]) -> tuple[float, float, float]:
    """``physics.friction`` from entity metadata (scalar sliding or full
    [sliding, torsional, rolling] triple), else the scoot-able default."""
    raw = entity.get("physics", {}).get("friction")
    sliding, torsional, rolling = _DEFAULT_FRICTION
    if isinstance(raw, int | float):
        sliding = float(raw)
    elif isinstance(raw, list | tuple) and len(raw) == 3:
        sliding, torsional, rolling = (float(v) for v in raw)
    return sliding, torsional, rolling


def _entity_rgba(descriptor: dict[str, Any]) -> tuple[float, float, float, float]:
    raw = descriptor.get("rgba")
    if isinstance(raw, list | tuple) and len(raw) == 4:
        return tuple(float(v) for v in raw)  # type: ignore[return-value]
    return _DEFAULT_RGBA


def add_scene_package_entities_to_spec(
    spec: mujoco.MjSpec,
    entities: list[dict[str, Any]],
    *,
    force_static: frozenset[str] = frozenset(),
) -> None:
    """Append scene-package entities as bodies on ``spec.worldbody``.

    Call after attaching the robot. Each ``spawn=="initial"`` entity
    becomes one body named ``entity:<id>`` with the descriptor's geom
    shape and friction; dynamic entities also receive a freejoint named
    ``entity:<id>:free``.

    ``force_static`` is a set of entity ids to demote to welded static
    regardless of their ``kind`` — used by the spawn-penetration audit
    when an entity boots in deep contact with the static scene.
    """
    import mujoco

    mesh_assets: dict[Path, str] = {}
    for entity in _initial_entities(entities):
        descriptor = entity.get("descriptor", {})
        entity_id = descriptor.get("entity_id") or entity.get("id")
        pose = entity.get("initial_pose")
        if not entity_id or not pose:
            continue
        entity_id = str(entity_id)

        kind = descriptor.get("kind", "kinematic")
        mass = float(descriptor.get("mass", 0.0))
        dynamic = kind == "dynamic" and mass > 0.0 and entity_id not in force_static

        body = spec.worldbody.add_body(
            name=scene_package_entity_body_name(entity_id),
            pos=[
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("z", 0.0)),
            ],
            quat=[
                float(pose.get("qw", 1.0)),
                float(pose.get("qx", 0.0)),
                float(pose.get("qy", 0.0)),
                float(pose.get("qz", 0.0)),
            ],
        )
        if dynamic:
            body.add_freejoint(name=f"{scene_package_entity_body_name(entity_id)}:free")

        rgba = _entity_rgba(descriptor)
        friction = _entity_friction(entity)
        geom_kwargs: dict[str, Any] = dict(
            name=f"{scene_package_entity_body_name(entity_id)}:geom",
            rgba=list(rgba),
            friction=list(friction),
            group=_ENTITY_GEOM_GROUP,
            # priority=1: contact friction comes from the entity geom alone.
            # MuJoCo's default combine rule (element-wise max across the
            # pair) would otherwise let the μ=1.0 floor override every
            # entity's friction.
            priority=1,
        )
        if dynamic:
            geom_kwargs["mass"] = mass

        shape = descriptor.get("shape_hint", "mesh")
        hull_paths = _entity_collision_hulls(entity) if shape == "mesh" else []
        if shape == "mesh" and not hull_paths and "collision_paths" not in entity:
            logger.warning(
                "entity %s: mesh entity has no cooked collision hulls; using AABB box "
                "(re-cook the scene package with dimos.experimental.scene_cooking.cook)",
                entity_id,
            )
        if hull_paths:
            base_name = scene_package_entity_body_name(entity_id)
            if dynamic:
                # MuJoCo derives per-geom mass from a body-level mass split
                # across geoms by volume. Setting mass on each geom would
                # double-count. Move mass to the body and drop it from the
                # per-geom kwargs.
                body.mass = mass
                geom_kwargs.pop("mass", None)
            for i, hull_obj in enumerate(hull_paths):
                hull_path = hull_obj.resolve()
                mesh_name = mesh_assets.get(hull_path)
                if mesh_name is None:
                    mesh_name = _entity_mesh_asset_name(hull_path)
                    spec.add_mesh(name=mesh_name, file=str(hull_path))
                    mesh_assets[hull_path] = mesh_name
                # Name geoms uniquely so MuJoCo's compile doesn't reject
                # duplicate-name collisions across multi-hull entities.
                gk = dict(geom_kwargs)
                gk["name"] = f"{base_name}:geom{i:03d}"
                body.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_MESH,
                    meshname=mesh_name,
                    **gk,
                )
            continue

        box = _box_size_and_offset(entity)
        if box is None:
            logger.warning("entity %s has no usable collision shape; skipping", entity_id)
            continue
        half, offset = box
        geom_kwargs["pos"] = offset
        if shape == "sphere":
            geom_type = mujoco.mjtGeom.mjGEOM_SPHERE
            size = [half[0], 0.0, 0.0]
        elif shape == "cylinder":
            geom_type = mujoco.mjtGeom.mjGEOM_CYLINDER
            size = [half[0], half[2], 0.0]
        else:
            geom_type = mujoco.mjtGeom.mjGEOM_BOX
            size = half
        body.add_geom(type=geom_type, size=size, **geom_kwargs)


def find_scene_package_entity_spawn_penetrators(model: mujoco.MjModel) -> frozenset[str]:
    """Entity ids whose geoms start in deep contact at the spawn pose.

    Run after ``spec.compile()`` and before stepping; pass the returned
    set as ``force_static`` to a second
    ``add_scene_package_entities_to_spec`` call on a fresh spec if you
    want to weld penetrating entities and recompile.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bad: set[str] = set()
    for c in range(data.ncon):
        contact = data.contact[c]
        if contact.dist >= -_SPAWN_PENETRATION_LIMIT_M:
            continue
        for geom_id in (contact.geom1, contact.geom2):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
            entity_id = _entity_id_from_geom_name(name)
            if entity_id is not None:
                bad.add(entity_id)
    return frozenset(bad)
