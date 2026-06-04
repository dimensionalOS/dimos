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

"""Compose scene-package entities into the cooked MuJoCo model.

The cook pipeline removes entity prims (chairs, props) from the static
scene bake (``remove_from_static``) so they can be simulated as dynamic
objects — but until now they only existed in browser Havok. This module
closes that gap for the MuJoCo backend: it wraps the cooked
``wrapper.xml`` in a thin parent MJCF that adds one body per packaged
entity, compiles it, and caches the result next to the wrapper.

Entities with ``kind == "dynamic"`` and positive mass get a freejoint
(robot can push/grasp them); anything else is welded static. Collision:
primitives use the descriptor ``extents``; mesh entities use the convex
hull of their cooked ``visual.glb`` (MuJoCo collides on the hull of
mesh geoms anyway, so this is exact for its collision model). Exact
concave collision (convex decomposition at cook time) is a follow-up.

After the first compile the spawn state is contact-audited: entities
that start in deep penetration with the static scene (e.g. a chair
whose hull still clips a desk) are demoted to static and the model is
recompiled once — a welded overlap is harmless, a free body ejecting
itself across the office is not.

Body naming: ``entity:<entity_id>`` — consumers map MuJoCo bodies back
to entity ids through this prefix.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any
from xml.sax.saxutils import quoteattr

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.simulation.scene_assets.spec import ScenePackage

logger = setup_logger()

ENTITY_BODY_PREFIX = "entity:"

_MIN_HALF_EXTENT = 0.01
# Sliding friction 0.3 ≈ furniture that scoots when bumped. Entity geoms
# carry priority=1 so this wins the contact pair outright (MuJoCo's
# default combine rule is element-wise max, which would let the μ=1.0
# floor override it). Graspable props override via ``physics.friction``.
_DEFAULT_FRICTION = (0.3, 0.05, 0.001)
_DEFAULT_RGBA = "0.62 0.62 0.68 1.0"
# Same geom group as the baked static scene so depth-based lidar renders
# (which hide robot groups 0/1) still see entities.
_ENTITY_GEOM_GROUP = "3"
# Spawn-contact audit: deeper penetration than this at t=0 demotes the
# entity to static instead of letting MuJoCo eject it.
_SPAWN_PENETRATION_LIMIT_M = 0.02


def entity_body_name(entity_id: str) -> str:
    return f"{ENTITY_BODY_PREFIX}{entity_id}"


def _initial_entities(scene_package: ScenePackage) -> list[dict[str, Any]]:
    return [e for e in scene_package.entities if e.get("spawn", "initial") == "initial"]


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
    half = [max((h - l) / 2.0, _MIN_HALF_EXTENT) for l, h in zip(lo, hi, strict=True)]
    center = [(h + l) / 2.0 for l, h in zip(lo, hi, strict=True)]
    origin = [float(pose.get(k, 0.0)) for k in ("x", "y", "z")]
    offset = [c - o for c, o in zip(center, origin, strict=True)]
    return half, offset


def _hull_obj_path(entity: dict[str, Any], out_dir: Path) -> Path | None:
    """Convex hull of the entity's cooked GLB as a cached OBJ for MuJoCo.

    The cooked per-entity GLBs are entity-local (origin = initial_pose,
    world axes), so the hull drops in with zero geom offset.
    """
    visual_path = entity.get("visual_path")
    if not isinstance(visual_path, str):
        return None
    glb = Path(visual_path)
    if not glb.exists():
        return None

    entity_id = str(entity.get("id", "unknown"))
    stat = glb.stat()
    key = hashlib.sha256(f"{glb}:{stat.st_size}:{stat.st_mtime_ns}".encode()).hexdigest()[:10]
    hull_dir = out_dir / "entity_hulls"
    hull_dir.mkdir(exist_ok=True)
    obj_path = hull_dir / f"{entity_id}_{key}.obj"
    if obj_path.exists():
        return obj_path

    try:
        import open3d as o3d  # type: ignore[import-untyped]

        mesh = o3d.io.read_triangle_mesh(str(glb))
        if not mesh.has_vertices():
            return None
        hull, _ = mesh.compute_convex_hull()
        o3d.io.write_triangle_mesh(str(obj_path), hull, write_vertex_normals=False)
    except Exception as exc:
        logger.warning("entity %s: hull from %s failed (%s); using AABB box", entity_id, glb, exc)
        return None
    return obj_path


def _entity_friction(entity: dict[str, Any]) -> str:
    """``physics.friction`` from entity metadata (scalar sliding or full
    [sliding, torsional, rolling] triple), else the scoot-able default."""
    raw = entity.get("physics", {}).get("friction")
    sliding, torsional, rolling = _DEFAULT_FRICTION
    if isinstance(raw, int | float):
        sliding = float(raw)
    elif isinstance(raw, list | tuple) and len(raw) == 3:
        sliding, torsional, rolling = (float(v) for v in raw)
    return f"{sliding} {torsional} {rolling}"


def _entity_body_xml(
    entity: dict[str, Any],
    out_dir: Path,
    force_static: frozenset[str],
) -> tuple[str, str | None] | None:
    """(body xml, asset xml or None) for one packaged entity."""
    descriptor = entity.get("descriptor", {})
    entity_id = descriptor.get("entity_id") or entity.get("id")
    pose = entity.get("initial_pose")
    if not entity_id or not pose:
        return None
    entity_id = str(entity_id)

    kind = descriptor.get("kind", "kinematic")
    mass = float(descriptor.get("mass", 0.0))
    dynamic = kind == "dynamic" and mass > 0.0 and entity_id not in force_static

    geom_name = quoteattr(entity_body_name(entity_id) + ":geom")
    geom_mass = f' mass="{mass}"' if dynamic else ""
    friction = _entity_friction(entity)
    # priority=1: contact friction comes from the entity geom alone.
    # Without it MuJoCo takes the element-wise max across the pair and the
    # μ=1.0 floor would override every entity's friction.
    common = (
        f'rgba="{_DEFAULT_RGBA}" friction="{friction}" priority="1" group="{_ENTITY_GEOM_GROUP}"'
    )

    asset_xml: str | None = None
    shape = descriptor.get("shape_hint", "mesh")
    hull_obj = _hull_obj_path(entity, out_dir) if shape == "mesh" else None
    if hull_obj is not None:
        mesh_name = quoteattr(entity_body_name(entity_id) + ":hull")
        asset_xml = f"    <mesh name={mesh_name} file={quoteattr(str(hull_obj))}/>"
        geom_xml = (
            f'      <geom name={geom_name} type="mesh" mesh={mesh_name}{geom_mass} {common}/>'
        )
    else:
        box = _box_size_and_offset(entity)
        if box is None:
            logger.warning("entity %s has no usable collision shape; skipping", entity_id)
            return None
        half, offset = box
        size = " ".join(f"{v:.6f}" for v in half)
        geom_pos = " ".join(f"{v:.6f}" for v in offset)
        geom_xml = (
            f'      <geom name={geom_name} type="box" size="{size}" '
            f'pos="{geom_pos}"{geom_mass} {common}/>'
        )

    name = quoteattr(entity_body_name(entity_id))
    pos = f"{pose.get('x', 0.0)} {pose.get('y', 0.0)} {pose.get('z', 0.0)}"
    quat = (
        f"{pose.get('qw', 1.0)} {pose.get('qx', 0.0)} {pose.get('qy', 0.0)} {pose.get('qz', 0.0)}"
    )

    lines = [f'    <body name={name} pos="{pos}" quat="{quat}">']
    if dynamic:
        lines.append(f"      <freejoint name={quoteattr(entity_body_name(entity_id) + ':free')}/>")
    lines.append(geom_xml)
    lines.append("    </body>")
    return "\n".join(lines), asset_xml


def _file_signature(path: Path) -> dict[str, Any]:
    try:
        stat = path.stat()
    except OSError:
        return {"path": str(path), "missing": True}
    return {"path": str(path), "size": stat.st_size, "mtime": stat.st_mtime_ns}


def _include_signatures(wrapper: Path) -> list[dict[str, Any]]:
    """Signatures of files the wrapper <include>s (one level — the robot
    MJCF). Include content is resolved at compile time, so it must be part
    of the cache key or a robot-file edit silently serves a stale mjb."""
    import re

    signatures: list[dict[str, Any]] = []
    try:
        text = wrapper.read_text()
    except OSError:
        return signatures
    for match in re.finditer(r'<include\s+file="([^"]+)"', text):
        include_path = Path(match.group(1))
        if not include_path.is_absolute():
            include_path = wrapper.parent / include_path
        signatures.append(_file_signature(include_path))
    return signatures


def _cache_key(wrapper: Path, entities: list[dict[str, Any]]) -> str:
    payload = json.dumps(
        {
            "wrapper": _file_signature(wrapper),
            "includes": _include_signatures(wrapper),
            "entities": entities,
            # Bump when the generated XML changes shape (collision repr,
            # audit policy, …) so stale cached mjbs don't survive.
            "schema": 4,
        },
        sort_keys=True,
    ).encode()
    return hashlib.sha256(payload).hexdigest()[:12]


def _write_and_compile(
    entities: list[dict[str, Any]],
    wrapper: Path,
    xml_path: Path,
    force_static: frozenset[str],
) -> Any:
    import mujoco

    bodies: list[str] = []
    assets: list[str] = []
    for entity in entities:
        result = _entity_body_xml(entity, wrapper.parent, force_static)
        if result is None:
            continue
        body_xml, asset_xml = result
        bodies.append(body_xml)
        if asset_xml is not None:
            assets.append(asset_xml)
    if not bodies:
        return None

    asset_block = f"  <asset>\n{chr(10).join(assets)}\n  </asset>\n" if assets else ""
    xml = (
        f'<mujoco model="scene_with_entities">\n'
        f"  <include file={quoteattr(str(wrapper))}/>\n"
        f"{asset_block}"
        f"  <worldbody>\n{chr(10).join(bodies)}\n  </worldbody>\n"
        f"</mujoco>\n"
    )
    xml_path.write_text(xml)
    return mujoco.MjModel.from_xml_path(str(xml_path))


def _spawn_penetrators(model: Any) -> frozenset[str]:
    """Entity ids whose geoms start in deep contact at the spawn pose."""
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bad: set[str] = set()
    for c in range(data.ncon):
        contact = data.contact[c]
        if contact.dist >= -_SPAWN_PENETRATION_LIMIT_M:
            continue
        for geom_id in (contact.geom1, contact.geom2):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith(ENTITY_BODY_PREFIX) and name.endswith(":geom"):
                bad.add(name[len(ENTITY_BODY_PREFIX) : -len(":geom")])
    return frozenset(bad)


def compose_entity_model(scene_package: ScenePackage) -> Path | None:
    """MuJoCo model path for the scene including its packaged entities.

    Returns the plain cooked model when the package has no entities (or
    no wrapper to extend), and ``None`` when the package has no MuJoCo
    artifacts at all. Compiled output is cached next to the wrapper,
    keyed on wrapper signature + entity metadata.
    """
    entities = _initial_entities(scene_package)
    wrapper = scene_package.mujoco_wrapper_path
    if not entities or wrapper is None or not Path(wrapper).exists():
        return scene_package.mujoco_model_path

    import mujoco

    wrapper = Path(wrapper)
    key = _cache_key(wrapper, entities)
    out_dir = wrapper.parent
    xml_path = out_dir / f"entities_{key}.xml"
    mjb_path = out_dir / f"entities_{key}.mjb"
    if mjb_path.exists():
        logger.info("entity scene cache hit: %s", mjb_path)
        return mjb_path

    logger.info(
        "compiling scene + %d entities -> %s (first run only)", len(entities), mjb_path.name
    )
    model = _write_and_compile(entities, wrapper, xml_path, frozenset())
    if model is None:
        return scene_package.mujoco_model_path

    # Spawn-contact audit: weld anything that starts inside the static
    # scene rather than letting the first physics step eject it.
    penetrators = _spawn_penetrators(model)
    if penetrators:
        logger.warning(
            "%d entities spawn in deep contact and are welded static: %s",
            len(penetrators),
            ", ".join(sorted(penetrators)),
        )
        model = _write_and_compile(entities, wrapper, xml_path, penetrators)
        if model is None:
            return scene_package.mujoco_model_path

    mujoco.mj_saveModel(model, str(mjb_path))
    return mjb_path


__all__ = ["ENTITY_BODY_PREFIX", "compose_entity_model", "entity_body_name"]
