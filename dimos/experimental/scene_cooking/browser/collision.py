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

"""Bake browser-side collision geometry from a scene asset."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]
import trimesh  # type: ignore[import-untyped]

from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec
from dimos.experimental.scene_cooking.package_config import BrowserCollisionSpec
from dimos.experimental.scene_cooking.source_assets.inspect import inspect_scene_asset
from dimos.experimental.scene_cooking.source_assets.mesh import (
    ScenePrimMesh,
    load_scene_prims,
    split_disconnected_scene_prims,
)
from dimos.simulation.scene_assets.spec import SceneMeshAlignment
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

OBJECTS_SIDECAR_NAME = "objects.json"

#: Bump to force a rebuild of artifacts cooked by earlier manifest schemas.
_MANIFEST_VERSION = 1
_MANIFEST_SUFFIX = ".manifest.json"


@dataclass(frozen=True)
class BrowserCollisionCookResult:
    path: Path
    stats: dict[str, Any]
    objects_path: Path | None = None


def cook_browser_collision(
    source_path: str | Path,
    output_dir: str | Path,
    *,
    alignment: SceneMeshAlignment | None = None,
    spec: BrowserCollisionSpec | None = None,
    collision_spec: CollisionSpec | None = None,
    rebake: bool = False,
) -> BrowserCollisionCookResult | None:
    """Write a simplified GLB used for browser picking/raycast/physics.

    For scene packages this should stay in source-asset coordinates; the
    browser applies the package alignment to the visual and collision roots
    together.

    Idempotent per input set: a manifest of the effective cook inputs is
    written next to the artifacts, and they are reused only while it
    matches. Changing the source, alignment, browser spec, or the resolved
    collision policy (sidecar skips, split settings, ...) rebuilds both
    ``collision.glb`` and ``objects.json`` without needing ``rebake``.
    """
    browser_spec = spec or BrowserCollisionSpec()
    if not browser_spec.enabled:
        return None

    source = Path(source_path).expanduser().resolve()
    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / browser_spec.output_name
    objects_path = out_dir / OBJECTS_SIDECAR_NAME
    manifest_path = out_path.with_suffix(_MANIFEST_SUFFIX)

    effective_collision = collision_spec or CollisionSpec.auto_discover(source)
    effective_alignment = alignment or SceneMeshAlignment(y_up=False)
    manifest = _cook_inputs_json(
        source,
        alignment=effective_alignment,
        browser_spec=browser_spec,
        collision_spec=effective_collision,
    )

    cached = (
        not rebake
        and out_path.exists()
        and objects_path.exists()
        and _manifest_matches(manifest_path, manifest)
    )
    if cached:
        return BrowserCollisionCookResult(
            path=out_path,
            stats=inspect_scene_asset(out_path).to_json_dict(),
            objects_path=objects_path,
        )

    prims = _load_collision_prims(
        source, alignment=effective_alignment, collision_spec=effective_collision
    )
    mesh = _build_fused_collision_mesh(prims, effective_collision)
    original_triangles = len(mesh.triangles)
    target_faces = int(browser_spec.target_faces)
    if target_faces > 0 and original_triangles > target_faces:
        logger.info(
            "browser collision: simplifying %s triangles -> %s",
            original_triangles,
            target_faces,
        )
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_faces)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
    _write_glb(mesh, out_path)
    stats: dict[str, Any] = inspect_scene_asset(out_path).to_json_dict()
    stats["source_triangles"] = original_triangles
    stats["target_faces"] = target_faces

    objects = extract_scene_objects(prims)
    _write_objects_json(objects_path, objects)
    stats["objects"] = len(objects)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return BrowserCollisionCookResult(path=out_path, stats=stats, objects_path=objects_path)


def _cook_inputs_json(
    source: Path,
    *,
    alignment: SceneMeshAlignment,
    browser_spec: BrowserCollisionSpec,
    collision_spec: CollisionSpec,
) -> dict[str, Any]:
    """Effective inputs that shape ``collision.glb`` / ``objects.json``.

    Round-tripped through JSON so comparing against the loaded manifest
    is exact (tuples become lists, keys sorted).
    """
    st = source.stat()
    return json.loads(  # type: ignore[no-any-return]
        json.dumps(
            {
                "manifest_version": _MANIFEST_VERSION,
                "source": {
                    "name": source.name,
                    "size": st.st_size,
                    "mtime_ns": st.st_mtime_ns,
                },
                "alignment": asdict(alignment),
                "browser_spec": asdict(browser_spec),
                "collision_spec": asdict(collision_spec),
            },
            sort_keys=True,
        )
    )


def _manifest_matches(path: Path, expected: dict[str, Any]) -> bool:
    if not path.exists():
        return False
    try:
        return bool(json.loads(path.read_text()) == expected)
    except json.JSONDecodeError:
        return False


def _write_glb(mesh: o3d.geometry.TriangleMesh, path: Path) -> None:
    # Quadric decimation collapses triangles but leaves the original input
    # vertices in the buffer (referenced by nothing). Drop them before export,
    # else the GLB carries millions of orphan vertices (~25x larger on a 100k
    # face supermarket collision: 86MB -> 3.3MB) with no geometry change.
    mesh.remove_unreferenced_vertices()
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.triangles, dtype=np.int64)
    if len(vertices) == 0 or len(faces) == 0:
        raise RuntimeError("browser collision bake produced an empty mesh")
    trimesh.Trimesh(vertices=vertices, faces=faces, process=False).export(str(path))


def _load_collision_prims(
    source: Path,
    *,
    alignment: SceneMeshAlignment | None,
    collision_spec: CollisionSpec | None,
) -> list[ScenePrimMesh]:
    spec = collision_spec or CollisionSpec.auto_discover(source)
    source_alignment = alignment or SceneMeshAlignment(y_up=False)

    prims = load_scene_prims(source, alignment=source_alignment)
    has_forced_splits = any(
        bool(override.get("split_components")) for override in spec.prim_overrides.values()
    )
    if spec.split_disconnected_components or has_forced_splits:

        def _split_override(prim: ScenePrimMesh) -> dict[str, object]:
            return spec.resolve(prim.prim_path or prim.name)

        def _can_split_prim(prim: ScenePrimMesh) -> bool:
            override = _split_override(prim)
            if override.get("split_components"):
                return True
            return (
                spec.split_disconnected_components and override.get("type", spec.default) == "auto"
            )

        def _force_split_prim(prim: ScenePrimMesh) -> bool:
            return bool(_split_override(prim).get("split_components"))

        prims, split_stats = split_disconnected_scene_prims(
            prims,
            min_components=spec.split_min_components,
            extent_ratio=spec.split_extent_ratio,
            prim_min_extent=spec.split_prim_min_extent_m,
            axis_ratio=spec.split_axis_ratio,
            min_component_extent=spec.split_component_min_extent_m,
            min_component_faces=spec.split_component_min_faces,
            can_split=_can_split_prim,
            force_split=_force_split_prim,
        )
        if split_stats["split_prims"]:
            logger.info(
                "browser collision: split %s disconnected prims into %s kept "
                "components; dropped %s tiny components",
                split_stats["split_prims"],
                split_stats["emitted_components"],
                split_stats["dropped_components"],
            )
    return prims


def _build_fused_collision_mesh(
    prims: list[ScenePrimMesh],
    spec: CollisionSpec,
) -> o3d.geometry.TriangleMesh:
    vertices: list[NDArray[np.float64]] = []
    faces: list[NDArray[np.int64]] = []
    vertex_offset = 0
    for prim in prims:
        mesh = _mesh_for_prim(prim, spec)
        if mesh is None:
            continue
        prim_vertices = np.asarray(mesh.vertices, dtype=np.float64)
        prim_faces = np.asarray(mesh.triangles, dtype=np.int64)
        if len(prim_vertices) == 0 or len(prim_faces) == 0:
            continue
        vertices.append(prim_vertices)
        faces.append(prim_faces + vertex_offset)
        vertex_offset += len(prim_vertices)
    if not vertices:
        raise RuntimeError("browser collision sidecar skipped every prim")
    return _mesh_from_arrays(np.concatenate(vertices, axis=0), np.concatenate(faces, axis=0))


def extract_scene_objects(prims: list[ScenePrimMesh]) -> list[dict[str, Any]]:
    """Per-prim semantic metadata (id, prim_path, AABB in source frame).

    Emitted independently of the fused collision GLB so the runtime can
    answer ``findAsset("sectional")``-style queries without paying a
    per-object PhysicsAggregate cost. AABB shares the collision GLB
    frame (source / z-up after alignment).
    """
    objects: list[dict[str, Any]] = []
    for prim in prims:
        v = np.asarray(prim.vertices, dtype=np.float64)
        if v.size == 0:
            continue
        objects.append(
            {
                "id": prim.name,
                "prim_path": prim.prim_path,
                "aabb": {
                    "min": v.min(axis=0).tolist(),
                    "max": v.max(axis=0).tolist(),
                },
            }
        )
    return objects


def _write_objects_json(path: Path, objects: list[dict[str, Any]]) -> None:
    payload = {"frame": "source", "objects": objects}
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _mesh_for_prim(
    prim: ScenePrimMesh,
    spec: CollisionSpec,
) -> o3d.geometry.TriangleMesh | None:
    override = spec.resolve(prim.prim_path or prim.name)
    override_type = override.get("type", spec.default)
    if override_type == "skip":
        return None

    mesh = _mesh_from_arrays(
        prim.vertices.astype(np.float64),
        prim.triangles.astype(np.int64),
    )
    target_faces = int(override.get("target_faces") or 0)
    if target_faces > 0 and len(mesh.triangles) > target_faces:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_faces)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
    return mesh


def _mesh_from_arrays(
    vertices: NDArray[np.float64], faces: NDArray[np.int64]
) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    return mesh
