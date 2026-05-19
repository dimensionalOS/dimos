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

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import trimesh

from dimos.simulation.mujoco.collision_spec import CollisionSpec
from dimos.simulation.scene_assets.inspect import inspect_scene_asset
from dimos.simulation.scene_assets.mesh_scene import (
    SceneMeshAlignment,
    ScenePrimMesh,
    load_scene_prims,
    split_disconnected_scene_prims,
)
from dimos.simulation.scene_assets.spec import BrowserCollisionSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class BrowserCollisionCookResult:
    path: Path
    stats: dict[str, Any]


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
    """
    browser_spec = spec or BrowserCollisionSpec()
    if not browser_spec.enabled:
        return None

    source = Path(source_path).expanduser().resolve()
    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / browser_spec.output_name
    if out_path.exists() and not rebake:
        return BrowserCollisionCookResult(
            path=out_path,
            stats=inspect_scene_asset(out_path).to_json_dict(),
        )

    mesh = _load_collision_mesh(source, alignment=alignment, collision_spec=collision_spec)
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
    stats = inspect_scene_asset(out_path).to_json_dict()
    stats["source_triangles"] = original_triangles
    stats["target_faces"] = target_faces
    return BrowserCollisionCookResult(path=out_path, stats=stats)


def _write_glb(mesh: o3d.geometry.TriangleMesh, path: Path) -> None:
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.triangles, dtype=np.int64)
    if len(vertices) == 0 or len(faces) == 0:
        raise RuntimeError("browser collision bake produced an empty mesh")
    trimesh.Trimesh(vertices=vertices, faces=faces, process=False).export(str(path))


def _load_collision_mesh(
    source: Path,
    *,
    alignment: SceneMeshAlignment | None,
    collision_spec: CollisionSpec | None,
) -> o3d.geometry.TriangleMesh:
    spec = collision_spec or CollisionSpec.auto_discover(source)
    source_alignment = alignment or SceneMeshAlignment(y_up=False)

    prims = load_scene_prims(source, alignment=source_alignment)
    if spec.split_disconnected_components:
        prims, split_stats = split_disconnected_scene_prims(
            prims,
            min_components=spec.split_min_components,
            extent_ratio=spec.split_extent_ratio,
            prim_min_extent=spec.split_prim_min_extent_m,
            axis_ratio=spec.split_axis_ratio,
            min_component_extent=spec.split_component_min_extent_m,
            min_component_faces=spec.split_component_min_faces,
            can_split=lambda prim: (
                spec.resolve(prim.prim_path or prim.name).get("type", spec.default) == "auto"
            ),
        )
        if split_stats["split_prims"]:
            logger.info(
                "browser collision: split %s disconnected prims into %s kept "
                "components; dropped %s tiny components",
                split_stats["split_prims"],
                split_stats["emitted_components"],
                split_stats["dropped_components"],
            )
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
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


def _mesh_from_arrays(vertices: np.ndarray, faces: np.ndarray) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    return mesh


__all__ = ["BrowserCollisionCookResult", "cook_browser_collision"]
