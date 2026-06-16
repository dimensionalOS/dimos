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

"""Fast scene asset inspection for cook reports and budget checks."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]


@dataclass(frozen=True)
class SceneAssetStats:
    path: str
    bytes: int
    format: str
    mesh_count: int = 0
    node_count: int = 0
    material_count: int = 0
    texture_count: int = 0
    vertex_count: int = 0
    triangle_count: int = 0

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


def inspect_scene_asset(path: str | Path) -> SceneAssetStats:
    """Return lightweight geometry/material counts for a supported scene file."""
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"scene asset not found: {resolved}")

    suffix = resolved.suffix.lower()
    if suffix in {".glb", ".gltf"}:
        return _inspect_gltf(resolved)
    if suffix in {".usd", ".usda", ".usdc", ".usdz"}:
        return _inspect_usd(resolved)
    return _inspect_open3d(resolved)


def _inspect_gltf(path: Path) -> SceneAssetStats:
    import trimesh

    loaded: Any = trimesh.load(str(path))
    if isinstance(loaded, trimesh.Trimesh):
        material_count = 1 if loaded.visual.material is not None else 0
        return SceneAssetStats(
            path=str(path),
            bytes=path.stat().st_size,
            format=path.suffix.lower().lstrip("."),
            mesh_count=1,
            node_count=1,
            material_count=material_count,
            texture_count=_count_material_textures([loaded.visual.material]),
            vertex_count=len(loaded.vertices),
            triangle_count=len(loaded.faces),
        )

    scene = loaded
    mesh_count = len(getattr(scene, "geometry", {}))
    node_count = len(getattr(scene.graph, "nodes_geometry", []))
    materials = []
    vertex_count = 0
    triangle_count = 0
    for geom in scene.geometry.values():
        if not isinstance(geom, trimesh.Trimesh):
            continue
        vertex_count += len(geom.vertices)
        triangle_count += len(geom.faces)
        materials.append(getattr(geom.visual, "material", None))
    material_keys = {repr(material) for material in materials if material is not None}
    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=mesh_count,
        node_count=node_count,
        material_count=len(material_keys),
        texture_count=_count_material_textures(materials),
        vertex_count=vertex_count,
        triangle_count=triangle_count,
    )


def _count_material_textures(materials: list[Any]) -> int:
    textures: set[int] = set()
    for material in materials:
        if material is None:
            continue
        for name in (
            "baseColorTexture",
            "metallicRoughnessTexture",
            "normalTexture",
            "emissiveTexture",
            "occlusionTexture",
            "image",
        ):
            image = getattr(material, name, None)
            if image is not None:
                textures.add(id(image))
    return len(textures)


def _inspect_usd(path: Path) -> SceneAssetStats:
    try:
        from pxr import Usd, UsdGeom, UsdShade  # type: ignore[import-not-found, import-untyped]
    except ImportError as exc:
        raise ImportError("inspecting USD assets requires usd-core") from exc

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open USD stage: {path}")

    mesh_count = 0
    vertex_count = 0
    triangle_count = 0
    materials: set[str] = set()
    textures: set[str] = set()
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
            mesh = UsdGeom.Mesh(prim)
            points_raw = mesh.GetPointsAttr().Get()
            counts_raw = mesh.GetFaceVertexCountsAttr().Get()
            points = points_raw if points_raw is not None else []
            face_counts = np.asarray(counts_raw if counts_raw is not None else [], dtype=np.int32)
            vertex_count += len(points)
            triangle_count += int(np.maximum(face_counts - 2, 0).sum())
            bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bound:
                materials.add(str(bound.GetPath()))
        if prim.IsA(UsdShade.Shader):
            shader = UsdShade.Shader(prim)
            if shader.GetIdAttr().Get() == "UsdUVTexture":
                file_input = shader.GetInput("file")
                if file_input and file_input.Get() is not None:
                    textures.add(str(file_input.Get()))

    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=mesh_count,
        node_count=mesh_count,
        material_count=len(materials),
        texture_count=len(textures),
        vertex_count=vertex_count,
        triangle_count=triangle_count,
    )


def _inspect_open3d(path: Path) -> SceneAssetStats:
    mesh = o3d.io.read_triangle_mesh(str(path))
    if len(mesh.triangles) == 0:
        raise RuntimeError(f"empty mesh: {path}")
    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=1,
        node_count=1,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
    )


__all__ = ["SceneAssetStats", "inspect_scene_asset"]
