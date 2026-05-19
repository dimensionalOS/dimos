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

"""Load a 3D scene asset into DimOS world-frame geometry.

Supports:
  * ``.glb`` / ``.gltf`` / ``.obj`` / ``.ply`` / ``.stl``  — via Open3D's
    ``read_triangle_mesh``.
  * ``.usdz`` / ``.usd`` / ``.usdc``  — via ``pxr.Usd`` (install ``usd-core``).

Returned meshes are in DimOS world frame, with optional scale,
Y-up-to-Z-up rotation, Euler rotation, and translation applied.

This loader is intentionally physics/viewer agnostic.  MuJoCo collision
baking, browser collision baking, ray-casting, and asset inspection all
share the same source transform instead of each subsystem guessing its own
coordinate convention.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]


@dataclass
class SceneMeshAlignment:
    """How to transform a raw scene mesh into dimos world frame.

    Apply order: scale → rotation (y_up swap then zyx euler) → translation.
    """

    scale: float = 1.0
    """Multiplicative scale.  Use 0.01 if the source is centimeters."""

    rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Yaw / pitch / roll in degrees, applied after the y_up swap."""

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """World-frame offset applied last."""

    y_up: bool = True
    """Most exporters (Blender, glTF, Apple USDZ) are Y-up.  When ``True``
    rotate the mesh -90 deg about world X to match dimos's Z-up convention."""


def _world_rotation(alignment: SceneMeshAlignment) -> np.ndarray:
    """Compose the y-up swap + ZYX Euler into one 3x3."""
    rad = np.radians(alignment.rotation_zyx_deg)
    cz, sz = np.cos(rad[0]), np.sin(rad[0])
    cy, sy = np.cos(rad[1]), np.sin(rad[1])
    cx, sx = np.cos(rad[2]), np.sin(rad[2])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    rzyx = rz @ ry @ rx
    if alignment.y_up:
        y_to_z = np.array(
            [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
            dtype=np.float64,
        )
        return rzyx @ y_to_z
    return rzyx


def _average_per_face_vertex(
    per_fv: np.ndarray, face_verts: np.ndarray, n_verts: int
) -> np.ndarray:
    """Scatter-average ``(n_face_verts, 3)`` values onto ``(n_verts, 3)`` indices."""
    out = np.zeros((n_verts, 3), dtype=np.float32)
    counts = np.zeros(n_verts, dtype=np.int32)
    np.add.at(out, face_verts, per_fv)
    np.add.at(counts, face_verts, 1)
    counts = np.maximum(counts, 1)[:, None]
    return out / counts


def _color_from_displaycolor(
    mesh: Any,
    n_verts: int,
    face_counts: np.ndarray,
    face_verts: np.ndarray,
) -> np.ndarray | None:
    """Per-vertex RGB from ``primvars:displayColor`` if present and valued.

    Handles the four standard interpolations: ``constant`` / ``vertex`` /
    ``uniform`` / ``faceVarying``.  Returns ``None`` when the primvar
    isn't authored with a value (Sketchfab USDZ exports typically declare
    the primvar but leave it empty — colors live on the bound material).
    """
    from pxr import UsdGeom  # type: ignore[import-not-found, import-untyped]

    pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("displayColor")
    if not pv or not pv.HasValue():
        return None
    raw = pv.Get()
    if raw is None:
        return None
    colors = np.asarray(raw, dtype=np.float32)
    if colors.ndim != 2 or colors.shape[1] != 3 or colors.size == 0:
        return None
    interp = pv.GetInterpolation()

    if interp == UsdGeom.Tokens.constant:
        return np.tile(colors[0:1], (n_verts, 1))

    if interp == UsdGeom.Tokens.vertex and len(colors) == n_verts:
        return colors

    if interp == UsdGeom.Tokens.uniform and len(colors) == len(face_counts):
        per_fv = np.repeat(colors, face_counts, axis=0)
        return _average_per_face_vertex(per_fv, face_verts, n_verts)

    if interp == UsdGeom.Tokens.faceVarying and len(colors) == len(face_verts):
        return _average_per_face_vertex(colors, face_verts, n_verts)

    return None


def _color_from_material(
    prim: Any, material_color_cache: dict[str, np.ndarray | None]
) -> np.ndarray | None:
    """Per-prim RGB from the bound material's ``inputs:diffuseColor``.

    Walks ``UsdShadeMaterialBindingAPI`` → surface shader → ``inputs:diffuseColor``,
    handling ``UsdPreviewSurface`` (the format Sketchfab USDZ uses).  Texture
    inputs aren't sampled — if ``diffuseColor`` is connected to a ``UsdUVTexture``
    rather than authored as a literal, this returns ``None`` and the caller
    falls back to the next strategy.

    Results are cached per material path so we don't re-walk the shader graph
    for every prim that shares a material.
    """
    from pxr import UsdShade  # type: ignore[import-not-found, import-untyped]

    mat_api = UsdShade.MaterialBindingAPI(prim)
    bound = mat_api.ComputeBoundMaterial()[0]
    if not bound:
        return None
    mat_path = str(bound.GetPath())
    if mat_path in material_color_cache:
        return material_color_cache[mat_path]

    color = _resolve_diffuse_color(bound)
    material_color_cache[mat_path] = color
    return color


def _resolve_diffuse_color(material: Any) -> np.ndarray | None:
    """Pull a literal ``diffuseColor`` out of a UsdShade material's surface shader."""
    from pxr import UsdShade  # type: ignore[import-not-found, import-untyped]

    surface = material.ComputeSurfaceSource("")[0]
    if not surface:
        return None
    diffuse_input = surface.GetInput("diffuseColor")
    if not diffuse_input:
        return None
    # If the input is connected (texture-driven), bail — we don't sample images.
    if diffuse_input.HasConnectedSource():
        connected = diffuse_input.GetConnectedSource()[0]
        if connected:
            shader = UsdShade.Shader(connected.GetPrim())
            if shader and shader.GetIdAttr().Get() == "UsdUVTexture":
                return None
    val = diffuse_input.Get()
    if val is None:
        return None
    arr = np.asarray(val, dtype=np.float32).reshape(-1)
    if arr.size != 3:
        return None
    return arr  # (3,) RGB in [0, 1]


def _load_usd_mesh(path: Path) -> o3d.geometry.TriangleMesh:
    """Walk every Mesh prim in a USD stage and concatenate to one o3d mesh.

    Also extracts per-vertex colors from ``primvars:displayColor`` when
    present so downstream consumers can render textured-looking Sketchfab
    exports without having to chase materials/textures.
    """
    try:
        from pxr import Usd, UsdGeom  # type: ignore[import-not-found, import-untyped]
    except ImportError as e:
        raise ImportError("loading .usdz/.usd requires usd-core: `uv pip install usd-core`") from e

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open USD stage: {path}")

    all_pts: list[np.ndarray] = []
    all_tris: list[np.ndarray] = []
    all_colors: list[np.ndarray] = []
    any_color = False
    vtx_offset = 0
    material_color_cache: dict[str, np.ndarray | None] = {}

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts_attr = mesh.GetPointsAttr().Get()
        if pts_attr is None or len(pts_attr) == 0:
            continue
        pts = np.asarray(pts_attr, dtype=np.float32)
        face_verts = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
        face_counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)

        # Bake the prim's local-to-world transform into the points so the
        # composite scene comes out in stage-root coordinates.
        xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        m = np.asarray(xform, dtype=np.float64).T  # USD matrices are row-major
        pts_h = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])
        pts_world = (m @ pts_h.T).T[:, :3].astype(np.float32)

        # Per-prim color resolution.  Try in order:
        #   1. ``primvars:displayColor`` (vertex / faceVarying / uniform / constant)
        #   2. Bound material's ``inputs:diffuseColor`` (UsdPreviewSurface — what
        #      Sketchfab USDZ uses, with one constant color per material).
        #   3. Neutral grey fallback.
        prim_colors = _color_from_displaycolor(mesh, len(pts), face_counts, face_verts)
        if prim_colors is None:
            mat_color = _color_from_material(prim, material_color_cache)
            if mat_color is not None:
                prim_colors = np.tile(mat_color[None, :], (len(pts), 1))
        if prim_colors is not None:
            any_color = True
        else:
            prim_colors = np.full((len(pts), 3), 0.7, dtype=np.float32)

        # USD allows quads / n-gons; fan-triangulate so o3d gets pure tris.
        tris: list[tuple[int, int, int]] = []
        cursor = 0
        for n in face_counts:
            for k in range(1, n - 1):
                tris.append(
                    (
                        int(face_verts[cursor]) + vtx_offset,
                        int(face_verts[cursor + k]) + vtx_offset,
                        int(face_verts[cursor + k + 1]) + vtx_offset,
                    )
                )
            cursor += n

        if not tris:
            continue
        all_pts.append(pts_world)
        all_tris.append(np.asarray(tris, dtype=np.int32))
        all_colors.append(prim_colors)
        vtx_offset += len(pts_world)

    if not all_pts:
        raise RuntimeError(f"no Mesh prims with triangles found in {path}")

    pts = np.concatenate(all_pts, axis=0).astype(np.float64)
    tris = np.concatenate(all_tris, axis=0)

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(pts)
    mesh.triangles = o3d.utility.Vector3iVector(tris)
    if any_color:
        colors = np.concatenate(all_colors, axis=0).astype(np.float64)
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
    return mesh


def load_scene_mesh(
    path: str | Path,
    alignment: SceneMeshAlignment | None = None,
) -> o3d.geometry.TriangleMesh:
    """Load a scene mesh from disk and apply alignment to put it in dimos world frame.

    Args:
        path: file path.  Supported extensions: ``.usdz``, ``.usd``, ``.usdc``,
            ``.glb``, ``.gltf``, ``.obj``, ``.ply``, ``.stl``.
        alignment: scale / rotation / translation to apply.

    Returns:
        an ``open3d.geometry.TriangleMesh`` in dimos world frame with vertex
        normals computed.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"scene mesh not found: {path}")
    suffix = path.suffix.lower()
    if suffix in {".usdz", ".usd", ".usdc", ".usda"}:
        mesh = _load_usd_mesh(path)
    elif suffix in {".glb", ".gltf"}:
        # GEOMETRY-ONLY GLB load. Used by floor-z probing and ray-casting;
        # it does not need PBR materials. ``trimesh.load(path, force="mesh")``
        # would flatten the scene by decompressing every embedded texture and
        # sampling per-vertex colors. For a scene with hundreds of 4K PBR
        # textures, that allocates ~10 GB transiently and OOMs 32 GB boxes.
        # We open in Scene mode (no flattening, no texture decode), walk the
        # instance graph applying each instance's world transform, and emit a
        # single concatenated mesh — peak stays under ~1 GB.
        import trimesh

        scene_or_mesh: Any = trimesh.load(str(path))
        if isinstance(scene_or_mesh, trimesh.Trimesh):
            verts_world = np.asarray(scene_or_mesh.vertices, dtype=np.float64)
            faces_world = np.asarray(scene_or_mesh.faces, dtype=np.int64)
        else:
            scene = scene_or_mesh
            verts_chunks: list[np.ndarray] = []
            faces_chunks: list[np.ndarray] = []
            v_off = 0
            for node_name in scene.graph.nodes_geometry:
                xform, geom_name = scene.graph[node_name]
                geom = scene.geometry.get(geom_name)
                if geom is None or not isinstance(geom, trimesh.Trimesh) or len(geom.faces) == 0:
                    continue
                v_local = np.asarray(geom.vertices, dtype=np.float64)
                f_local = np.asarray(geom.faces, dtype=np.int64)
                m = np.asarray(xform, dtype=np.float64)
                v_h = np.hstack([v_local, np.ones((len(v_local), 1), dtype=np.float64)])
                v_world = (m @ v_h.T).T[:, :3]
                verts_chunks.append(v_world)
                faces_chunks.append(f_local + v_off)
                v_off += len(v_local)
            if not verts_chunks:
                raise RuntimeError(f"glTF loaded but no Trimesh instances found: {path}")
            verts_world = np.concatenate(verts_chunks, axis=0)
            faces_world = np.concatenate(faces_chunks, axis=0)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts_world)
        mesh.triangles = o3d.utility.Vector3iVector(faces_world.astype(np.int32))
    else:
        mesh = o3d.io.read_triangle_mesh(str(path))
        if len(mesh.triangles) == 0:
            raise RuntimeError(f"o3d.io.read_triangle_mesh returned an empty mesh for {path}")

    align = alignment or SceneMeshAlignment()
    if align.scale != 1.0:
        mesh.scale(align.scale, center=np.zeros(3))
    rot = _world_rotation(align)
    if not np.allclose(rot, np.eye(3)):
        mesh.rotate(rot, center=np.zeros(3))
    if any(align.translation):
        mesh.translate(np.asarray(align.translation, dtype=np.float64))

    mesh.compute_vertex_normals()
    return mesh


def floor_z_under_origin(
    scene_mesh_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
) -> float:
    """Return the first scene surface under world ``x=0, y=0``.

    Falls back to the mesh bbox minimum when the origin ray misses.
    """
    import open3d.core as o3c

    mesh = load_scene_mesh(scene_mesh_path, alignment=alignment)
    scene = make_raycasting_scene(mesh)
    rays = o3c.Tensor(
        np.array([[0.0, 0.0, 1000.0, 0.0, 0.0, -1.0]], dtype=np.float32),
        dtype=o3c.Dtype.Float32,
    )
    t_hit = float(scene.cast_rays(rays)["t_hit"].numpy()[0])
    if np.isfinite(t_hit):
        return 1000.0 - t_hit
    bbox = mesh.get_axis_aligned_bounding_box()
    return float(bbox.min_bound[2])


def make_raycasting_scene(
    mesh: o3d.geometry.TriangleMesh,
) -> o3d.t.geometry.RaycastingScene:
    """Wrap a TriangleMesh into Open3D's BVH-backed ray-casting scene."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


@dataclass
class ScenePrimMesh:
    """One USD ``Mesh`` prim's geometry, ready to write to OBJ.

    Used by ``load_scene_prims`` to keep prims separate so MuJoCo can
    treat each as its own (approximately convex) collision shape.  When
    the loader handles a non-USD format the input is returned as a
    single-element list with the whole mesh in it.
    """

    name: str
    """Sanitized identifier (safe for MJCF asset names) — typically the
    USD prim path with non-alphanumerics replaced."""

    vertices: np.ndarray
    """``(N, 3)`` float32, in world frame after alignment."""

    triangles: np.ndarray
    """``(M, 3)`` int32 vertex indices."""

    prim_path: str | None = None
    """Original scene-graph path when the source format provides one."""


def split_disconnected_scene_prims(
    prims: list[ScenePrimMesh],
    *,
    min_components: int,
    extent_ratio: float,
    prim_min_extent: float,
    axis_ratio: float,
    min_component_extent: float,
    min_component_faces: int,
    can_split: Callable[[ScenePrimMesh], bool] | None = None,
) -> tuple[list[ScenePrimMesh], dict[str, int]]:
    """Split scene-graph nodes that are disconnected prop clusters.

    Some game exports group many small disconnected objects under one node
    (for example leaves, cups, bottles).  Primitive fitting sees only the
    combined bounds and can turn the group into one scene-scale slab.  This
    helper keeps normal connected props intact, but splits suspicious wide
    clusters so tiny decorative islands can be dropped and larger islands can
    be fitted independently.
    """
    import trimesh

    result: list[ScenePrimMesh] = []
    stats = {
        "source_prims": len(prims),
        "split_prims": 0,
        "emitted_components": 0,
        "dropped_components": 0,
    }

    for prim in prims:
        if can_split is not None and not can_split(prim):
            result.append(prim)
            continue
        if len(prim.triangles) < max(min_component_faces * 2, 1):
            result.append(prim)
            continue
        prim_extent = np.ptp(prim.vertices, axis=0)
        if float(prim_extent.max()) < prim_min_extent:
            result.append(prim)
            continue
        positive_extent = prim_extent[prim_extent > 1e-6]
        if (
            len(positive_extent) < 3
            or float(positive_extent.max() / positive_extent.min()) < axis_ratio
        ):
            result.append(prim)
            continue

        mesh = trimesh.Trimesh(vertices=prim.vertices, faces=prim.triangles, process=False)
        parts = mesh.split(only_watertight=False)
        if len(parts) < min_components:
            result.append(prim)
            continue

        component_extents = np.array(
            [np.ptp(np.asarray(part.vertices), axis=0).max() for part in parts],
            dtype=np.float64,
        )
        median_component_extent = float(np.median(component_extents))
        if median_component_extent <= 0.0:
            result.append(prim)
            continue
        if float(prim_extent.max()) / median_component_extent < extent_ratio:
            result.append(prim)
            continue

        emitted = 0
        dropped = 0
        for index, part in enumerate(parts):
            vertices = np.asarray(part.vertices, dtype=np.float32)
            triangles = np.asarray(part.faces, dtype=np.int32)
            component_extent = float(np.ptp(vertices, axis=0).max()) if len(vertices) else 0.0
            if len(triangles) < min_component_faces or component_extent < min_component_extent:
                dropped += 1
                continue
            result.append(
                ScenePrimMesh(
                    name=f"{prim.name}_part{index:04d}",
                    vertices=vertices,
                    triangles=triangles,
                    prim_path=(
                        f"{prim.prim_path}/component_{index:04d}"
                        if prim.prim_path is not None
                        else f"{prim.name}/component_{index:04d}"
                    ),
                )
            )
            emitted += 1

        stats["split_prims"] += 1
        stats["emitted_components"] += emitted
        stats["dropped_components"] += dropped
        if emitted == 0:
            continue

    return result, stats


def _load_glb_prims(path: Path, alignment: SceneMeshAlignment) -> list[ScenePrimMesh]:
    """Enumerate per-instance prims from a glTF/GLB.

    ``trimesh.load(file.glb)`` returns a ``Scene`` whose ``graph`` records
    the world transform for every geometry instance.  Iterating
    ``graph.nodes_geometry`` is the trimesh equivalent of USD's
    ``stage.Traverse()`` — it yields one entry per instance, even when
    multiple instances share the same underlying mesh (typical for chairs,
    cabinets, etc.).  Without this enumeration, ``trimesh.load(... force="mesh")``
    collapses the whole scene to one mesh and CoACD produces a single coarse
    decomposition, which is essentially useless for collision against
    multi-object scenes.
    """
    import trimesh

    loaded: Any = trimesh.load(str(path))
    R = _world_rotation(alignment)
    T = np.asarray(alignment.translation, dtype=np.float64)
    s = float(alignment.scale)

    if isinstance(loaded, trimesh.Trimesh):
        # Single-mesh GLB (no scene graph).  Treat as one prim.
        pts = np.asarray(loaded.vertices, dtype=np.float64)
        faces = np.asarray(loaded.faces, dtype=np.int32)
        if len(faces) == 0:
            return []
        pts_world = (R @ (s * pts).T).T + T
        return [
            ScenePrimMesh(
                name="scene",
                vertices=pts_world.astype(np.float32),
                triangles=faces,
                prim_path="scene",
            )
        ]

    scene = loaded
    prims: list[ScenePrimMesh] = []
    for node_name in scene.graph.nodes_geometry:
        xform, geom_name = scene.graph[node_name]
        geom = scene.geometry.get(geom_name)
        if geom is None or not isinstance(geom, trimesh.Trimesh):
            continue
        if len(geom.faces) == 0:
            continue

        pts_local = np.asarray(geom.vertices, dtype=np.float64)
        faces = np.asarray(geom.faces, dtype=np.int32)

        # Local → scene-root via the instance transform.
        m = np.asarray(xform, dtype=np.float64)
        pts_h = np.hstack([pts_local, np.ones((len(pts_local), 1), dtype=np.float64)])
        pts_stage = (m @ pts_h.T).T[:, :3]

        # Scene-root → dimos world via SceneMeshAlignment.
        pts_world = (R @ (s * pts_stage).T).T + T

        clean = "".join(c if c.isalnum() else "_" for c in str(node_name))
        prims.append(
            ScenePrimMesh(
                name=f"{clean}__{len(prims)}",
                vertices=pts_world.astype(np.float32),
                triangles=faces,
                prim_path=str(node_name),
            )
        )
    return prims


def load_scene_prims(
    path: str | Path,
    alignment: SceneMeshAlignment | None = None,
) -> list[ScenePrimMesh]:
    """Load a USD/USDZ scene as one ``ScenePrimMesh`` per Mesh prim.

    Per-prim splitting is what MuJoCo wants for non-trivial scenes:
    each prim's convex hull approximates the prim well, while the
    convex hull of the *whole* scene is its bounding box.  Falls back
    to a single ScenePrimMesh for non-USD inputs (a single ``.obj`` or
    ``.glb`` doesn't carry per-part semantics in our loader).

    Same alignment rules as ``load_scene_mesh``.
    """
    path = Path(path)
    align = alignment or SceneMeshAlignment()
    suffix = path.suffix.lower()

    if suffix in {".glb", ".gltf"}:
        return _load_glb_prims(path, align)

    if suffix not in {".usdz", ".usd", ".usdc", ".usda"}:
        # Non-USD, non-glTF (e.g. .obj/.ply/.stl): one part, whole mesh.
        whole = load_scene_mesh(path, alignment=align)
        return [
            ScenePrimMesh(
                name="scene",
                vertices=np.asarray(whole.vertices, dtype=np.float32),
                triangles=np.asarray(whole.triangles, dtype=np.int32),
                prim_path="scene",
            )
        ]

    try:
        from pxr import Usd, UsdGeom  # type: ignore[import-not-found, import-untyped]
    except ImportError as e:
        raise ImportError("loading .usdz/.usd requires usd-core: `uv pip install usd-core`") from e

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open USD stage: {path}")

    R = _world_rotation(align)
    T = np.asarray(align.translation, dtype=np.float64)
    s = float(align.scale)

    prims: list[ScenePrimMesh] = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        usd_mesh = UsdGeom.Mesh(prim)
        pts_attr = usd_mesh.GetPointsAttr().Get()
        if pts_attr is None or len(pts_attr) == 0:
            continue
        pts = np.asarray(pts_attr, dtype=np.float64)
        face_verts = np.asarray(usd_mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
        face_counts = np.asarray(usd_mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)

        # Local → stage-root via the USD prim's accumulated transform.
        xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        m = np.asarray(xform, dtype=np.float64).T
        pts_h = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)])
        pts_stage = (m @ pts_h.T).T[:, :3]

        # Stage-root → dimos world via SceneMeshAlignment (scale → rot → trans).
        pts_world = (R @ (s * pts_stage).T).T + T

        # Triangulate any quads / n-gons (vertex indices are local to this prim now).
        tris: list[tuple[int, int, int]] = []
        cursor = 0
        for n in face_counts:
            for k in range(1, n - 1):
                tris.append(
                    (
                        int(face_verts[cursor]),
                        int(face_verts[cursor + k]),
                        int(face_verts[cursor + k + 1]),
                    )
                )
            cursor += n
        if not tris:
            continue

        # MJCF asset names: strip the leading slash, swap remaining
        # path separators / dots for underscores.  USD prim paths can
        # collide on the same leaf; suffix the index so each is unique.
        raw = str(prim.GetPath()).lstrip("/")
        clean = "".join(c if c.isalnum() else "_" for c in raw)
        prim_path = str(prim.GetPath())
        prims.append(
            ScenePrimMesh(
                name=f"{clean}__{len(prims)}",
                vertices=pts_world.astype(np.float32),
                triangles=np.asarray(tris, dtype=np.int32),
                prim_path=prim_path,
            )
        )

    if not prims:
        raise RuntimeError(f"no Mesh prims with triangles found in {path}")
    return prims


__all__ = [
    "SceneMeshAlignment",
    "ScenePrimMesh",
    "floor_z_under_origin",
    "load_scene_mesh",
    "load_scene_prims",
    "make_raycasting_scene",
    "split_disconnected_scene_prims",
]
