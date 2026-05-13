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

"""Load a 3D scene mesh from disk for ray-casting + visualization.

Supports:
  * ``.glb`` / ``.gltf`` / ``.obj`` / ``.ply`` / ``.stl``  — via Open3D's
    ``read_triangle_mesh``.
  * ``.usdz`` / ``.usd`` / ``.usdc``  — via ``pxr.Usd`` (install ``usd-core``).

Returned form is a single concatenated ``open3d.geometry.TriangleMesh``
in world frame, with optional scale + Y-up→Z-up + translation applied.

The same mesh feeds:
  1. ``MeshCameraModule`` — ``Open3D RaycastingScene`` per-pixel ray-cast
     to publish a real RGB head-camera feed on ``/splat/color_image``.
  2. ``ViserRenderModule`` — drawn in the browser viewer as a colored
     ``add_mesh_trimesh`` so the user can navigate the loaded scene.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d


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
    from pxr import UsdGeom

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
    from pxr import UsdShade

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
    from pxr import UsdShade

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
    present so downstream consumers (viser) can render textured-looking
    Sketchfab exports without having to chase materials/textures.
    """
    try:
        from pxr import Usd, UsdGeom
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

    if suffix not in {".usdz", ".usd", ".usdc", ".usda"}:
        # Non-USD: one part, whole mesh.
        whole = load_scene_mesh(path, alignment=align)
        return [
            ScenePrimMesh(
                name="scene",
                vertices=np.asarray(whole.vertices, dtype=np.float32),
                triangles=np.asarray(whole.triangles, dtype=np.int32),
            )
        ]

    try:
        from pxr import Usd, UsdGeom
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
        prims.append(
            ScenePrimMesh(
                name=f"{clean}__{len(prims)}",
                vertices=pts_world.astype(np.float32),
                triangles=np.asarray(tris, dtype=np.int32),
            )
        )

    if not prims:
        raise RuntimeError(f"no Mesh prims with triangles found in {path}")
    return prims


def floor_z_under_origin(
    scene_mesh_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
) -> float:
    """Return the z of the first surface ``(x=0, y=0)`` falls onto.

    Casts one ray straight down from a high z; the first hit defines
    the local floor at origin.  Falls back to the mesh's bbox min-z
    when origin is over a hole (or outside the bbox xy footprint).
    Used by the GR00T sim blueprint to align the loaded scene with
    the robot's default spawn pose so all three downstream views
    (viser, mesh camera, MuJoCo physics) share one world frame.
    """
    import open3d.core as o3c

    mesh = load_scene_mesh(scene_mesh_path, alignment=alignment)
    scene = make_raycasting_scene(mesh)
    rays = o3c.Tensor(
        np.array([[0.0, 0.0, 1000.0, 0.0, 0.0, -1.0]], dtype=np.float32),
        dtype=o3c.Dtype.Float32,
    )
    t = float(scene.cast_rays(rays)["t_hit"].numpy()[0])
    if np.isfinite(t):
        return 1000.0 - t
    bb = mesh.get_axis_aligned_bounding_box()
    return float(bb.min_bound[2])


__all__ = [
    "SceneMeshAlignment",
    "load_scene_mesh",
    "make_raycasting_scene",
]
