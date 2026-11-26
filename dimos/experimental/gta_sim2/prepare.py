# Copyright 2026 Dimensional Inc.
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

"""Prepare a bounded, local-coordinate G1 scene from exported GTA bounds.

Run with --help. Extracted proprietary assets and generated USD stay outside git.
Triangle prisms preserve concavities instead of replacing an entire road with
one convex hull. This is a bounded experiment, not runtime physics streaming.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.unitree.g1.sim2_scene import PELVIS


def glb_triangles(path: Path) -> list[tuple[str, NDArray[np.float64]]]:
    """Read SixNG's world-space collision export, converting glTF back to Z-up."""
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<III", data)
    if (magic, version, length) != (0x46546C67, 2, len(data)):
        raise ValueError(f"Invalid GLB: {path}")
    size, kind = struct.unpack_from("<II", data, 12)
    if kind != 0x4E4F534A:
        raise ValueError("Missing GLB JSON chunk")
    doc = json.loads(data[20 : 20 + size])
    offset = 20 + size
    binary_size, kind = struct.unpack_from("<II", data, offset)
    if kind != 0x004E4942:
        raise ValueError("Missing GLB binary chunk")
    binary = data[offset + 8 : offset + 8 + binary_size]

    def array(index: int) -> NDArray[Any]:
        a = doc["accessors"][index]
        view = doc["bufferViews"][a["bufferView"]]
        dtype = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}[a["componentType"]]
        width = {"SCALAR": 1, "VEC3": 3}[a["type"]]
        stride = view.get("byteStride", np.dtype(dtype).itemsize * width)
        return np.ndarray(
            (a["count"], width),
            dtype=dtype,
            buffer=binary,
            offset=view.get("byteOffset", 0) + a.get("byteOffset", 0),
            strides=(stride, np.dtype(dtype).itemsize),
        )

    for node in doc.get("nodes", []):
        if any(k in node for k in ("matrix", "translation", "rotation", "scale")):
            raise ValueError("Collision export must already have world-space transforms")
    result = []
    for mesh in doc["meshes"]:
        for primitive in mesh["primitives"]:
            if primitive.get("mode", 4) != 4:
                raise ValueError("Only triangulated collision exports are supported")
            vertices = array(primitive["attributes"]["POSITION"]).astype(np.float64)
            vertices = vertices[:, [0, 2, 1]] * [1, -1, 1]
            triangles = vertices[array(primitive["indices"]).reshape(-1, 3)]
            material = doc["materials"][primitive["material"]]["extras"]["surface_id"]
            result.append((material, triangles))
    return result


def clip_triangle(
    triangle: NDArray[np.float64], lo: NDArray[np.float64], hi: NDArray[np.float64]
) -> list[NDArray[np.float64]]:
    """Clip against a finite box without filling holes or bridging islands."""
    polygon = list(triangle)
    for axis in range(3):
        for edge, sign in ((lo[axis], 1), (hi[axis], -1)):
            clipped = []
            if not polygon:
                return []
            previous = polygon[-1]
            d_previous = sign * (previous[axis] - edge)
            for point in polygon:
                distance = sign * (point[axis] - edge)
                if (distance >= 0) != (d_previous >= 0):
                    clipped.append(
                        previous + d_previous / (d_previous - distance) * (point - previous)
                    )
                if distance >= 0:
                    clipped.append(point)
                previous, d_previous = point, distance
            polygon = clipped
    return [
        np.asarray((polygon[0], polygon[i], polygon[i + 1])) for i in range(1, len(polygon) - 1)
    ]


def ground_height(triangles: NDArray[np.float64], xy: tuple[float, float], below: float) -> float:
    """Highest nonvertical surface below a supplied spawn search ceiling."""
    a, b, c = triangles.transpose(1, 0, 2)
    normal = np.cross(b - a, c - a)
    denom = normal[:, 2]
    valid = np.abs(denom) > 1e-8
    u = np.zeros(len(a))
    v = np.zeros(len(a))
    delta = np.asarray(xy) - a[:, :2]
    u[valid] = (delta[:, 0] * (c - a)[:, 1] - delta[:, 1] * (c - a)[:, 0])[valid] / denom[valid]
    v[valid] = ((b - a)[:, 0] * delta[:, 1] - (b - a)[:, 1] * delta[:, 0])[valid] / denom[valid]
    z = a[:, 2] + u * (b - a)[:, 2] + v * (c - a)[:, 2]
    valid &= (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1 + 1e-7) & (z < below)
    if not np.any(valid):
        raise ValueError("No authored ground at spawn; refusing an invented floor")
    return float(z[valid].max())


def prepare(
    bounds: Path,
    reference: Path,
    output: Path,
    center: tuple[float, float],
    half_extent: float,
    z_range: tuple[float, float],
    friction: float,
    spawn_ceiling: float | None = None,
    excluded_sources: frozenset[str] = frozenset(),
) -> dict[str, Any]:
    """Write USD and provenance manifest, retaining G1 prim paths and drives."""
    if half_extent <= 0 or friction <= 0 or z_range[0] >= z_range[1]:
        raise ValueError("Invalid patch dimensions/material")
    reference = reference.resolve()
    output.mkdir(parents=True, exist_ok=True)
    lo = np.array([center[0] - half_extent, center[1] - half_extent, z_range[0]])
    hi = np.array([center[0] + half_extent, center[1] + half_extent, z_range[1]])
    triangles, surfaces, sources = [], [], []
    omitted: Counter[str] = Counter()
    omitted_in_patch: Counter[str] = Counter()
    for line in (bounds / "index.jsonl").open():
        row = json.loads(line)
        if row["id"] in excluded_sources:
            continue
        bbox = row.get("bbox")
        if not bbox or np.any(np.array(bbox[1]) < lo) or np.any(np.array(bbox[0]) > hi):
            continue
        path = bounds / Path(row["file"]).name
        count = 0
        for surface, candidates in glb_triangles(path):
            mask = np.all(candidates.max(axis=1) >= lo, axis=1) & np.all(
                candidates.min(axis=1) <= hi, axis=1
            )
            for tri in candidates[mask]:
                for clipped in clip_triangle(tri, lo, hi):
                    if (
                        np.linalg.norm(np.cross(clipped[1] - clipped[0], clipped[2] - clipped[0]))
                        < 1e-8
                    ):
                        continue
                    triangles.append(clipped)
                    surfaces.append(surface)
                    count += 1
        for primitive in row.get("primitives", []):
            # Explicitly report omissions; analytic primitives need a separate importer.
            omitted[primitive["type"]] += 1
            if primitive["type"] == "capsule":
                ends = np.array([primitive["a"], primitive["b"]])
                radius = primitive["radius"]
                if np.all(ends.max(axis=0) + radius >= lo) and np.all(
                    ends.min(axis=0) - radius <= hi
                ):
                    omitted_in_patch["capsule"] += 1
            else:
                # Unknown bounds must never be reported as safely outside the patch.
                omitted_in_patch[primitive["type"] + "_unverified_bounds"] += 1
        if count:
            sources.append(
                {
                    "id": row["id"],
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "triangles": count,
                }
            )
    xyz = np.asarray(triangles)
    ground = ground_height(xyz, center, z_range[1] if spawn_ceiling is None else spawn_ceiling)
    origin = np.array([*center, ground])
    xyz -= origin
    # Flatten/rescope references before relocation; the robot's source files remain read-only.
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString(resolve_usd_assets(reference.read_text(), str(reference.parent)))
    stage = Usd.Stage.Open(layer)
    for name in ("floor", "desk_mount", "box_mount"):
        stage.RemovePrim("/world/" + name)
    patch = UsdGeom.Xform.Define(stage, "/world/gta_patch")
    patch.GetPrim().CreateAttribute("gta:origin", Sdf.ValueTypeNames.Double3, custom=True).Set(
        Gf.Vec3d(*origin)
    )
    material = UsdPhysics.MaterialAPI.Apply(
        stage.DefinePrim("/world/gta_patch/contact_material", "Material")
    )
    material.CreateStaticFrictionAttr(friction)
    material.CreateDynamicFrictionAttr(friction)
    material.CreateRestitutionAttr(0)
    # All surface IDs retained. Their real friction coefficients are unknown.
    faces = [0, 1, 2, 5, 4, 3, 0, 3, 4, 0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 3, 2, 3, 0]
    for i, (tri, surface) in enumerate(zip(xyz, surfaces, strict=True)):
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        normal /= np.linalg.norm(normal)
        if normal[2] < 0:
            tri = tri[[0, 2, 1]]
            normal = -normal
        prism = np.concatenate((tri, tri - normal * 0.04))
        mesh = UsdGeom.Mesh.Define(stage, f"/world/gta_patch/contact_{i}")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in prism])
        mesh.CreateFaceVertexCountsAttr([3] * 8)
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.GetPrim().CreateAttribute(
            "gta:surface_id", Sdf.ValueTypeNames.String, custom=True
        ).Set(surface)
        mesh.GetPrim().CreateAttribute("mjc:group", Sdf.ValueTypeNames.Int).Set(3)
        mesh.CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            UsdShade.Material(material.GetPrim()), materialPurpose="physics"
        )
    visual = UsdGeom.Mesh.Define(stage, "/world/gta_patch/optical_surface")
    visual.CreatePointsAttr([Gf.Vec3f(*p) for p in xyz.reshape(-1, 3)])
    visual.CreateFaceVertexCountsAttr([3] * len(xyz))
    visual.CreateFaceVertexIndicesAttr(list(range(len(xyz) * 3)))
    visual.CreateSubdivisionSchemeAttr("none")
    visual.CreateDisplayColorAttr([(0.32, 0.35, 0.38)])
    # Reference spawn remains at local (0,0,.793), with the authored road at z=0.
    assert stage.GetPrimAtPath("/" + PELVIS)
    stage.GetRootLayer().Export(str(output / "world.usda"))
    report = {
        "version": 1,
        "origin_rage_m": origin.tolist(),
        "bounds_rage_m": [lo.tolist(), hi.tolist()],
        "contact_prisms": len(xyz),
        "thickness_m": 0.04,
        "authored_friction": friction,
        "excluded_overlapping_sources": sorted(excluded_sources),
        "surface_ids": dict(Counter(surfaces)),
        "sources": sources,
        "omitted_analytic_primitives_in_candidate_files": dict(omitted),
        "omitted_analytic_primitives_overlapping_patch": dict(omitted_in_patch),
        "limitations": [
            "Static world-space exported bounds only; archetype/interior placement coverage unverified",
            "Friction is an explicit assumption, not decoded from the game",
            "USD material values require a compiled-model audit; importer support varies",
            "Optical fallback uses collision surfaces; SixNG supplies detailed map imagery",
            "No runtime topology streaming, doors, movable world props, or water physics",
        ],
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(output / "surfaces.npz", triangles=xyz)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bounds", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--center", type=float, nargs=2, required=True)
    parser.add_argument("--half-extent", type=float, default=10)
    parser.add_argument("--z-range", type=float, nargs=2, default=(9, 16))
    parser.add_argument("--spawn-ceiling", type=float)
    parser.add_argument("--exclude-source", action="append", default=[])
    parser.add_argument("--friction", type=float, default=0.8)
    args = parser.parse_args()
    report = prepare(
        args.bounds,
        args.reference,
        args.output,
        tuple(args.center),
        args.half_extent,
        tuple(args.z_range),
        args.friction,
        args.spawn_ceiling,
        frozenset(args.exclude_source),
    )
    print(json.dumps({k: v for k, v in report.items() if k != "sources"}, indent=2))


if __name__ == "__main__":
    main()
