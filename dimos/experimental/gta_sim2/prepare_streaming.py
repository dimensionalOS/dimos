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

"""Index an explicitly bounded region of exported collision for streaming.

Whole source triangles retain their identity and shape across window changes.
This is a coverage-limited experiment, not a claim of full-map asset coverage.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from dimos.experimental.gta_sim2.prepare import glb_triangles


def prepare(source: Path, scene: Path, output: Path, half_extent: float) -> dict[str, Any]:
    manifest = json.loads(scene.with_name("manifest.json").read_text())
    origin = np.asarray(manifest["origin_rage_m"])
    lo = origin + np.array([-half_extent, -half_extent, -5])
    hi = origin + np.array([half_extent, half_extent, 25])
    excluded = set(manifest["excluded_overlapping_sources"])
    triangles, surfaces, sources = [], [], []
    omissions: Counter[str] = Counter()
    for line in (source / "index.jsonl").read_text().splitlines():
        row = json.loads(line)
        bbox = row.get("bbox")
        if row["id"] in excluded or not bbox:
            continue
        if np.any(np.asarray(bbox[1]) < lo) or np.any(np.asarray(bbox[0]) > hi):
            continue
        path = source / Path(row["file"]).name
        count = 0
        for surface, candidate in glb_triangles(path):
            mask = np.all(candidate.max(axis=1) >= lo, axis=1) & np.all(
                candidate.min(axis=1) <= hi, axis=1
            )
            candidate = candidate[mask]
            normal = np.cross(candidate[:, 1] - candidate[:, 0], candidate[:, 2] - candidate[:, 0])
            candidate = candidate[np.linalg.norm(normal, axis=1) > 1e-8]
            triangles.extend(candidate - origin)
            surfaces.extend([surface] * len(candidate))
            count += len(candidate)
        if count:
            sources.append(
                {
                    "id": row["id"],
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "triangles": count,
                }
            )
        omissions.update(p["type"] for p in row.get("primitives", []))
    xyz = np.asarray(triangles, dtype=np.float64)
    if not len(xyz):
        raise ValueError("No exported collision in requested region")
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output / "surfaces.npz",
        triangles=xyz,
        surface_ids=np.asarray(surfaces),
        coverage_bounds=np.array([lo - origin, hi - origin]),
    )
    digest = hashlib.sha256((output / "surfaces.npz").read_bytes()).hexdigest()
    stage = Usd.Stage.Open(str(scene.resolve()))
    stage.RemovePrim("/world/gta_patch")
    patch = UsdGeom.Xform.Define(stage, "/world/gta_patch")
    patch.GetPrim().CreateAttribute(
        "gta:collision_catalog_sha256", Sdf.ValueTypeNames.String, custom=True
    ).Set(digest)
    patch.GetPrim().CreateAttribute("gta:origin", Sdf.ValueTypeNames.Double3, custom=True).Set(
        Gf.Vec3d(*origin)
    )
    visual = UsdGeom.Mesh.Define(stage, "/world/gta_patch/optical_surface")
    visual.CreatePointsAttr([Gf.Vec3f(*p) for p in xyz.reshape(-1, 3)])
    visual.CreateFaceVertexCountsAttr([3] * len(xyz))
    visual.CreateFaceVertexIndicesAttr(list(range(len(xyz) * 3)))
    visual.CreateSubdivisionSchemeAttr("none")
    visual.CreateDisplayColorAttr([(0.32, 0.35, 0.38)])
    UsdPhysics.CollisionAPI.Apply(visual.GetPrim()).CreateCollisionEnabledAttr(False)
    stage.GetRootLayer().Export(str(output / "world.usda"))
    manifest.update(
        version=2,
        bounds_rage_m=[lo.tolist(), hi.tolist()],
        sources=sources,
        contact_prisms=len(xyz),
        surface_ids=dict(Counter(surfaces)),
        streaming_catalog_sha256=digest,
        omitted_analytic_primitives_in_candidate_files=dict(omissions),
        limitations=[
            "Bounded exported static triangles only; placement and analytic primitive coverage incomplete",
            "Uniform friction 0.8 is an assumption",
            "No movable props, doors or water dynamics",
            "Only collision is streamed; the optical proxy remains resident",
        ],
    )
    manifest.pop("omitted_analytic_primitives_overlapping_patch", None)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return {
        "triangles": len(xyz),
        "sources": len(sources),
        "catalog_sha256": digest,
        "bounds_rage_m": manifest["bounds_rage_m"],
        "omitted_primitives": dict(omissions),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bounds", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--half-extent", type=float, default=32)
    args = parser.parse_args()
    if not 16 <= args.half_extent <= 128:
        parser.error("half-extent must be in [16, 128] metres")
    print(json.dumps(prepare(args.bounds, args.scene, args.output, args.half_extent)), flush=True)


if __name__ == "__main__":
    main()
