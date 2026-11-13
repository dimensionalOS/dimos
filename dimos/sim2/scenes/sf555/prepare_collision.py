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

"""Prepare collision assets from the source objects of the saved SF555 map.

Design ledger: one fixed link sf555/world, at the saved Blender world origin.
Metres, right-handed Z-up; all geometry vertices are already in that frame.
No joints or inertial are needed for this fixed environment. Visuals retain
source triangles in a small set of combined query meshes. Contacts target G1 locomotion:
sub-10cm details have no collision; sub-1m concave components use conservative
hulls; larger concave objects use V-HACD. No whole-room convex hull is allowed. The preparation report names every approximation.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import os
from pathlib import Path
from typing import Any, cast
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from dimos.robot.assets.model import RobotModel


def prepare_object(args: tuple[Path, Path, dict[str, str]]) -> dict[str, Any]:
    source, output, row = args
    key = Path(row["file"]).stem
    cached = output / f"{key}.json"
    if cached.exists():
        return cast("dict[str, Any]", json.loads(cached.read_text()))
    with np.load(source / row["file"]) as data:
        mesh = trimesh.Trimesh(data["vertices"], data["faces"])
    center = mesh.vertices.mean(axis=0)
    mesh.vertices -= center
    visual = output / f"{key}-visual.stl"
    result: dict[str, Any] = {
        "name": row["name"],
        "visual": str(visual.resolve()),
        "origin": center.tolist(),
        "collisions": [],
        "method": "convex",
    }
    # MuJoCo builds a hull even for query-only visuals. Give a flat visual a
    # 20 micrometre backing, keeping this entirely out of physical collisions.
    _, singular, axes = np.linalg.svd(mesh.vertices, full_matrices=False)
    visible = mesh.copy()
    if singular[-1] < 1e-6:
        offset = axes[-1] * 0.00002
        visible = trimesh.util.concatenate(
            (mesh, trimesh.Trimesh(mesh.vertices + offset, mesh.faces[:, ::-1]))
        )
        result["visual_backing_m"] = 0.00002
    visible.export(visual)
    # A paper/texture plane has no volume; adding fictitious solid walls would
    # change the scene. Keep it visible and record the omission explicitly.
    singular = np.linalg.svd(mesh.vertices - mesh.vertices.mean(axis=0), compute_uv=False)
    if (
        not len(mesh.faces)
        or singular[-1] < 1e-6
        or min(mesh.extents) < 0.001
        or max(mesh.extents) < 0.1
    ):
        result["method"] = "visual_only_small_or_thin_detail"
        parts = []
    elif mesh.is_convex:
        parts = [mesh.convex_hull]
    elif max(mesh.extents) < 1.0:
        # Conservative envelopes for individual furniture components. The
        # environment is a locomotion model, not a grasp/contact-detail model.
        parts = [mesh.convex_hull]
        result["method"] = "component_hull_below_1m"
    else:
        large = max(mesh.extents) >= 3
        parts = [
            trimesh.Trimesh(**part)
            for part in trimesh.decomposition.convex_decomposition(
                mesh,
                resolution=400_000 if large else 100_000,
                maxConvexHulls=64 if large else 16,
                asyncACD=False,
            )
        ]
        result["method"] = "vhacd_400k_64" if large else "vhacd_100k_16"
    for i, part in enumerate(parts):
        path = output / f"{key}-collision-{i}.stl"
        part.export(path)
        result["collisions"].append(str(path.resolve()))
    cached.write_text(json.dumps(result))
    return result


def prepare(source: Path, destination: Path, workers: int = 6) -> None:
    objects = json.loads((source / "objects.json").read_text())
    meshes = destination / "meshes" / (objects["source_sha256"][:16] + "-vhacd-v4")
    meshes.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = []
        for index, result in enumerate(
            pool.map(
                prepare_object, ((source, meshes, row) for row in objects["objects"]), chunksize=1
            )
        ):
            results.append(result)
            if index % 100 == 0:
                print(f"Prepared {index}/{len(objects['objects'])}: {result['name']}", flush=True)
    for row in results:
        valid, rejected = [], []
        for path in row["collisions"]:
            part = trimesh.load_mesh(path)
            _, _, axes = np.linalg.svd(
                part.vertices - part.vertices.mean(axis=0), full_matrices=False
            )
            # Decomposition of an open decorative surface can yield flat hulls.
            # They cannot support physical volume and MuJoCo rejects them.
            if len(part.vertices) < 4 or np.ptp(part.vertices @ axes[-1]) < 0.00001:
                rejected.append(path)
            else:
                valid.append(path)
        row["collisions"] = valid
        row["discarded_degenerate_hulls"] = rejected
    report = {
        "source_sha256": objects["source_sha256"],
        "algorithm": "vhacd",
        "large_object_resolution": 400_000,
        "large_object_hulls": 64,
        "small_object_resolution": 100_000,
        "small_object_hulls": 16,
        "conservative_hull_below_m": 1.0,
        "visual_only_below_m": 0.1,
        "objects": results,
    }
    # All environment visuals belong to the same fixed body. A single query
    # mesh preserves their triangles and avoids quadratic name checks during
    # MuJoCo import. Native Unreal still uses the original material-rich level.
    vertices, faces = [], []
    offset = 0
    for row in results:
        mesh = trimesh.load_mesh(row["visual"])
        vertices.append(mesh.vertices + row["origin"])
        faces.append(mesh.faces + offset)
        offset += len(mesh.vertices)
    all_vertices, all_faces = np.concatenate(vertices), np.concatenate(faces)
    report["visuals"] = []
    # MuJoCo's binary STL decoder limits each asset to 200,000 faces.
    for index, start in enumerate(range(0, len(all_faces), 180_000)):
        visual_path = meshes / f"office-query-{index:03d}.stl"
        trimesh.Trimesh(all_vertices, all_faces[start : start + 180_000], process=False).export(
            visual_path
        )
        report["visuals"].append(str(visual_path.resolve()))
    (destination / "collision-report.json").write_text(json.dumps(report))
    root = description(report)
    ET.indent(root)
    path = destination / "office.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"))
    RobotModel.from_file(path).load()
    print(f"Prepared {path}", flush=True)


def description(report: dict[str, Any]) -> ET.Element:
    robot = ET.Element("robot", name="sf555_office")
    link = ET.SubElement(robot, "link", name="sf555/world")
    for index, path in enumerate(report.get("visuals", [])):
        visual = ET.SubElement(link, "visual", name=f"office/query/{index}")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=path)
    for row in report["objects"]:
        for kind, paths in (
            ("visual", [] if "visuals" in report else [row["visual"]]),
            ("collision", row["collisions"]),
        ):
            for index, path in enumerate(paths):
                shape = ET.SubElement(link, kind, name=f"{row['name']}/{kind}/{index}")
                ET.SubElement(shape, "origin", xyz=" ".join(map(str, row["origin"])))
                ET.SubElement(ET.SubElement(shape, "geometry"), "mesh", filename=path)
    return robot


def gen_urdf() -> ET.Element:
    """Regenerate the explicit prepared target using the URDF skill launcher."""
    root = Path(os.environ.get("DIMOS_SF555_ASSETS", str(Path.home() / ".cache/dimos/sf555")))
    return description(json.loads((root / "collision-report.json").read_text()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    prepare(args.source, args.output, args.workers)
