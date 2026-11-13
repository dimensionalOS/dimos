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

"""Extract two complete downstairs bottles into independent rigid bodies.

Ledger: source SF555 V24 world coordinates are metres, right-handed and Z-up.
The fixed office retains its original origin. Each bottle frame is the centre
of its source mesh bounds; all cap/label/body visuals move together. Collision
is a convex hull of those source triangles. Assumed mass is 0.30 kg (part-full
water bottle); inertia approximates a solid cylinder about the bounds centre.
No fluid, shell deformation or measured material friction is claimed. Free
motion belongs to SceneConfig.Instance(floating=True), not a URDF joint.
"""

import argparse
import copy
import json
import os
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from dimos.sim2.scenes.sf555.prepare_collision import description

BOTTLES = {
    "bottle_a": "Hall_A_middle_water_bottle_",
    "bottle_b": "Hall_A_outer_water_bottle_",
}
ASSUMED_MASS_KG = 0.30
ASSETS = Path(
    os.environ.get("DIMOS_SF555_JOYLO_ASSETS", str(Path.home() / ".cache/dimos/sf555-joylo"))
)


def bottle_description(row: dict[str, Any]) -> ET.Element:
    root = ET.Element("robot", name=row["id"])
    link = ET.SubElement(root, "link", name=f"props/{row['id']}")
    inertia = ET.SubElement(link, "inertial")
    ET.SubElement(inertia, "mass", value=str(ASSUMED_MASS_KG))
    radius, height = row["radius_m"], row["height_m"]
    transverse = ASSUMED_MASS_KG * (3 * radius**2 + height**2) / 12
    axial = ASSUMED_MASS_KG * radius**2 / 2
    ET.SubElement(
        inertia,
        "inertia",
        ixx=str(transverse),
        iyy=str(transverse),
        izz=str(axial),
        ixy="0",
        ixz="0",
        iyz="0",
    )
    for part in row["parts"]:
        visual = ET.SubElement(link, "visual", name=part["name"])
        ET.SubElement(visual, "origin", xyz=" ".join(map(str, part["local_origin"])))
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=part["visual"])
        rgba = "0.08 0.20 0.65 1" if "cap" in part["name"] else "0.65 0.85 0.95 1"
        ET.SubElement(ET.SubElement(visual, "material", name=part["name"]), "color", rgba=rgba)
    collision = ET.SubElement(link, "collision")
    ET.SubElement(ET.SubElement(collision, "geometry"), "mesh", filename=row["collision"])
    return root


def prepare(source: Path, destination: Path) -> None:
    report = json.loads((source / "collision-report.json").read_text())
    destination.mkdir(parents=True, exist_ok=True)
    meshes = destination / "meshes"
    meshes.mkdir(exist_ok=True)
    selected = {
        key: [r for r in report["objects"] if r["name"].startswith(prefix)]
        for key, prefix in BOTTLES.items()
    }
    if any(not rows for rows in selected.values()):
        raise ValueError("The expected downstairs bottle assemblies are missing")
    removed = {r["name"] for rows in selected.values() for r in rows}
    office = copy.deepcopy(report)
    office["objects"] = [r for r in report["objects"] if r["name"] not in removed]
    # Rebuild only the visual query mesh. Reuse all unaffected collision assets.
    # This removes bottles from both static contact and static optical geometry.
    vertices, faces = [], []
    vertex_offset = 0
    for row in office["objects"]:
        mesh = trimesh.load_mesh(row["visual"], process=False)
        vertices.append(mesh.vertices + row["origin"])
        faces.append(mesh.faces + vertex_offset)
        vertex_offset += len(mesh.vertices)
    all_vertices, all_faces = np.concatenate(vertices), np.concatenate(faces)
    office["visuals"] = []
    for index, start in enumerate(range(0, len(all_faces), 180_000)):
        path = meshes / f"office-query-{index:03d}.stl"
        trimesh.Trimesh(all_vertices, all_faces[start : start + 180_000], process=False).export(
            path
        )
        office["visuals"].append(str(path.resolve()))
    (destination / "collision-report.json").write_text(json.dumps(office))
    objects = []
    for key, parts in selected.items():
        assembled = trimesh.util.concatenate(
            [
                trimesh.load_mesh(r["visual"], process=False).apply_translation(r["origin"])
                for r in parts
            ]
        )
        center = assembled.bounds.mean(axis=0)
        extents = assembled.extents
        assembled.apply_translation(-center)
        collision = meshes / f"{key}-collision.stl"
        assembled.convex_hull.export(collision)
        objects.append(
            {
                "id": key,
                "prefix": BOTTLES[key],
                "position": center.tolist(),
                "radius_m": float(max(extents[:2]) / 2),
                "height_m": float(extents[2]),
                "collision": str(collision.resolve()),
                "parts": [
                    {**r, "local_origin": (np.array(r["origin"]) - center).tolist()} for r in parts
                ],
            }
        )
    (destination / "pickables.json").write_text(
        json.dumps({"source_sha256": report["source_sha256"], "bottles": objects}, indent=2)
    )
    print(
        json.dumps(
            {
                "output": str(destination),
                "removed_static_parts": len(removed),
                "bottles": [
                    {k: r[k] for k in ("id", "position", "height_m", "radius_m")} for r in objects
                ],
            }
        ),
        flush=True,
    )


def gen_urdf() -> ET.Element:
    """Explicit target selected by SF555_URDF_TARGET: office, bottle_a, bottle_b."""
    target = os.environ.get("SF555_URDF_TARGET", "office")
    if target == "office":
        return description(json.loads((ASSETS / "collision-report.json").read_text()))
    rows = json.loads((ASSETS / "pickables.json").read_text())["bottles"]
    return bottle_description(next(row for row in rows if row["id"] == target))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path.home() / ".cache/dimos/sf555")
    parser.add_argument("--output", type=Path, default=ASSETS)
    args = parser.parse_args()
    prepare(args.source, args.output)
