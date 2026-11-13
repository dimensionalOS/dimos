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

"""Preserve the 2025 model, decomposing only the four finger collision meshes.

Ledger: source triangles and link frames are metric and unchanged; visual,
kinematic, inertial and actuator data remain the vendor-derived model's.
MuJoCo takes a convex hull per mesh. A single hull bridges the bent fingers'
concave openings, so their contact geometry needs several convex pieces.
V-HACD is an approximation, not a measured pad/friction model.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import trimesh

from dimos.robot.galaxea.r1pro.sim.config import MODEL
from dimos.sim2.scenes.sf555.pickables import ASSETS


def prepare(destination: Path) -> None:
    model = MODEL.load()
    root = ET.fromstring(model.xml)
    rows = {}
    output = destination / "meshes"
    output.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        for index in (1, 2):
            name = f"r1pro/{side}_gripper_finger_link{index}"
            mesh = root.find(f"link[@name='{name}']/collision/geometry/mesh")
            assert mesh is not None
            source = Path(mesh.attrib["filename"])
            parts = trimesh.decomposition.convex_decomposition(
                trimesh.load_mesh(source),
                resolution=1_000_000,
                maxConvexHulls=16,
                asyncACD=False,
            )
            paths = []
            for part_index, part in enumerate(parts):
                path = output / f"{side}-finger-{index}-hull-{part_index}.stl"
                trimesh.Trimesh(**part).export(path)
                paths.append(str(path.resolve()))
            rows[name] = {
                "source": str(source),
                "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "hulls": paths,
            }
    report = {
        "model_sha256": hashlib.sha256(model.xml.encode()).hexdigest(),
        "resolution": 1_000_000,
        "max_hulls": 16,
        "fingers": rows,
    }
    (destination / "finger-collisions.json").write_text(json.dumps(report, indent=2))


def gen_urdf() -> str:
    model = MODEL.load()
    report = json.loads((ASSETS / "finger-collisions.json").read_text())
    if hashlib.sha256(model.xml.encode()).hexdigest() != report["model_sha256"]:
        raise ValueError("Regenerate finger collisions for the current R1 model")
    root = ET.fromstring(model.xml)
    for name, row in report["fingers"].items():
        source = Path(row["source"])
        if hashlib.sha256(source.read_bytes()).hexdigest() != row["source_sha256"]:
            raise ValueError(f"Finger mesh changed: {source}")
        link = root.find(f"link[@name='{name}']")
        assert link is not None
        original = link.find("collision")
        assert original is not None
        link.remove(original)
        for path in row["hulls"]:
            collision = copy.deepcopy(original)
            mesh = collision.find("geometry/mesh")
            assert mesh is not None
            mesh.set("filename", path)
            link.append(collision)
    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ASSETS)
    prepare(parser.parse_args().output)
