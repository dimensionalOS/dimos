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

"""Extract one existing office desk, computer and chair; no runtime scene builder.

Design ledger: existing scene_office1 OBJ assets use metres, Y up. Select the
workstation in source X [-2.85,-1.0], with separate Z bounds for the complete
desk, computer and chair (no cut faces); rotate Rx(pi/2) to Z up and
translate its desk centre to (0,0,0). Root desk is fixed, with no invented mass.
Visuals preserve the original UVs/diffuse maps. Collision boxes approximate the measured tabletop, legs and brace; the chair uses seat/back/base boxes. The computer is static decorative visual.
Targets: URDF manager/Desktop and native MuJoCo. This is a scene fixture, no joints.
Run prepare_meshes() explicitly before regenerating the URDF with the skill launcher.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from dimos.utils.data import LfsPath

ASSETS = Path(__file__).resolve().parents[4] / "assets" / "g1_desk"
_PARTS = (
    ("desk", "Cube_007_woodenDesk", "plane_divided_wood_floor_BaseColor.png"),
    ("computer", "Plane_037_Desktop", "Computer_low_Desktop_BaseColor.png"),
    ("chair", "BezierCurve_009_SwivlChair", "SwivlChair_low_SwivlChair_BaseColor.png"),
)


def prepare_meshes() -> None:
    """One-time derived visual assets; source remains the existing LFS office pack."""
    ASSETS.mkdir(parents=True, exist_ok=True)
    source = Path(str(LfsPath("mujoco_sim/scene_office1/office_split")))
    for name, stem, _ in _PARTS:
        mesh = trimesh.load_mesh(source / (stem + ".obj"), process=False)
        centres = mesh.triangles_center
        z_min, z_max = (2.8, 4.0) if name == "chair" else (3.6, 4.4 if name == "computer" else 4.5)
        selected = (
            (centres[:, 0] > -2.85)
            & (centres[:, 0] < -1.0)
            & (centres[:, 2] > z_min)
            & (centres[:, 2] < z_max)
        )
        part = mesh.submesh([np.flatnonzero(selected)], append=True)
        if not isinstance(part, trimesh.Trimesh) or not len(part.faces):
            raise ValueError(f"no faces selected for {name}")
        (ASSETS / (name + ".obj")).write_text(str(part.export(file_type="obj")).strip() + "\n")


def gen_urdf() -> ET.Element:
    root = ET.Element("robot", name="office_desk")
    link = ET.SubElement(root, "link", name="desk")
    for name, _, texture in _PARTS:
        visual = ET.SubElement(link, "visual", name="desk/" + name)
        ET.SubElement(visual, "origin", xyz="1.9 4.08 0", rpy="1.5707963267948966 0 0")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=name + ".obj")
        material = ET.SubElement(visual, "material", name="desk_" + name)
        ET.SubElement(material, "color", rgba="1 1 1 1")
        ET.SubElement(material, "texture", filename="package://office/textures/" + texture)
    # Dimensions measured from the retained source desk, not an enclosing convex hull.
    for xyz, size in (
        ("0 0 .635", "1.65 .86 .07"),
        ("-.72 0 .315", ".05 .72 .63"),
        (".72 0 .315", ".05 .72 .63"),
        ("0 -.32 .31", "1.44 .04 .40"),
        ("-.03 .6 .43", ".58 .55 .10"),
        ("-.03 .99 .72", ".55 .08 .46"),
        ("-.03 .6 .22", ".12 .12 .44"),
    ):
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", xyz=xyz)
        ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=size)
    return root
