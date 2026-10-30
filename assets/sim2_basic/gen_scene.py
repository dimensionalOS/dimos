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

"""Small visual core example: `dimos run sim2-basic`.

Generator ledger: invented primitives, SI metres/kg/radians, X forward/Y left/Z up.
A grounded base drives a 0.3 kg, 10 cm tool along +X (0..1 m); its zero is
(-.4, 0, .08). A 14 cm, 0.3 kg loose box starts separately at (.1, 0, .075).
The wall is fixed to the base. Geometry/COM offsets share each link's axes;
solid-box inertias, matching collision/visual shapes, no external meshes.
The `assets` root only groups model subtrees and never enters the simulator.
Camera optical +Z looks forward/down; lidar/IMU follow the moving tool.
"""

import xml.etree.ElementTree as ET


def gen_urdf() -> ET.Element:
    """Source for scene.urdf; regenerate with the URDF generation tool."""
    root = ET.Element("robot", name="sim2_basic")
    ET.SubElement(root, "link", name="assets")
    for name, size, mass, color, offset in (
        ("base", (0.12, 0.3, 0.10), 2.0, ".25 .3 .4 1", "-.55 0 .05"),
        ("tool", (0.1, 0.1, 0.1), 0.3, ".1 .7 .9 1", "0 0 0"),
        ("box", (0.14, 0.14, 0.14), 0.3, ".95 .45 .1 1", "0 0 0"),
        ("wall", (0.08, 1.6, 0.8), 5.0, ".75 .7 .6 1", "1 0 .4"),
    ):
        link = ET.SubElement(root, "link", name=name)
        inertia = ET.SubElement(link, "inertial")
        ET.SubElement(inertia, "origin", xyz=offset)
        ET.SubElement(inertia, "mass", value=str(mass))
        x, y, z = size
        ET.SubElement(
            inertia,
            "inertia",
            ixx=str(mass * (y * y + z * z) / 12),
            iyy=str(mass * (x * x + z * z) / 12),
            izz=str(mass * (x * x + y * y) / 12),
            ixy="0",
            ixz="0",
            iyz="0",
        )
        for kind in ("visual", "collision"):
            shape = ET.SubElement(link, kind)
            ET.SubElement(shape, "origin", xyz=offset)
            ET.SubElement(ET.SubElement(shape, "geometry"), "box", size=" ".join(map(str, size)))
            if kind == "visual":
                ET.SubElement(ET.SubElement(shape, "material", name=name), "color", rgba=color)
    for parent, child, origin in (
        ("assets", "base", "0 0 0"),
        ("assets", "box", "0 0 0"),
        ("base", "wall", "0 0 0"),
        ("base", "tool", "-.4 0 .08"),
    ):
        joint = ET.SubElement(
            root, "joint", name=child + "_joint", type="prismatic" if child == "tool" else "fixed"
        )
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=origin)
        if child == "tool":
            ET.SubElement(joint, "axis", xyz="1 0 0")
            ET.SubElement(joint, "limit", lower="0", upper="1", effort="80", velocity=".5")
            ET.SubElement(joint, "dynamics", damping="2")
    return root
