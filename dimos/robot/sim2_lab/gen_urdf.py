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

"""Canonical demonstration robot; regenerate with the URDF skill launcher.

Design ledger: SI metres/kg/radians; X forward, Y left, Z up. Two orthogonal
prismatic joints, +X and +Y, grounded base; no floating base or humanoid policy.
Tool zero is (-.4, 0, .08). Tool is a .10 m cube, mass .3 kg; carriage is
.10 x .10 x .04 m, mass .5 kg, above the contact plane. Solid-box inertias.
The base stays behind the tool travel. Collision shapes equal visual boxes.
Camera/IMU/lidar are massless reference frames added via RobotModel, not bodies
with invented inertia. Motors and sensor rates are deployment settings.
This is an intentionally simple physical test fixture, not a manufactured robot.
"""

import xml.etree.ElementTree as ET


def box_link(
    root: ET.Element,
    name: str,
    size: tuple[float, float, float],
    mass: float,
    color: str,
    offset: str = "0 0 0",
) -> None:
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


def gen_urdf() -> ET.Element:
    root = ET.Element("robot", name="cartesian_pusher")
    box_link(root, "base_link", (0.12, 0.3, 0.10), 2.0, ".25 .3 .4 1", "-.55 0 .05")
    box_link(root, "carriage", (0.10, 0.10, 0.04), 0.5, ".2 .5 .8 1", "0 0 .15")
    box_link(root, "tool", (0.10, 0.10, 0.10), 0.3, ".1 .7 .9 1")
    for name, parent, child, origin, axis, lower, upper in (
        ("slide_x", "base_link", "carriage", "-.4 0 .08", "1 0 0", "0", ".8"),
        ("slide_y", "carriage", "tool", "0 0 0", "0 1 0", "-.3", ".3"),
    ):
        joint = ET.SubElement(root, "joint", name=name, type="prismatic")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=origin)
        ET.SubElement(joint, "axis", xyz=axis)
        ET.SubElement(joint, "limit", lower=lower, upper=upper, effort="80", velocity=".5")
        ET.SubElement(joint, "dynamics", damping="2", friction="0")
    return root
