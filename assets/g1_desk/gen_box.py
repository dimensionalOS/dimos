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

"""Scene box: SI, .1 m solid cube of .2 kg, root at its centre, +Z up.

The visual, collision and COM coincide. Inertia is m * side**2 / 6 on each
axis. This is the loose object used by G1 and the private physics tests.
"""

import xml.etree.ElementTree as ET


def gen_urdf() -> ET.Element:
    root = ET.Element("robot", name="box")
    link = ET.SubElement(root, "link", name="box")
    inertia = ET.SubElement(link, "inertial")
    ET.SubElement(inertia, "origin", xyz="0 0 0")
    ET.SubElement(inertia, "mass", value="0.2")
    diagonal = str(0.2 * (0.1**2 + 0.1**2) / 12)
    ET.SubElement(
        inertia, "inertia", ixx=diagonal, iyy=diagonal, izz=diagonal, ixy="0", ixz="0", iyz="0"
    )
    for kind in ("visual", "collision"):
        shape = ET.SubElement(link, kind)
        ET.SubElement(shape, "origin", xyz="0 0 0")
        ET.SubElement(ET.SubElement(shape, "geometry"), "box", size="0.1 0.1 0.1")
        if kind == "visual":
            ET.SubElement(
                ET.SubElement(shape, "material", name="box"), "color", rgba=".9 .45 .12 1"
            )
    return root
