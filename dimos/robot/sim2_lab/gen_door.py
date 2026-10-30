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

"""SI, grounded root at hinge; panel extends +X; +Z hinge opens counterclockwise.

Solid .4 x .025 x .3 m panel, .4 kg, COM at (.225,0,.15); ±1.5 rad stops.
Identical visual/collision boxes; root frame carries a small fixed support.
"""

import xml.etree.ElementTree as ET

from dimos.robot.sim2_lab.gen_urdf import box_link


def gen_urdf() -> ET.Element:
    root = ET.Element("robot", name="door")
    box_link(root, "base", (0.04, 0.04, 0.32), 1.0, ".3 .3 .3 1", "0 0 .16")
    box_link(root, "panel", (0.4, 0.025, 0.3), 0.4, ".7 .6 .3 1", ".225 0 .15")
    joint = ET.SubElement(root, "joint", name="hinge", type="revolute")
    ET.SubElement(joint, "parent", link="base")
    ET.SubElement(joint, "child", link="panel")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="-1.5", upper="1.5", effort="10", velocity="2")
    ET.SubElement(joint, "dynamics", damping=".1")
    return root
