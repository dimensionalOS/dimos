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

"""Five free bottles and a supported tray, built as a local scene overlay."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene

PACKING_SOURCES = ((0.45, -0.28), (0.34, -0.40), (0.45, -0.40), (0.24, -0.40), (0.34, -0.52))
PACKING_TRAY_XY = (0.30, -0.10)
PACKING_BODIES = ("task_bottle", *(f"task_bottle_{i}" for i in range(2, 6)))
PACKING_JOINTS = ("task_bottle_free", *(f"task_bottle_free_{i}" for i in range(2, 6)))


def prepare_packing_scene(output: Path, *, scene_package: Path | None = None) -> Path:
    """Preserve house assets; all bottles and the tray obey contact physics."""
    output = prepare_tray_delivery_scene(output, scene_package=scene_package)
    tree = ET.parse(output)
    # A loaded arm needs adequate torso damping during each Cartesian transfer.
    for index in range(1, 5):
        actuator = tree.getroot().find(f'.//actuator/general[@name="r1pro/torso_joint{index}"]')
        assert actuator is not None
        actuator.set("biasprm", "0 -250 -80")
    world = tree.getroot().find("worldbody")
    assert world is not None
    original = world.find('body[@name="task_bottle"]')
    tray = world.find('body[@name="task_bin"]')
    assert original is not None and tray is not None
    template = deepcopy(original)
    for index, (x, y) in enumerate(PACKING_SOURCES):
        body = original if index == 0 else deepcopy(template)
        if index:
            for element in body.iter():
                if "name" in element.attrib:
                    element.set("name", f"{element.attrib['name']}_{index + 1}")
            world.append(body)
        body.set("pos", f"{x} {y} 0.771")
    tray.set("pos", f"{PACKING_TRAY_XY[0]} {PACKING_TRAY_XY[1]} 0.701")
    ET.indent(tree.getroot())
    tree.write(output, encoding="unicode")
    mujoco.MjModel.from_xml_path(str(output))
    return output
