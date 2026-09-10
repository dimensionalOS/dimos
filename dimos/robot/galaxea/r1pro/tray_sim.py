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

"""A free tray with two graspable handles, initially supported by the worktop."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from dimos.robot.galaxea.r1pro.grasping_sim import (
    BIN_XY,
    MANIPULATION_JOINTS,
    TABLE_Z,
    prepare_grasping_scene,
)

TRAY_HANDLE_Y = 0.175
TRAY_HANDLE_Z = 0.075
TRAY_TCP_HEIGHT = TRAY_HANDLE_Z  # Finger-body offset cancels the pad-local offset.


def configure_tray_holding(model: mujoco.MjModel) -> None:
    """Steady the loaded torso and show the destination after ACT stops."""
    for name in MANIPULATION_JOINTS[:4]:
        model.actuator_biasprm[model.actuator(name).id, 2] = -80.0
    # House entities have collision meshes in hidden render group 3. The
    # laptop also needs to be visible; group selection does not affect physics.
    laptop = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "entity:laptop")
    if laptop >= 0:
        model.geom_group[model.geom_bodyid == laptop] = 2


def prepare_tray_delivery_scene(output: Path, *, scene_package: Path | None = None) -> Path:
    """Give the tray free-body dynamics; only table/finger contacts can support it."""
    output = prepare_grasping_scene(output, scene_package=scene_package, mobile=True)
    tree = ET.parse(output)
    root = tree.getroot()
    option = root.find("option")
    assert option is not None
    option.set("noslip_iterations", "20")
    world = root.find("worldbody")
    base = root.find('.//body[@name="base_link"]')
    assert world is not None and base is not None
    tray = base.find('body[@name="task_bin"]')
    assert tray is not None
    base.remove(tray)
    world.append(tray)
    tray.set("pos", f"{BIN_XY[0]} {BIN_XY[1]} {TABLE_Z + 0.001}")
    ET.SubElement(tray, "freejoint", name="task_tray_free")
    for index, geom in enumerate(tray.findall("geom")):
        geom.set("name", "bin_floor" if index == 0 else f"bin_wall_{index}")
        geom.set("mass", "0.15" if index == 0 else "0.04")
        geom.set("friction", "1.0 0.02 0.001")
        geom.set("condim", "4")
        geom.set("solref", "0.005 1")
    for side, sign in (("left", 1), ("right", -1)):
        y = BIN_XY[1] + sign * TRAY_HANDLE_Y
        ET.SubElement(
            tray,
            "geom",
            name=f"tray_{side}_handle",
            type="box",
            pos=f"0 {sign * TRAY_HANDLE_Y} {TRAY_HANDLE_Z}",
            size="0.055 0.012 0.012",
            mass="0.025",
            rgba="0.18 0.18 0.2 1",
            friction="1.5 0.02 0.001",
            condim="4",
            solref="0.005 1",
            conaffinity="3",
        )
        for x in (-0.055, 0.055):
            ET.SubElement(
                tray,
                "geom",
                name=f"tray_{side}_strut_{x}",
                type="capsule",
                fromto=f"{x} {sign * 0.11} {TRAY_HANDLE_Z} {x} {sign * TRAY_HANDLE_Y} {TRAY_HANDLE_Z}",
                size="0.006",
                mass="0.01",
                rgba="0.9 0.35 0.06 1",
                conaffinity="3",
            )
        ET.SubElement(
            tray,
            "site",
            name=f"tray_{side}_grasp",
            pos=f"0 {y - BIN_XY[1]} {TRAY_TCP_HEIGHT}",
            size="0.003",
            rgba="0 1 0 0",
        )
    ET.indent(root)
    tree.write(output, encoding="unicode")
    mujoco.MjModel.from_xml_path(str(output))
    return output
