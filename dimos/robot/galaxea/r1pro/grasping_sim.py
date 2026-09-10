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

"""R1Pro contact-grasping scene shared by demonstrations and policy rollout.

The 2026 CAD model omits gripper travel. The G1 simulator reference specifies
0.05 m per finger with opposing axes. We add that travel and symmetric coupling,
and approximate the CAD fingertips with flat rubber contact pads. These are
simulation parameters, not a calibrated model of real gripper dynamics.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from dimos.robot.galaxea.r1pro.learning import R1PRO_GRIPPER_JOINTS, R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.sim_act import prepare_r1pro_act_scene

GRIPPER_JOINTS = R1PRO_GRIPPER_JOINTS
MANIPULATION_JOINTS = R1PRO_PICK_PLACE_JOINTS
TABLE_Z = 0.70
BOTTLE_RADIUS = 0.025
BOTTLE_HALF_HEIGHT = 0.07
BOTTLE_XY = (0.43, -0.27)
BIN_XY = (0.36, -0.005)
BIN_INNER_HALF_SIZE = (0.105, 0.105)
BIN_FLOOR_Z = TABLE_Z + 0.015
TCP_OFFSET = (0.0, 0.0, -0.085)
VIRTUAL_BASE_JOINTS = ("r1pro/base_x", "r1pro/base_y", "r1pro/base_yaw")


def _configure_planar_ground_contacts(root: ET.Element, base: ET.Element) -> None:
    """A planar stage supplies chassis support; retain its obstacle collisions.

    The frozen CAD wheels were not intended as a ground-contact model. Once a
    planar joint is added, their convex floor contacts can push the chassis out
    of its parked pose. Collision bit 2 excludes only low ground surfaces from
    the fixed chassis; arms, tray, and cargo continue to collide with the floor.
    """
    world = root.find("worldbody")
    assert world is not None
    nodes = list(world.iter("geom"))
    for index, geom in enumerate(nodes):
        if "name" not in geom.attrib:
            geom.set("name", f"r1pro_stage_geom_{index}")
    fixed_names = {base.attrib["name"]}
    pending = list(base.findall("body"))
    while pending:
        body = pending.pop()
        if body.find("joint") is not None or body.attrib["name"] == "task_bin":
            continue
        fixed_names.add(body.attrib["name"])
        pending.extend(body.findall("body"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot_names = {body.attrib["name"] for body in base.iter("body")}
    for node in nodes:
        gid = model.geom(node.attrib["name"]).id
        if model.geom_contype[gid] == 0 and model.geom_conaffinity[gid] == 0:
            continue
        body_name = model.body(int(model.geom_bodyid[gid])).name
        rotation = data.geom_xmat[gid].reshape(3, 3)
        centre = data.geom_xpos[gid] + rotation @ model.geom_aabb[gid, :3]
        top = centre[2] + (np.abs(rotation) @ model.geom_aabb[gid, 3:])[2]
        ground = body_name not in robot_names and (
            model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_PLANE or top <= 0.06
        )
        if body_name in fixed_names:
            node.set("contype", "2")
            node.set("conaffinity", "2")
        elif ground:
            node.set("contype", "1")
            node.set("conaffinity", "1")
        else:
            node.set("conaffinity", str(int(model.geom_conaffinity[gid]) | 2))


def prepare_grasping_scene(
    output: Path,
    *,
    scene_package: Path | None = None,
    mobile: bool = False,
) -> Path:
    """Build a tabletop bottle-to-bin scene with two actuated G1 grippers."""
    with TemporaryDirectory(prefix="r1pro-grasp-") as directory:
        base = prepare_r1pro_act_scene(Path(directory) / "base.xml", scene_package=scene_package)
        root = ET.parse(base).getroot()
    bodies = {body.attrib["name"]: body for body in root.findall(".//body")}
    actuators = root.find("actuator")
    world = root.find("worldbody")
    assert actuators is not None and world is not None
    equality = ET.SubElement(root, "equality")
    for side, primary in zip(("left", "right"), GRIPPER_JOINTS, strict=True):
        follower = f"{side}_gripper_follower"
        for finger, name, sign in ((1, primary, 1), (2, follower, -1)):
            body = bodies[f"{side}_gripper_finger_link{finger}"]
            ET.SubElement(
                body,
                "joint",
                name=name,
                type="slide",
                axis=f"0 {sign} 0",
                range="0 0.05",
                damping="1",
                armature="0.01",
            )
            for geom in body.findall("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
            # Origins are from the pinned 2026 finger links. Pads meet at the
            # palm centre at zero and separate by 2*q; the grasp centre is -Z.
            ET.SubElement(
                body,
                "geom",
                name=f"{side}_finger_pad{finger}",
                type="box",
                pos=f"{-sign * 0.017997} {sign * 0.027003} -0.0972",
                size="0.009 0.0025 0.0175",
                rgba="0.12 0.12 0.14 1",
                friction="1.5 0.02 0.001",
                condim="4",
                solref="0.005 1",
                mass="0.001",
            )
        ET.SubElement(
            equality,
            "joint",
            joint1=follower,
            joint2=primary,
            polycoef="0 1 0 0 0",
            solref="0.002 1",
        )
        ET.SubElement(
            actuators,
            "position",
            name=primary,
            joint=primary,
            kp="4000",
            kv="30",
            ctrlrange="0 0.05",
            forcerange="-50 50",
        )
        palm = bodies[f"{side}_gripper_link"]
        ET.SubElement(
            palm, "site", name=f"{side}_tcp", pos="0 0 -0.085", size="0.004", rgba="0 1 0 0"
        )
        ET.SubElement(
            palm,
            "camera",
            name=f"{side}_wrist",
            pos="0.08 0 0.025",
            xyaxes="0 -1 0 0.809 0 -0.588",
            fovy="75",
        )
    ET.SubElement(
        bodies["torso_link4"],
        "camera",
        name="head",
        pos="0.12 0 0.45",
        xyaxes="-.481890 -.876232 0 .821375 -.451721 .348270",
        fovy="55",
    )
    table = ET.SubElement(world, "body", name="task_table", pos=f"0.49 -0.18 {TABLE_Z - 0.025}")
    ET.SubElement(
        table,
        "geom",
        type="box",
        size="0.31 0.40 0.025",
        rgba="0.7 0.68 0.62 1",
        friction="0.8 0.005 0.001",
    )
    for x in (-0.18, 0.26):
        for y in (-0.35, 0.35):
            ET.SubElement(
                table,
                "geom",
                type="box",
                pos=f"{x} {y} -0.325",
                size="0.025 0.025 0.325",
                rgba="0.3 0.3 0.3 1",
            )
    bottle = ET.SubElement(
        world,
        "body",
        name="task_bottle",
        pos=f"{BOTTLE_XY[0]} {BOTTLE_XY[1]} {TABLE_Z + BOTTLE_HALF_HEIGHT + 0.001}",
    )
    ET.SubElement(bottle, "freejoint", name="task_bottle_free")
    ET.SubElement(
        bottle,
        "geom",
        name="bottle_body",
        type="cylinder",
        size=f"{BOTTLE_RADIUS} {BOTTLE_HALF_HEIGHT}",
        mass="0.12",
        rgba="0.05 0.4 0.85 1",
        friction="1.0 0.02 0.001",
        condim="4",
        solref="0.005 1",
    )
    ET.SubElement(
        bottle,
        "geom",
        name="bottle_cap",
        type="cylinder",
        size="0.014 0.009",
        pos="0 0 0.079",
        mass="0.005",
        rgba="0.95 0.95 0.95 1",
    )
    container = ET.SubElement(
        world, "body", name="task_bin", pos=f"{BIN_XY[0]} {BIN_XY[1]} {TABLE_Z}"
    )
    ET.SubElement(
        container,
        "geom",
        name="bin_floor",
        type="box",
        pos="0 0 0.0075",
        size="0.115 0.115 0.0075",
        rgba="0.9 0.35 0.06 1",
    )
    for axis in (0, 1):
        for sign in (-1, 1):
            pos = [0.0, 0.0, 0.045]
            size = [0.115, 0.115, 0.045]
            pos[axis] = sign * 0.11
            size[axis] = 0.005
            ET.SubElement(
                container,
                "geom",
                type="box",
                pos=" ".join(map(str, pos)),
                size=" ".join(map(str, size)),
                rgba="0.9 0.35 0.06 1",
            )
    if mobile:
        # A planar simulation stage stands in for the wheeled chassis controller.
        # Steering/wheel contact dynamics are not being claimed by this demo.
        base_body = bodies["base_link"]
        for name, stage_axis, kind, limits in zip(
            VIRTUAL_BASE_JOINTS,
            ("1 0 0", "0 1 0", "0 0 1"),
            ("slide", "slide", "hinge"),
            ("-10 10", "-10 10", "-6.28 6.28"),
            strict=True,
        ):
            ET.SubElement(
                base_body,
                "joint",
                name=name,
                type=kind,
                axis=stage_axis,
                range=limits,
                damping="1000",
                armature="1",
            )
            ET.SubElement(
                actuators,
                "position",
                name=name,
                joint=name,
                kp="100000",
                kv="3000",
                ctrlrange=limits,
                forcerange="-2000 2000",
            )
        # The bin is an onboard tray. The bottle remains a free body and is
        # transported by contact with the tray, with no bottle-to-robot weld.
        # Leave physical clearance above the workstation: a coplanar tray
        # bottom would scrape the table and excite the planar servo.
        container.set("pos", f"{BIN_XY[0]} {BIN_XY[1]} {TABLE_Z + 0.002}")
        world.remove(container)
        base_body.append(container)
        _configure_planar_ground_contacts(root, base_body)
    ET.indent(root)
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="unicode")
    mujoco.MjModel.from_xml_path(str(output))
    return output
