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

"""Private primitive worlds for physics contracts; no runnable robot or blueprint.

SI metres/kg/radians; X forward, Y left, Z up. Two orthogonal +X/+Y sliders,
grounded base, tool zero (-.4, 0, .08). Solid-box inertias and identical visual/
collision boxes. Passive +Z hinge, .4 kg panel, COM (.225,0,.15), +/-1.5 rad stops.
These Python builders own temporary URDFs; they are not distributed robot assets.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from dimos.robot.assets.model import RobotModel
from dimos.sim2.types import Camera, Drive, Imu, Instance, Lidar, World


def _box_link(
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


def _pusher_urdf() -> ET.Element:
    root = ET.Element("robot", name="cartesian_pusher")
    _box_link(root, "base_link", (0.12, 0.3, 0.10), 2.0, ".25 .3 .4 1", "-.55 0 .05")
    _box_link(root, "carriage", (0.10, 0.10, 0.04), 0.5, ".2 .5 .8 1", "0 0 .15")
    _box_link(root, "tool", (0.10, 0.10, 0.10), 0.3, ".1 .7 .9 1")
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


def _door_urdf() -> ET.Element:
    root = ET.Element("robot", name="door")
    _box_link(root, "base", (0.04, 0.04, 0.32), 1.0, ".3 .3 .3 1", "0 0 .16")
    _box_link(root, "panel", (0.4, 0.025, 0.3), 0.4, ".7 .6 .3 1", ".225 0 .15")
    joint = ET.SubElement(root, "joint", name="hinge", type="revolute")
    ET.SubElement(joint, "parent", link="base")
    ET.SubElement(joint, "child", link="panel")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="-1.5", upper="1.5", effort="10", velocity="2")
    ET.SubElement(joint, "dynamics", damping=".1")
    return root


@pytest.fixture
def world(tmp_path: Path) -> World:
    pusher_path, door_path, box_path = (
        tmp_path / name for name in ("pusher.urdf", "door.urdf", "box.urdf")
    )
    ET.ElementTree(_pusher_urdf()).write(pusher_path, encoding="unicode")
    ET.ElementTree(_door_urdf()).write(door_path, encoding="unicode")
    # 10 cm, 200 g solid box, at its COM; same fixture geometry as the former demo asset.
    box = ET.Element("robot", name="box")
    _box_link(box, "box", (0.1, 0.1, 0.1), 0.2, ".9 .45 .12 1")
    ET.ElementTree(box).write(box_path, encoding="unicode")
    pusher = (
        RobotModel.from_file(pusher_path)
        .with_fixed_frame(
            "camera", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
        )
        .with_fixed_frame("lidar", "tool", xyz=(0, 0, 0.12))
        .with_fixed_frame("imu", "tool")
    )

    robots = tuple(
        Instance(
            name,
            pusher.with_prefix(name + "/"),
            xyz=(0, y, 0),
            drives=tuple(Drive(name + "/" + joint) for joint in ("slide_x", "slide_y")),
        )
        for name, y in (("left", 0.0), ("right", 1.2))
    )
    return World(
        (
            *robots,
            Instance(
                "box",
                RobotModel.from_file(box_path),
                xyz=(0, 0, 0.055),
                floating=True,
            ),
            Instance(
                "door",
                RobotModel.from_file(door_path).with_prefix("door/"),
                xyz=(0.6, 0.4, 0),
            ),
        ),
        tuple(
            profile(f"{name}/{profile.kind}")
            for name in ("left", "right")
            for profile in (Camera, Lidar, Imu)
        ),
    )
