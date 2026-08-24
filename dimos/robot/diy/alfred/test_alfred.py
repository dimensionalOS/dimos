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

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.diy.alfred.blueprints.alfred_mls_nav import D455_MOUNT

URDF = Path(__file__).parent / "alfred.urdf"


def joint_origin(name: str) -> tuple[np.ndarray, np.ndarray]:
    joint = ElementTree.parse(URDF).getroot().find(f"./joint[@name='{name}']")
    assert joint is not None, f"{name} is missing from alfred.urdf"
    origin = joint.find("origin")
    assert origin is not None
    return (
        np.array([float(v) for v in origin.attrib["xyz"].split()]),
        np.array([float(v) for v in origin.attrib["rpy"].split()]),
    )


def test_the_camera_mount_the_blueprint_publishes_matches_the_urdf():
    """The camera driver publishes base_link->camera_link itself and never reads the urdf,
    so the two can drift apart silently."""
    translation, rpy = joint_origin("camera_joint")
    mount = D455_MOUNT.translation
    assert np.allclose([mount.x, mount.y, mount.z], translation, atol=1e-4)

    urdf_rotation = Rotation.from_euler("xyz", rpy)
    blueprint_rotation = Rotation.from_quat(
        [D455_MOUNT.rotation.x, D455_MOUNT.rotation.y, D455_MOUNT.rotation.z, D455_MOUNT.rotation.w]
    )
    assert (urdf_rotation.inv() * blueprint_rotation).magnitude() < 1e-3


def test_every_urdf_link_has_exactly_one_parent():
    root = ElementTree.parse(URDF).getroot()
    parents: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        assert child is not None and parent is not None
        parents.setdefault(child.attrib["link"], []).append(parent.attrib["link"])
    doubled = {child: names for child, names in parents.items() if len(names) > 1}
    assert not doubled, f"tf would reject these: {doubled}"

    links = {link.attrib["name"] for link in root.findall("link")}
    assert links - set(parents) == {"base_link"}, "every link but base_link needs a joint"
