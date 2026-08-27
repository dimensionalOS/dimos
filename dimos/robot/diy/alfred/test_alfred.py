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

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.diy.alfred.blueprints.alfred_mls_nav import D455_MOUNT
from dimos.robot.diy.alfred.blueprints.vis_nav import (
    ALFRED_RERUN_ROOT,
    GLOBAL_PATH_PURPLE,
    IR_ENTITY_BY_FRAME,
    _alfred_urdf_static,
    _ir_image,
    _ir_pinhole,
    _path_colored,
    _rerun_blueprint,
)

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
    """The camera driver publishes base_link->d455_link itself and never reads the urdf,
    so the two can drift apart silently."""
    translation, rpy = joint_origin("d455_joint")
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


@dataclass
class FakeMessage:
    """Stands in for an image or path message; `to_rerun` just records how it was called."""

    frame_id: str = ""

    def to_rerun(self, **kwargs: Any) -> dict[str, Any]:
        return kwargs


def test_each_ir_eye_routes_to_its_own_entity():
    """Both imagers arrive on one topic, so the entity can only come from the frame id.
    If the two collapsed onto one path each eye would overwrite the other."""
    left = _ir_image(FakeMessage("d455_infra1_optical_frame"))
    right = _ir_image(FakeMessage("d455_infra2_optical_frame"))
    assert left is not None and right is not None
    assert left[0][0] != right[0][0]
    assert _ir_image(FakeMessage("d455_color_optical_frame")) is None


def test_ir_pinhole_keeps_the_frame_and_entity_agreeing():
    """The pinhole has to name the same entity the image went to and the frame it came
    from, or the calibration lands on the wrong eye."""
    pinhole = _ir_pinhole(FakeMessage("d455_infra2_optical_frame"))
    assert pinhole["image_topic"] == IR_ENTITY_BY_FRAME["d455_infra2_optical_frame"]
    assert pinhole["optical_frame"] == "d455_infra2_optical_frame"
    assert _ir_pinhole(FakeMessage("d455_color_optical_frame")) is None


def test_global_path_is_recolored_away_from_the_local_one():
    assert _path_colored(FakeMessage(), GLOBAL_PATH_PURPLE)["color"] == GLOBAL_PATH_PURPLE


def test_every_published_camera_entity_has_a_view_to_render_into():
    """The stock bridge blueprint is 3D only. An image published to an entity with no
    2D view is recorded but never drawn, which looks like a dead camera."""
    views: list[str] = []

    def collect(node: Any) -> None:
        origin = getattr(node, "origin", None)
        if origin is not None:
            views.append(str(origin))
        for child in getattr(node, "contents", None) or []:
            collect(child)

    collect(_rerun_blueprint().root_container)
    assert set(IR_ENTITY_BY_FRAME.values()) <= set(views)


def test_urdf_static_hangs_the_robot_off_base_link():
    """The mesh tree is static, so it only lines up with the live robot if it is parented
    to the tf base_link rather than left at the origin."""
    import rerun as rr

    entities = _alfred_urdf_static(rr)
    assert entities, "the urdf produced no rerun entities"
    assert entities[-1][0] == ALFRED_RERUN_ROOT
