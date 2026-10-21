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

import xml.etree.ElementTree as ElementTree

from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.diy.alfred.blueprints.vis_nav import DEPTH_FRAME, IR_ENTITY_BY_FRAME
from dimos.robot.diy.alfred.config import ALFRED_URDF
from dimos.robot.diy.alfred.mount_tf import CAMERA_LINKS, mount_transforms


def _buffer(root_frame: str) -> MultiTBuffer:
    buffer = MultiTBuffer()
    buffer.receive_transform(*mount_transforms(root_frame))
    return buffer


def test_every_camera_frame_dim_slam_wants_is_rooted_at_base_link():
    """cuVSLAM places no camera at all until every configured frame resolves against
    base_link, so one missing mount edge drops every image instead of degrading."""
    buffer = _buffer("base_link")
    for frame in [*IR_ENTITY_BY_FRAME, DEPTH_FRAME]:
        # The driver publishes the imager leaves off its own link; the mount tree owes
        # that link a path to base_link.
        link = frame.rsplit("_", 3)[0] + "_link"
        assert buffer.get("base_link", link) is not None, f"{frame} has no path to base_link"


def test_camera_imager_frames_are_left_to_their_drivers():
    published = {t.child_frame_id for t in mount_transforms("base_link")}
    assert CAMERA_LINKS <= published
    assert not published & {*IR_ENTITY_BY_FRAME, DEPTH_FRAME}


def test_every_urdf_link_has_exactly_one_parent():
    root = ElementTree.parse(ALFRED_URDF).getroot()
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


def test_mount_tree_rooted_at_mid360_link_composes_back_to_the_urdf():
    """Point-LIO owns the one parent of mid360_link, so the mount tree has to hang below it."""
    assert "mid360_link" not in {t.child_frame_id for t in mount_transforms("mid360_link")}
    rooted, reference = _buffer("mid360_link"), _buffer("base_link")
    for frame in ["mid360_link", *CAMERA_LINKS]:
        leg, expected = rooted.get("base_link", frame), reference.get("base_link", frame)
        assert leg is not None and expected is not None, f"{frame} has no path to base_link"
        assert (leg.translation - expected.translation).length() < 1e-9
        assert abs(abs(leg.rotation.dot(expected.rotation)) - 1.0) < 1e-9
