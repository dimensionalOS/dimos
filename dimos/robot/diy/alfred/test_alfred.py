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

from dimos.robot.diy.alfred.blueprints.vis_nav import DEPTH_FRAME, IR_ENTITY_BY_FRAME
from dimos.robot.diy.alfred.mount_tf import mount_transforms

URDF = Path(__file__).parent / "alfred.urdf"


def test_every_camera_frame_dim_slam_wants_is_rooted_at_base_link():
    """cuVSLAM places no camera at all until every configured frame resolves against
    base_link, so one missing mount edge drops every image instead of degrading."""
    parents = {t.child_frame_id: t.frame_id for t in mount_transforms()}
    for frame in [*IR_ENTITY_BY_FRAME, DEPTH_FRAME]:
        # The driver publishes the imager leaves off its own link; the mount tree owes
        # that link a path to base_link.
        link = frame.split("_")[0] + "_link"
        assert parents.get(link) == "base_link", f"{frame} has no path to base_link"


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
