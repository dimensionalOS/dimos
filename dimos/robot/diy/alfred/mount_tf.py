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

"""Alfred's mount tree, read off alfred.urdf and published onto tf.

Every sensor driver publishes only its own subtree, rooted at its own link, so
nothing connects base_link to d455_link or mid360_link. cuVSLAM resolves its rig
by looking up base_link -> each camera frame and places no camera at all until
every one of them resolves, so without these edges it drops every frame it is
handed.
"""

from __future__ import annotations

from xml.etree import ElementTree

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.diy.alfred.config import ALFRED_URDF


def mount_transforms() -> list[Transform]:
    """One transform per fixed joint of the urdf, minus the imager frames.

    The drivers publish their own imager offsets from the factory extrinsics read
    off the device; the urdf's copies of those are nominal, so publishing them too
    would put a second, worse answer on tf for the same edge.
    """
    transforms = []
    for joint in ElementTree.parse(ALFRED_URDF).getroot().findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        if parent is None or child is None or origin is None:
            continue
        child_link = child.attrib["link"]
        if child_link.endswith("_frame"):
            continue
        translation = [float(value) for value in origin.attrib["xyz"].split()]
        rpy = [float(value) for value in origin.attrib["rpy"].split()]
        transforms.append(
            Transform(
                translation=Vector3(*translation),
                rotation=Quaternion.from_euler(Vector3(*rpy)),
                frame_id=parent.attrib["link"],
                child_frame_id=child_link,
            )
        )
    return transforms


class AlfredMountTf(StaticTfPublisher):
    """Publishes Alfred's urdf mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return mount_transforms()
