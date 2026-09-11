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


def alfred_mount_transforms() -> list[Transform]:
    """Fixed sensor-mount edges of alfred_v1 that hang off base_link.

    The lift subtree moves with the pillar and is left to the planner; the imager frames
    under each camera_*_link are published by the RealSense driver from the device's own
    extrinsics. Rooted at mid360_link: Point-LIO owns the lidar's parent edge.
    """
    from dimos.robot.diy.alfred.alfred_model import ALFRED_V1_MODEL

    root = ElementTree.fromstring(ALFRED_V1_MODEL.load().xml)
    fixed = []
    children: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        if parent is None or child is None or origin is None:
            continue
        children.setdefault(parent.attrib["link"], []).append(child.attrib["link"])
        if joint.get("type") == "fixed":
            fixed.append((parent.attrib["link"], child.attrib["link"], origin))
    moving = _descendants("lift_link", children)
    transforms = []
    for parent_link, child_link, origin in fixed:
        if parent_link in moving or parent_link == "lift_link":
            continue
        if _is_camera_imager_edge(parent_link):
            continue
        translation = [float(value) for value in origin.attrib["xyz"].split()]
        rpy = [float(value) for value in origin.attrib["rpy"].split()]
        transform = Transform(
            translation=Vector3(*translation),
            rotation=Quaternion.from_euler(Vector3(*rpy)),
            frame_id=parent_link,
            child_frame_id=child_link,
        )
        if parent_link == "base_link" and child_link == "mid360_link":
            transform = -transform
        transforms.append(transform)
    return transforms


def _descendants(link: str, children: dict[str, list[str]]) -> set[str]:
    found: set[str] = set()
    stack = list(children.get(link, []))
    while stack:
        node = stack.pop()
        if node not in found:
            found.add(node)
            stack.extend(children.get(node, []))
    return found


def _is_camera_imager_edge(parent_link: str) -> bool:
    return parent_link.startswith("camera_") and not parent_link.endswith("_bottom_screw_frame")


class AlfredLidarMountTf(StaticTfPublisher):
    """Publishes the alfred_v1 sensor mounts rooted at mid360_link, for lidar odometry."""

    def transforms(self) -> list[Transform]:
        return alfred_mount_transforms()
