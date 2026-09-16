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

Sensor drivers publish only their own subtree, so nothing otherwise connects
base_link to d455_link or mid360_link - and cuVSLAM drops every frame until its
whole rig resolves against base_link.
"""

from __future__ import annotations

from xml.etree import ElementTree

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.diy.alfred.config import ALFRED_URDF


def mount_transforms(root_frame: str = "base_link") -> list[Transform]:
    """One transform per fixed joint of the urdf, minus the imager frames.

    Imager frames are skipped: the drivers publish factory extrinsics for those,
    and the urdf's copies are nominal.

    ``root_frame`` re-roots the tree when odometry already parents a frame - on
    alfred-nav Point-LIO owns mid360_link, so that edge must be inverted or the
    lidar ends up with two parents.
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
    if root_frame != "base_link":
        for index, transform in enumerate(transforms):
            if transform.child_frame_id == root_frame:
                transforms[index] = -transform
                break
        else:
            raise ValueError(f"{ALFRED_URDF.name} has no base_link -> {root_frame} joint")
    return transforms


class AlfredMountTfConfig(StaticTfPublisherConfig):
    # Frame the tree hangs from; set it to whichever frame odometry already parents.
    root_frame: str = "base_link"


class AlfredMountTf(StaticTfPublisher):
    """Publishes Alfred's urdf mount tree onto tf on a fixed interval."""

    config: AlfredMountTfConfig

    def transforms(self) -> list[Transform]:
        return mount_transforms(self.config.root_frame)
