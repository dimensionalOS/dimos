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

"""The dual OpenYAM URDF for web viewers, with a see-through copy posed by the leader arms.

The served URDF holds a second set of the robot's links, named ``leader_<link>``
and in the URDF group ``leader``, which a viewer shows or hides apart from the
robot's own links. Only the served URDF has them: the planner's model is not
touched, and the follower's own tf is not changed.
"""

from __future__ import annotations

import asyncio
import copy
from dataclasses import replace
import time
import xml.etree.ElementTree as ET

import numpy as np

from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.urdf_manager import UrdfManager

LEADER_LINK_PREFIX = "leader_"
LEADER_GROUP = "leader"
# Every leader visual in one flat colour and mostly see-through (a viewer honours the URDF's alpha).
LEADER_RGBA = "0.2 0.6 1.0 0.4"
_MOUNT_JOINT = f"{LEADER_LINK_PREFIX}mount"


def _with_leader_copy(xml: str, root_link: str) -> str:
    """The URDF plus a prefixed copy of every link and joint, welded to the root link at identity."""
    robot = ET.fromstring(xml)
    copies: list[ET.Element] = []
    for element in robot:
        if element.tag not in ("link", "joint"):
            continue
        twin = copy.deepcopy(element)
        twin.set("name", LEADER_LINK_PREFIX + element.attrib["name"])
        if element.tag == "link":
            for child in list(twin):
                if child.tag in ("inertial", "collision"):
                    twin.remove(child)
            for visual in twin.iter("visual"):
                for material in visual.findall("material"):
                    visual.remove(material)
                material = ET.SubElement(visual, "material", name=f"{LEADER_LINK_PREFIX}tint")
                ET.SubElement(material, "color", rgba=LEADER_RGBA)
            ET.SubElement(twin, "group").text = LEADER_GROUP
        else:
            for tag in ("parent", "child"):
                end = twin.find(tag)
                assert end is not None
                end.set("link", LEADER_LINK_PREFIX + end.attrib["link"])
            for mimic in twin.findall("mimic"):
                twin.remove(mimic)
        copies.append(twin)
    mount = ET.Element("joint", name=_MOUNT_JOINT, type="fixed")
    ET.SubElement(mount, "parent", link=root_link)
    ET.SubElement(mount, "child", link=LEADER_LINK_PREFIX + root_link)
    robot.extend([*copies, mount])
    return ET.tostring(robot, encoding="unicode")


class DualOpenYamLeaderUrdfManager(UrdfManager):
    """UrdfManager that also serves, and poses on tf, a leader copy of the robot.

    The copy's joints come from ``leader_joint_command``. While following is
    paused the leaders send a command with no positions; it is ignored and the
    copy stays where it was.
    """

    leader_joint_command: In[JointState]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        description = self._description
        self._leader_joints = tuple(
            replace(
                joint,
                name=LEADER_LINK_PREFIX + joint.name,
                parent_link=LEADER_LINK_PREFIX + joint.parent_link,
                child_link=LEADER_LINK_PREFIX + joint.child_link,
                mimic=None,
            )
            for joint in description.joints
        )
        self._leader_source_names = tuple(joint.name for joint in description.joints)
        self._leader_root = LEADER_LINK_PREFIX + description.root_link
        self._leader_positions: dict[str, float] = {}
        self._next_leader_tf_t = 0.0
        # Only the served description gains the leader links; self._joints, which poses the robot, does not.
        self._description = replace(
            description, xml=_with_leader_copy(description.xml, description.root_link)
        )

    async def handle_leader_joint_command(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_leader_joint_state, state)

    def add_leader_joint_state(self, state: JointState) -> None:
        """Pose the leader copy at the command's joints; a command with no positions changes nothing."""
        if not state.position:
            return
        self._leader_positions.update(zip(state.name, state.position, strict=False))
        if time.monotonic() < self._next_leader_tf_t:
            return
        self._next_leader_tf_t = time.monotonic() + 1.0 / self.config.joint_tf_hz
        self.tfbuffer.publish(*self.leader_transforms(state.ts))

    def leader_transforms(self, ts: float) -> list[Transform]:
        """One parent -> child transform per leader link, at the last commanded joints."""
        mount = Transform.from_matrix(
            np.eye(4),
            frame_id=self._description.root_link,
            child_frame_id=self._leader_root,
            ts=ts,
        )
        return [
            mount,
            *[
                Transform.from_matrix(
                    joint.parent_from_child(self._leader_positions.get(source, 0.0)),
                    frame_id=joint.parent_link,
                    child_frame_id=joint.child_link,
                    ts=ts,
                )
                for joint, source in zip(
                    self._leader_joints, self._leader_source_names, strict=True
                )
            ],
        ]
