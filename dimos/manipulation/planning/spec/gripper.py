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

"""Where a gripper may touch the object it grasps, and how its fingers close."""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.protocol.service.spec import BaseConfig

_Vector3 = tuple[float, float, float]
# Name GraspContact.touches gives a grasp a depth search did not reach because
# enough grasps before it passed (``GripperContactModel.fit_depth``).
UNCHECKED = "unchecked"


class ContactPatch(BaseConfig):
    """A box on one gripper link where touching the grasped object is allowed.

    Typically a finger's inner pad. An object point near the link counts as an
    allowed contact when it lies inside this box.

    Args:
        link: URDF link the box is fixed to.
        center: Box centre in that link's frame, metres.
        size: Full edge lengths along the link's x, y and z axes, metres. Make the
            box reach past the pad's face, toward the object, by more than the
            contact margin the check uses, or a pad that only grazes the object
            counts as a contact elsewhere.
    """

    link: str = Field(min_length=1)
    center: _Vector3
    size: _Vector3

    @model_validator(mode="after")
    def _positive_size(self) -> Self:
        if any(edge <= 0.0 for edge in self.size):
            raise ValueError(f"ContactPatch on {self.link} needs a positive size, got {self.size}")
        return self


class GripperSpec(BaseConfig):
    """Which gripper surfaces may touch the object, and the finger positions of a close.

    Args:
        contact_patches: Surfaces allowed to touch the object being grasped.
        open_joints: Finger joint positions with the gripper open, by URDF joint
            name, radians (metres for a sliding joint). Joints that mimic one of
            these in the URDF follow it.
        closed_joints: The same joints fully closed. Must name the same joints.
        closing_steps: How many poses between open and closed to check. 0 checks
            the open pose only (a suction cup, say).
        base_link: The link the gripper hangs from; it and every link below it are
            the gripper. None uses the nearest link that holds the tool tip and
            every contact patch.
    """

    contact_patches: tuple[ContactPatch, ...] = Field(min_length=1)
    open_joints: dict[str, float] = Field(default_factory=dict)
    closed_joints: dict[str, float] = Field(default_factory=dict)
    closing_steps: int = Field(default=4, ge=0)
    base_link: str | None = None

    @model_validator(mode="after")
    def _same_joints(self) -> Self:
        if set(self.open_joints) != set(self.closed_joints):
            raise ValueError(
                "GripperSpec open_joints and closed_joints must name the same joints, got "
                f"{sorted(self.open_joints)} and {sorted(self.closed_joints)}"
            )
        return self

    def joint_positions(self) -> list[dict[str, float]]:
        """Finger joint positions at the open pose, then at each closing step, last fully closed."""
        steps = self.closing_steps if self.open_joints else 0
        if steps == 0:
            return [dict(self.open_joints)]
        return [
            {
                name: start + (self.closed_joints[name] - start) * step / steps
                for name, start in self.open_joints.items()
            }
            for step in range(steps + 1)
        ]


@dataclass(frozen=True)
class GraspContact:
    """What one grasp candidate's gripper would touch that it should not.

    Args:
        touches: "target" when a part other than a contact patch comes near the
            object being grasped, then the name of each obstacle point set a part
            overlaps. Empty when the grasp is clear.
        while_closing: True when every such touch comes only as the fingers
            close, not at the open pose.
        empty_gap: True when too few of the object's points lie in the gap the
            finger pads close across, so the gripper would close on nothing.
        depth_shift: How far (metres) the grasp must move along its approach to
            get this verdict; positive is deeper, toward the object. Only a depth
            search (``GripperContactModel.fit_depth``) sets it.
    """

    touches: tuple[str, ...] = ()
    while_closing: bool = False
    empty_gap: bool = False
    depth_shift: float = 0.0

    @property
    def clear(self) -> bool:
        return not self.touches and not self.empty_gap


class ApproachOffsets(NamedTuple):
    """Where a gripper's finger pads, fingertips and palm sit along its approach.

    The approach is the tool tip's z axis, pointing from the gripper toward the object.

    Args:
        pad_behind_tcp: How far (metres) the contact patches' centre sits behind
            the tool tip with the jaws fully closed.
        fingertip_reach: The farthest (metres) any gripper part reaches past the
            tool tip while the jaws close; negative when it stays behind.
        palm_behind_tcp: How far (metres) the nearest part other than the links
            holding contact patches (the palm, a knuckle) sits behind the tool
            tip with the jaws open.
    """

    pad_behind_tcp: float
    fingertip_reach: float
    palm_behind_tcp: float
