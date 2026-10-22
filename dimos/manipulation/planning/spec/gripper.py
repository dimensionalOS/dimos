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

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.protocol.service.spec import BaseConfig

_Vector3 = tuple[float, float, float]


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


class GripperConfig(BaseConfig):
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
                "GripperConfig open_joints and closed_joints must name the same joints, got "
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
