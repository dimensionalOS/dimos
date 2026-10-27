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

"""Which gripper links may touch the object it grasps, and how its fingers close."""

from __future__ import annotations

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class GripperConfig(BaseConfig):
    """Which gripper links may touch the object, and how the fingers close.

    Args:
        pad_links: URDF links allowed to touch the object being grasped, typically
            the fingers that carry the pads. The whole link counts.
        finger_joints: Each finger joint's (open, closed) position, by URDF joint
            name, radians (metres for a sliding joint). They all close together;
            joints that mimic one of these in the URDF follow it.
        closing_steps: How many poses between open and closed to check. 0 checks
            the open pose only (a suction cup, say).
    """

    pad_links: tuple[str, ...] = Field(min_length=1)
    finger_joints: dict[str, tuple[float, float]] = Field(default_factory=dict)
    closing_steps: int = Field(default=4, ge=0)

    def joint_positions(self) -> list[dict[str, float]]:
        """Finger joint positions at the open pose, then at each closing step, last fully closed."""
        steps = self.closing_steps if self.finger_joints else 0
        return [
            {
                name: opened + (closed - opened) * step / max(steps, 1)
                for name, (opened, closed) in self.finger_joints.items()
            }
            for step in range(steps + 1)
        ]
