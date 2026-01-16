# Copyright 2025-2026 Dimensional Inc.
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

"""Controller pose with conversion utilities for teleop output types."""

from __future__ import annotations

import time

from dimos.msgs.geometry_msgs.Pose import Pose, PoseConvertable
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class ControllerPose(Pose):
    """Controller pose with conversion utilities for teleop output types.

    Extends Pose with methods to convert to PoseStamped or Twist commands.
    """

    @classmethod
    def from_pose(cls, pose: Pose) -> ControllerPose:
        """Create ControllerPose from a Pose.

        Args:
            pose: Pose to convert.

        Returns:
            ControllerPose with same position and orientation.
        """
        return cls(position=pose.position, orientation=pose.orientation)

    def to_pose_stamped(
        self,
        initial_robot_pose: Pose | None = None,
        frame_id: str = "teleop_target",
    ) -> PoseStamped:
        """Convert to PoseStamped (target = initial + delta).

        Args:
            initial_robot_pose: Initial robot pose to add delta to.
                If None, delta is used directly as target.
            frame_id: Frame ID for the PoseStamped message.

        Returns:
            PoseStamped target pose.
        """
        if initial_robot_pose is not None:
            target_pose = initial_robot_pose + self
        else:
            target_pose = self

        return PoseStamped(
            ts=time.time(),
            frame_id=frame_id,
            position=target_pose.position,
            orientation=target_pose.orientation,
        )

    def to_twist(
        self,
        linear_scale: float = 1.0,
        angular_scale: float = 1.0,
        max_linear: float = 0.5,
        max_angular: float = 1.0,
    ) -> Twist:
        """Convert to Twist velocity command.

        Args:
            linear_scale: Scale factor for linear velocity.
            angular_scale: Scale factor for angular velocity.
            max_linear: Maximum linear velocity clamp.
            max_angular: Maximum angular velocity clamp.

        Returns:
            Twist velocity command.
        """

        def clamp(value: float, limit: float) -> float:
            return max(-limit, min(limit, value))

        linear = Vector3(
            clamp(self.position.x * linear_scale, max_linear),
            clamp(self.position.y * linear_scale, max_linear),
            clamp(self.position.z * linear_scale, max_linear),
        )

        euler = self.orientation.to_euler()
        angular = Vector3(
            clamp(euler.x * angular_scale, max_angular),
            clamp(euler.y * angular_scale, max_angular),
            clamp(euler.z * angular_scale, max_angular),
        )

        return Twist(linear=linear, angular=angular)

    def __sub__(self, other: Pose | PoseConvertable) -> ControllerPose:  # type: ignore[override]
        """Compute delta pose, returning ControllerPose instead of Pose."""
        result = super().__sub__(other)
        return ControllerPose.from_pose(result)
