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

"""Universal agent-facing manipulation primitive facade."""

from __future__ import annotations

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.manipulation.agentic_manipulation_spec import ManipulationControlSpec
from dimos.manipulation.skill_errors import ManipulationSkillError


class AgenticManipulationModule(Module):
    """Expose stable manipulation primitives for agent/tool callers."""

    _manipulation: ManipulationControlSpec

    @skill
    def get_robot_state(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Get current robot state for manipulation.

        Args:
            robot_name: Robot to query (only needed for multi-arm setups).
        """
        return self._manipulation.get_robot_state(robot_name)

    @skill
    def move_to_joints(
        self, joints: str, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to a target joint configuration.

        Args:
            joints: Comma-separated joint positions in radians, e.g. "0.1, -0.5, 1.2, 0.0, 0.3, -0.1".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        return self._manipulation.move_to_joints(joints, robot_name)

    @skill
    def set_motion_speed(self, speed_scale: float) -> SkillResult[ManipulationSkillError]:
        """Set runtime manipulation motion speed for future motions.

        Args:
            speed_scale: Speed multiplier in the range `(0, 1]`. Use values below
                1.0 for slower, gentler motion.
        """
        if not self._manipulation.set_motion_speed(speed_scale):
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                "Motion speed scale must be greater than 0 and less than or equal to 1.",
            )
        return SkillResult[ManipulationSkillError].ok(
            f"Motion speed scale set to {speed_scale:.2f}x. Re-plan to apply it."
        )

    @skill
    def get_motion_speed(self) -> SkillResult[ManipulationSkillError]:
        """Get the current runtime manipulation motion speed scale."""
        speed_scale = self._manipulation.get_motion_speed()
        return SkillResult[ManipulationSkillError].ok(
            f"Current motion speed scale is {speed_scale:.2f}x.", speed_scale=speed_scale
        )

    @skill
    def open_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Open the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        return self._manipulation.open_gripper(robot_name)

    @skill
    def close_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Close the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        return self._manipulation.close_gripper(robot_name)


agentic_manipulation = AgenticManipulationModule.blueprint
