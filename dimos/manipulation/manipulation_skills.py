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

"""Deprecated skill wrappers over :class:`ManipulationSpec`."""

from __future__ import annotations

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ManipulationSpec,
    PlanResult,
)
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


class ManipulationSkills(Module):
    """Legacy LLM tools kept separate from the primitive RPC module."""

    manipulation: ManipulationSpec

    @staticmethod
    def _command_result(result: CommandResult) -> SkillResult[ManipulationSkillError]:
        if result.succeeded:
            return SkillResult.ok(result.message)
        return SkillResult.fail("GRIPPER_FAILED", result.message)

    @staticmethod
    def _execution_result(result: ExecutionResult) -> SkillResult[ManipulationSkillError]:
        if result.succeeded:
            return SkillResult.ok(str(result))
        if result.status.name == "TIMED_OUT":
            return SkillResult.fail("EXECUTION_TIMEOUT", result.message or result.status.name)
        return SkillResult.fail("EXECUTION_FAILED", result.message or result.status.name)

    @staticmethod
    def _planning_result(result: PlanResult) -> SkillResult[ManipulationSkillError] | None:
        if result.succeeded:
            return None
        return SkillResult.fail("PLANNING_FAILED", result.message or result.status.name)

    def _select_group(
        self,
        planning_group: PlanningGroupID | None,
        *,
        pose_capable: bool = False,
    ) -> PlanningGroupID | None:
        groups = self.manipulation.list_planning_groups()
        if planning_group is not None:
            return planning_group if any(group.id == planning_group for group in groups) else None
        candidates = tuple(
            group for group in groups if not pose_capable or group.tip_frame is not None
        )
        return candidates[0].id if len(candidates) == 1 else None

    def _move_to_preset(
        self,
        preset: str,
        planning_group: PlanningGroupID | None,
    ) -> SkillResult[ManipulationSkillError]:
        group_id = self._select_group(planning_group)
        if group_id is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Planning group is missing or ambiguous")
        state = self.manipulation.get_state().groups[group_id]
        target = state.joint_presets.get(preset)
        if target is None:
            return SkillResult.fail("NOT_CONFIGURED", f"No '{preset}' joint preset configured")
        plan = self.manipulation.plan_to_joints({group_id: target})
        if failure := self._planning_result(plan):
            return failure
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill
    def get_robot_state(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Get manipulation state, optionally for one planning group.

        Args:
            planning_group: Opaque planning-group ID. Omit to return every group.
        """

        snapshot = self.manipulation.get_state()
        if planning_group is None:
            return SkillResult.ok(repr(snapshot))
        state = snapshot.groups.get(planning_group)
        if state is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", f"Unknown group: {planning_group}")
        return SkillResult.ok(f"{planning_group}: {state!r}")

    @skill(uses=[CAP_MOVEMENT])
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move an end effector to an absolute world-frame pose.

        Args:
            x: World-frame X position in metres.
            y: World-frame Y position in metres.
            z: World-frame Z position in metres.
            roll: Roll in radians. Omit to preserve the current value.
            pitch: Pitch in radians. Omit to preserve the current value.
            yaw: Yaw in radians. Omit to preserve the current value.
            planning_group: Opaque planning-group ID. Omit when only one arm exists.
        """

        group_id = self._select_group(planning_group, pose_capable=True)
        if group_id is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Pose-capable group is missing or ambiguous")
        current = self.manipulation.get_state().groups[group_id].end_effector_pose
        if current is None:
            return SkillResult.fail("INVALID_STATE", "End-effector pose is unavailable")
        if roll is None and pitch is None and yaw is None:
            orientation = current.orientation
        else:
            euler = current.orientation.to_euler()
            orientation = Quaternion.from_euler(
                Vector3(
                    euler.x if roll is None else roll,
                    euler.y if pitch is None else pitch,
                    euler.z if yaw is None else yaw,
                )
            )
        target = PoseStamped(
            frame_id="world",
            position=Vector3(x, y, z),
            orientation=orientation,
        )
        plan = self.manipulation.plan_to_poses({group_id: target})
        if failure := self._planning_result(plan):
            return failure
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill(uses=[CAP_MOVEMENT])
    def move_to_joints(
        self,
        joints: str,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move one planning group to comma-separated joint positions.

        Args:
            joints: Comma-separated joint positions in radians.
            planning_group: Opaque planning-group ID. Omit when only one group exists.
        """

        group_id = self._select_group(planning_group)
        if group_id is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Planning group is missing or ambiguous")
        try:
            values = [float(value.strip()) for value in joints.split(",")]
        except ValueError:
            return SkillResult.fail("INVALID_INPUT", "joints must be comma-separated floats")
        group = next(
            group for group in self.manipulation.list_planning_groups() if group.id == group_id
        )
        if len(values) != len(group.joint_names):
            return SkillResult.fail(
                "INVALID_INPUT",
                f"Expected {len(group.joint_names)} joint values, got {len(values)}",
            )
        plan = self.manipulation.plan_to_joints(
            {group_id: JointState(name=list(group.joint_names), position=values)}
        )
        if failure := self._planning_result(plan):
            return failure
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill(uses=[CAP_MOVEMENT])
    def go_home(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Open the gripper and move to the configured home preset.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one group exists.
        """

        self.manipulation.set_gripper_position(1.0, planning_group)
        return self._move_to_preset("home", planning_group)

    @skill(uses=[CAP_MOVEMENT])
    def go_init(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Move to the joint state captured at startup.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one group exists.
        """

        return self._move_to_preset("init", planning_group)

    @skill(uses=[CAP_MOVEMENT])
    def set_gripper(
        self,
        position: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Set the gripper opening as a fraction of its travel.

        Args:
            position: 0.0 = fully closed, 1.0 = fully open.
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(
            self.manipulation.set_gripper_position(position, planning_group)
        )

    @skill(uses=[CAP_MOVEMENT])
    def open_gripper(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Open the gripper fully.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(self.manipulation.set_gripper_position(1.0, planning_group))

    @skill(uses=[CAP_MOVEMENT])
    def close_gripper(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Close the gripper fully.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(self.manipulation.set_gripper_position(0.0, planning_group))
