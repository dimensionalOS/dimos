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
    UNCONFIRMED_STOP,
    CommandResult,
    ExecutionResult,
    ManipulationSpec,
    PlanResult,
)
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def _status(result: PlanResult | ExecutionResult | CommandResult) -> str:
    """Status name and message of a result, e.g. 'FAILED: no path'."""
    return f"{result.status.name}: {result.message}" if result.message else result.status.name


class ManipulationSkills(Module):
    """Legacy LLM tools kept separate from the primitive RPC module."""

    manipulation: ManipulationSpec

    @staticmethod
    def _command_result(result: CommandResult) -> SkillResult:
        """Report an accepted command. Raises RuntimeError when it was not accepted."""
        if not result.succeeded:
            raise RuntimeError(_status(result))
        return SkillResult.ok(result.message)

    @staticmethod
    def _execution_result(result: ExecutionResult) -> SkillResult:
        """Report how running the plan ended."""
        if result.succeeded:
            return SkillResult.ok(str(result))
        return SkillResult.ok(f"Execution ended with {_status(result)}")

    @staticmethod
    def _planning_result(result: PlanResult) -> SkillResult | None:
        """None when a plan was found, else a statement of what the planner returned."""
        if result.succeeded:
            return None
        return SkillResult.ok(f"No plan; planner returned {_status(result)}")

    def _select_group(
        self,
        planning_group: PlanningGroupID | None,
        *,
        pose_capable: bool = False,
    ) -> PlanningGroupID:
        """Pick the planning group to command.

        Args:
            planning_group: Group ID to use, or None to use the only candidate.
            pose_capable: When True and ``planning_group`` is None, only groups
                with a tool frame are candidates.

        Raises ValueError when the ID is unknown, or when it is omitted and
        there is not exactly one candidate.
        """
        groups = self.manipulation.list_planning_groups()
        if planning_group is not None:
            if any(group.id == planning_group for group in groups):
                return planning_group
            raise ValueError(
                f"Unknown planning group {planning_group!r}; groups: {[g.id for g in groups]}"
            )
        candidates = [
            group.id for group in groups if not pose_capable or group.tip_frame is not None
        ]
        if len(candidates) != 1:
            kind = "pose-capable planning group" if pose_capable else "planning group"
            raise ValueError(
                f"Expected exactly one {kind} when planning_group is omitted; found {candidates}"
            )
        return candidates[0]

    def _move_to_preset(
        self,
        preset: str,
        planning_group: PlanningGroupID | None,
    ) -> SkillResult:
        group_id = self._select_group(planning_group)
        state = self.manipulation.get_state().groups[group_id]
        target = state.joint_presets.get(preset)
        if target is None:
            raise RuntimeError(
                f"Planning group {group_id!r} has no {preset!r} joint preset; "
                f"presets: {sorted(state.joint_presets)}"
            )
        plan = self.manipulation.plan_to_joints({group_id: target})
        if no_plan := self._planning_result(plan):
            return no_plan
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill
    def cancel(self) -> SkillResult:
        """Stop the active motion or planning attempt, leaving the arm where it is."""
        result = self.manipulation.cancel()
        if result.status in UNCONFIRMED_STOP:
            return SkillResult.ok(
                f"Requested a stop; the arm's stop was not confirmed ({_status(result)})"
            )
        return SkillResult.ok(result.message or "Cancelled")

    @skill
    def reset(self) -> SkillResult:
        """Stop any motion and return to IDLE. Use after a motion fails."""
        result = self.manipulation.reset()
        if not result.succeeded:
            return SkillResult.ok(f"Did not reset; module returned {_status(result)}")
        return SkillResult.ok(result.message)

    @skill
    def get_robot_state(self, planning_group: PlanningGroupID | None = None) -> SkillResult:
        """Get manipulation state, optionally for one planning group.

        Args:
            planning_group: Opaque planning-group ID. Omit to return every group.
        """

        snapshot = self.manipulation.get_state()
        if planning_group is None:
            return SkillResult.ok(repr(snapshot))
        state = snapshot.groups.get(planning_group)
        if state is None:
            raise ValueError(
                f"Unknown planning group {planning_group!r}; groups: {sorted(snapshot.groups)}"
            )
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
    ) -> SkillResult:
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
        current = self.manipulation.get_state().groups[group_id].end_effector_pose
        if current is None:
            raise RuntimeError(f"End-effector pose for planning group {group_id!r} is unavailable")
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
        if no_plan := self._planning_result(plan):
            return no_plan
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill(uses=[CAP_MOVEMENT])
    def move_to_joints(
        self,
        joints: str,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult:
        """Move one planning group to comma-separated joint positions.

        Args:
            joints: Comma-separated positions in each joint's native coordinate.
            planning_group: Opaque planning-group ID. Omit when only one group exists.
        """

        group_id = self._select_group(planning_group)
        try:
            values = [float(value.strip()) for value in joints.split(",")]
        except ValueError as exc:
            raise ValueError(f"joints must be comma-separated floats, got {joints!r}") from exc
        group = next(
            group for group in self.manipulation.list_planning_groups() if group.id == group_id
        )
        if len(values) != len(group.joint_names):
            raise ValueError(
                f"Expected {len(group.joint_names)} joint values for {group_id!r}, "
                f"got {len(values)}"
            )
        plan = self.manipulation.plan_to_joints(
            {group_id: JointState(name=list(group.joint_names), position=values)}
        )
        if no_plan := self._planning_result(plan):
            return no_plan
        return self._execution_result(self.manipulation.execute(blocking=True))

    @skill(uses=[CAP_MOVEMENT])
    def go_home(self, planning_group: PlanningGroupID | None = None) -> SkillResult:
        """Open the gripper and move to the configured home preset.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one group exists.
        """

        self.manipulation.set_gripper_position(1.0, planning_group)
        return self._move_to_preset("home", planning_group)

    @skill(uses=[CAP_MOVEMENT])
    def go_init(self, planning_group: PlanningGroupID | None = None) -> SkillResult:
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
    ) -> SkillResult:
        """Set the gripper opening as a fraction of its travel.

        Args:
            position: 0.0 = fully closed, 1.0 = fully open.
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(
            self.manipulation.set_gripper_position(position, planning_group)
        )

    @skill(uses=[CAP_MOVEMENT])
    def open_gripper(self, planning_group: PlanningGroupID | None = None) -> SkillResult:
        """Open the gripper fully.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(self.manipulation.set_gripper_position(1.0, planning_group))

    @skill(uses=[CAP_MOVEMENT])
    def close_gripper(self, planning_group: PlanningGroupID | None = None) -> SkillResult:
        """Close the gripper fully.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one gripper exists.
        """

        return self._command_result(self.manipulation.set_gripper_position(0.0, planning_group))
