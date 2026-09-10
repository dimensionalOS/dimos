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

"""Capability-composed pick-and-place workflow."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasp_verification import (
    GraspVerificationConfig,
    GripperSettle,
    await_gripper_settle,
    grasp_failure,
    open_failure,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)


class PickAndPlaceModule(Module):
    """Coordinate scene registration, grasp generation, and manipulation execution."""

    config: PickAndPlaceModuleConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec
    _manipulation: ManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, dict[str, Any]] = {}
        self._grasp_candidates = GraspCandidateArray()
        self._selected_object_id: str | None = None
        self._selected_grasp: PoseStamped | None = None
        self._holding_object = False

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult[ManipulationSkillError]:
        """Scan the latest RGB-D frame for prompted objects.

        Args:
            prompts: Object labels to detect. Use an ID from this scan with pick_object.
        """
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            return SkillResult.fail("INVALID_INPUT", "At least one object prompt is required")
        if not self._holding_object:
            self._clear_selection()
        self._objects = {}
        try:
            detections = self._scene.scan_scene(text=prompts)
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        objects = [
            {
                "object_id": str(detection.id),
                "name": str(detection.results[0].hypothesis.class_id),
            }
            for detection in detections.detections
            if detection.id and detection.results
        ]
        self._objects = {str(obj["object_id"]): obj for obj in objects if "object_id" in obj}
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)",
            prompts=prompts,
            objects=list(self._objects.values()),
        )

    @rpc
    def get_object(self, object_id: str) -> dict[str, Any] | None:
        return self._objects.get(object_id)

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Generate ranked grasps and pick one object from the latest scan.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._holding_object:
            return SkillResult.fail(
                "INVALID_STATE", "Place the held object before starting another pick"
            )
        self._clear_selection()
        if object_id not in self._objects:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"Unknown object_id: {object_id}")
        try:
            pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
            if pointcloud is None:
                return SkillResult.fail(
                    "OBJECT_NOT_DETECTED", f"No pointcloud for object_id: {object_id}"
                )
            candidates = self._grasp_generator.propose_grasps(pointcloud)
        except (RuntimeError, ValueError) as exc:
            return SkillResult.fail("GRASP_GENERATION_FAILED", str(exc))
        self._grasp_candidates = candidates
        if candidates.header.frame_id != self.config.planning_frame:
            return SkillResult.fail(
                "GRASP_FRAME_MISMATCH",
                f"Expected {self.config.planning_frame}, got {candidates.header.frame_id}",
            )
        if not candidates.candidates:
            return SkillResult.fail("GRASP_GENERATION_FAILED", "No grasp candidates generated")
        group = self._resolve_group(planning_group)
        if group is None:
            return SkillResult.fail(
                "ROBOT_NOT_FOUND", "Gripper-capable planning group is missing or ambiguous"
            )
        candidate = candidates.candidates[0]
        grasp = self._apply_yaw_policy(
            PoseStamped(
                ts=candidates.header.timestamp,
                frame_id=candidates.header.frame_id,
                position=candidate.pose.position,
                orientation=candidate.pose.orientation,
            ),
            group,
        )
        pregrasp = self._offset_pose(grasp, self.config.pregrasp_offset)
        if failure := self._open_gripper(group, "pre-grasp open"):
            return failure
        if failure := self._move(pregrasp, group):
            return failure
        if failure := self._move(grasp, group):
            return failure
        if failure := self._close_and_verify(group):
            return failure

        self._selected_object_id = object_id
        self._selected_grasp = grasp
        self._holding_object = True
        if failure := self._move(pregrasp, group):
            return failure
        return SkillResult.ok(
            "Pick complete",
            object_id=object_id,
            rank=0,
            score=candidate.score,
            candidates=len(candidates.candidates),
        )

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        return self._grasp_candidates

    @skill(uses=[CAP_MOVEMENT])
    def place_at(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Place the held object at an explicit planning-frame position.

        Args:
            x: Planning-frame X coordinate in meters.
            y: Planning-frame Y coordinate in meters.
            z: Planning-frame Z coordinate in meters.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._selected_grasp is None or not self._holding_object:
            return SkillResult.fail("INVALID_STATE", "Pick an object before placing")
        group = self._resolve_group(planning_group)
        if group is None:
            return SkillResult.fail(
                "ROBOT_NOT_FOUND", "Gripper-capable planning group is missing or ambiguous"
            )
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=self._selected_grasp.orientation,
        )
        preplace = self._offset_pose(place, self.config.pregrasp_offset)
        if failure := self._move(preplace, group):
            return failure
        if failure := self._move(place, group):
            return failure
        if failure := self._open_gripper(group, "release"):
            return failure
        self._holding_object = False
        self._clear_selection()
        return self._move(preplace, group) or SkillResult.ok("Place complete")

    def _clear_selection(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._selected_object_id = None
        self._selected_grasp = None

    def _resolve_group(self, planning_group: PlanningGroupID | None) -> PlanningGroupID | None:
        groups = [
            group
            for group in self._manipulation.list_planning_groups()
            if group.has_gripper and group.tip_frame is not None
        ]
        if planning_group is not None:
            return planning_group if any(group.id == planning_group for group in groups) else None
        return groups[0].id if len(groups) == 1 else None

    def _apply_yaw_policy(self, pose: PoseStamped, group: PlanningGroupID) -> PoseStamped:
        if self.config.yaw_policy == "generated":
            return pose
        current = self._manipulation.get_state().groups[group].end_effector_pose
        if current is None:
            return pose
        euler = pose.orientation.to_euler()
        current_euler = current.orientation.to_euler()
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position,
            orientation=Quaternion.from_euler(Vector3(euler.x, euler.y, current_euler.z)),
        )

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + pose.orientation.rotate_vector(Vector3(0.0, 0.0, -offset)),
            orientation=pose.orientation,
        )

    def _move(
        self, pose: PoseStamped, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        plan = self._manipulation.plan_to_poses({planning_group: pose})
        if not plan.succeeded:
            return SkillResult.fail("PLANNING_FAILED", plan.message)
        execution = self._manipulation.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", execution.message)
        return None

    def _command_and_settle(
        self,
        position: float,
        planning_group: PlanningGroupID,
        arrival_tolerance: float | None = None,
    ) -> GripperSettle | SkillResult[ManipulationSkillError]:
        result = self._manipulation.set_gripper_position(position, planning_group)
        if not result.succeeded:
            return SkillResult.fail(
                "GRIPPER_FAILED", result.message or "Gripper command was rejected"
            )
        return await_gripper_settle(
            lambda: self._gripper_position(planning_group),
            position,
            self.config.grasp_verification,
            arrival_tolerance=arrival_tolerance,
        )

    def _open_gripper(
        self, planning_group: PlanningGroupID, step: str
    ) -> SkillResult[ManipulationSkillError] | None:
        # Jaws resting against the open stop never move and never reach the
        # commanded extreme; open_tolerance is the band that already decides
        # whether where they stopped counts as open.
        settle = self._command_and_settle(
            self.config.grasp_verification.open_position,
            planning_group,
            arrival_tolerance=self.config.grasp_verification.open_tolerance,
        )
        if isinstance(settle, SkillResult):
            return settle
        if settle.position is None:
            return None
        if failure := open_failure(settle, self.config.grasp_verification):
            return SkillResult.fail("GRIPPER_FAILED", f"{step}: {failure}")
        return None

    def _close_and_verify(
        self, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        settle = self._command_and_settle(
            self.config.grasp_verification.closed_position, planning_group
        )
        if isinstance(settle, SkillResult):
            return settle
        if not self.config.grasp_verification.enabled:
            return None
        if failure := grasp_failure(settle, self.config.grasp_verification):
            if settle.position is not None and "nothing in the jaws" in failure:
                recovered = self._open_gripper(planning_group, "empty-grasp recovery")
                if recovered:
                    return recovered
            return SkillResult.fail("GRASP_VERIFICATION_FAILED", failure)
        return None

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None
