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

import time
from typing import Any, Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
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
    planning_frame: str = "world"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_empty_closed_threshold: float = Field(default=0.01, ge=0.0)
    grasp_feedback_delay: float = Field(default=0.5, ge=0.0)


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
        self._selected_pregrasp: PoseStamped | None = None
        self._holding_object = False

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult[ManipulationSkillError]:
        """Scan the latest RGB-D frame for prompted objects."""
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            return SkillResult.fail("INVALID_INPUT", "At least one object prompt is required")
        try:
            self._scene.set_prompts(prompts)
            detections = self._scene.scan_scene()
            objects = self._scene.get_detected_objects()
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        self._objects = {str(obj["object_id"]): obj for obj in objects if "object_id" in obj}
        if not self._holding_object:
            self._clear_selection()
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)",
            prompts=prompts,
            objects=list(self._objects.values()),
        )

    @rpc
    def get_object(self, object_id: str) -> dict[str, Any] | None:
        return self._objects.get(object_id)

    @skill
    def select_grasp(self, object_id: str, rank: int = 0) -> SkillResult[ManipulationSkillError]:
        """Select a provider proposal for one scanned object."""
        self._clear_selection()
        if rank < 0:
            return SkillResult.fail("INVALID_INPUT", "Grasp rank must be non-negative")
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
        if candidates.header.frame_id != self.config.planning_frame:
            return SkillResult.fail(
                "GRASP_FRAME_MISMATCH",
                f"Expected {self.config.planning_frame}, got {candidates.header.frame_id}",
            )
        self._grasp_candidates = candidates
        if rank >= len(candidates.candidates):
            return SkillResult.fail("GRASP_GENERATION_FAILED", f"Grasp rank {rank} is unavailable")
        candidate = candidates.candidates[rank]
        self._selected_object_id = object_id
        self._selected_grasp = PoseStamped(
            ts=candidates.header.timestamp,
            frame_id=candidates.header.frame_id,
            position=candidate.pose.position,
            orientation=candidate.pose.orientation,
        )
        self._selected_pregrasp = self._offset_pose(
            self._selected_grasp, self.config.pregrasp_offset
        )
        return SkillResult.ok(
            "Grasp selected",
            object_id=object_id,
            rank=rank,
            score=candidate.score,
            candidates=len(candidates.candidates),
        )

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        return self._grasp_candidates

    @skill
    def pick_selected(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Pick the object selected by select_grasp."""
        grasp = self._selected_grasp
        if grasp is None:
            return SkillResult.fail("INVALID_STATE", "Select a grasp before starting a pick")
        group = self._resolve_group(planning_group)
        if group is None:
            return SkillResult.fail(
                "ROBOT_NOT_FOUND", "Gripper-capable planning group is missing or ambiguous"
            )
        grasp = self._apply_yaw_policy(grasp, group)
        pregrasp = self._offset_pose(grasp, self.config.pregrasp_offset)
        self._selected_grasp = grasp
        self._selected_pregrasp = pregrasp
        if failure := self._set_gripper(1.0, group):
            return failure
        if failure := self._move(pregrasp, group):
            return failure
        if failure := self._move(grasp, group):
            return failure
        if failure := self._set_gripper(0.0, group):
            return failure
        time.sleep(self.config.grasp_feedback_delay)
        gripper_position = self._gripper_position(group)
        if gripper_position is None:
            return SkillResult.fail("GRIPPER_FAILED", "Gripper feedback is unavailable")
        if gripper_position <= self.config.grasp_empty_closed_threshold:
            self._set_gripper(1.0, group)
            self._move(pregrasp, group)
            return SkillResult.fail("GRASP_VERIFICATION_FAILED", "Gripper reached empty-closed")
        if failure := self._move(pregrasp, group):
            return failure
        self._holding_object = True
        return SkillResult.ok("Pick complete", object_id=self._selected_object_id)

    @skill
    def place_at(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Place the held object at an explicit planning-frame position."""
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
        if failure := self._set_gripper(1.0, group):
            return failure
        self._holding_object = False
        return self._move(preplace, group) or SkillResult.ok("Place complete")

    def _clear_selection(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._selected_object_id = None
        self._selected_grasp = None
        self._selected_pregrasp = None

    def _resolve_group(self, planning_group: PlanningGroupID | None) -> PlanningGroupID | None:
        groups = [group for group in self._manipulation.list_planning_groups() if group.has_gripper]
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

    def _set_gripper(
        self, position: float, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        result = self._manipulation.set_gripper_position(position, planning_group)
        if not result.succeeded:
            return SkillResult.fail("GRIPPER_FAILED", result.message)
        return None

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None
