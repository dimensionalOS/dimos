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

"""Robot-independent, capability-composed pick-and-place workflow."""

from __future__ import annotations

import threading
from typing import Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.candidate_filter_spec import GraspCandidateFilterSpec
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.pick_execution_spec import PickExecutionSpec, PickMotionSpec
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.perception.experimental.object import Object as DetObject
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PickAndPlaceModuleConfig(ModuleConfig):
    """Configuration for the capability-composed pick-and-place workflow."""

    # Standoff along the approach axis. Omit to use the robot model's
    # pre_grasp_offset, which is the calibrated value for that arm.
    pregrasp_offset: float | None = Field(default=None, gt=0.0)
    candidate_filter: Literal["off", "ik_collision"] = "ik_collision"
    candidate_ranking: Literal["confidence", "ik_feasibility"] = "confidence"
    candidate_ik_limit: int = Field(default=10, gt=0)
    scan_timeout: float = Field(default=5.0, gt=0.0)


class PickAndPlaceModule(Module):
    """Scan objects, select grasp proposals, and execute picks through capabilities.

    A skill adapter, not a planning module: every primitive it needs arrives
    through a Spec, so it carries no robot model and no collision world of its
    own. It deliberately does not estimate tabletops, install scene geometry,
    or publish visualization.
    """

    config: PickAndPlaceModuleConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec
    _grasp_filter: GraspCandidateFilterSpec
    _motion: PickMotionSpec
    _execution: PickExecutionSpec
    objects: In[list[DetObject]]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._objects_condition = threading.Condition()
        self._objects: tuple[DetObject, ...] = ()
        self._objects_version = 0
        self._grasp_candidates = GraspCandidateArray()
        self._selected_object_id: str | None = None
        self._selected_grasp: PoseStamped | None = None
        self._selected_pregrasp: PoseStamped | None = None
        self._holding_object = False

    @rpc
    def start(self) -> None:
        super().start()
        self.objects.subscribe(self._on_objects)

    def _on_objects(self, objects: list[DetObject]) -> None:
        with self._objects_condition:
            self._objects = tuple(objects)
            self._objects_version += 1
            self._objects_condition.notify_all()

    def _pregrasp_offset(self) -> float:
        if self.config.pregrasp_offset is not None:
            return self.config.pregrasp_offset
        return self._execution.get_pre_grasp_offset()

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult[ManipulationSkillError]:
        """Scan for objects matching the supplied simple noun-phrase prompts.

        Args:
            prompts: Object names to detect, one simple noun phrase each.
        """
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            return SkillResult.fail("INVALID_INPUT", "At least one object prompt is required")
        with self._objects_condition:
            objects_version = self._objects_version
        self._scene.set_prompts(prompts)
        with self._objects_condition:
            fresh = self._objects_condition.wait_for(
                lambda: self._objects_version > objects_version,
                timeout=self.config.scan_timeout,
            )
            objects = self._objects
        if not fresh:
            return SkillResult.fail(
                "OBJECT_NOT_DETECTED",
                f"No detections published within {self.config.scan_timeout:.1f}s of the scan",
            )
        return SkillResult.ok(
            f"Detected {len(objects)} object(s)",
            prompts=prompts,
            objects=[
                {"object_id": obj.object_id, "name": obj.name, "confidence": obj.confidence}
                for obj in objects
            ],
        )

    @rpc
    def get_object(self, object_id: str) -> DetObject | None:
        """Return an object from the most recent scan by its stable object ID."""
        with self._objects_condition:
            return next((obj for obj in self._objects if obj.object_id == object_id), None)

    @skill
    def select_grasp(self, object_id: str, rank: int = 0) -> SkillResult[ManipulationSkillError]:
        """Generate ranked grasp proposals for an object and select one without moving.

        Args:
            object_id: Stable object ID from the most recent scan.
            rank: Which ranked proposal to select, 0 being the best.
        """
        if rank < 0:
            return SkillResult.fail("INVALID_INPUT", "rank must not be negative")
        obj = self.get_object(object_id)
        if obj is None:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"No object with ID {object_id!r}")
        try:
            candidates = self._grasp_generator.propose_grasps(obj.pointcloud)
        except ValueError as exc:
            return SkillResult.fail("GRASP_INPUT_INVALID", str(exc))
        candidates = self._filter_candidates(candidates)
        self._grasp_candidates = candidates
        if rank >= len(candidates.candidates):
            self._clear_selection()
            return SkillResult.fail(
                "GRASP_GENERATION_FAILED",
                f"Only {len(candidates.candidates)} reachable proposal(s) for {obj.name!r}",
            )
        candidate = candidates.candidates[rank]
        self._selected_object_id = object_id
        self._selected_grasp = PoseStamped(
            ts=candidates.header.timestamp,
            frame_id=candidates.header.frame_id,
            position=candidate.pose.position,
            orientation=candidate.pose.orientation,
        )
        offset = self._selected_grasp.orientation.rotate_vector(
            Vector3(0.0, 0.0, -self._pregrasp_offset())
        )
        self._selected_pregrasp = PoseStamped(
            ts=self._selected_grasp.ts,
            frame_id=self._selected_grasp.frame_id,
            position=self._selected_grasp.position + offset,
            orientation=self._selected_grasp.orientation,
        )
        position = self._selected_grasp.position
        return SkillResult.ok(
            f"Selected grasp {rank} of {len(candidates.candidates)} for {obj.name!r} at "
            f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})",
            object_id=object_id,
        )

    def _clear_selection(self) -> None:
        self._selected_object_id = None
        self._selected_grasp = None
        self._selected_pregrasp = None

    def _filter_candidates(self, candidates: GraspCandidateArray) -> GraspCandidateArray:
        ranked = sorted(candidates.candidates, key=lambda candidate: -candidate.score)
        if self.config.candidate_filter == "off" and self.config.candidate_ranking == "confidence":
            return GraspCandidateArray(candidates.header, ranked)
        candidates_to_evaluate = ranked[: self.config.candidate_ik_limit]
        evaluated = [
            (
                candidate,
                self._grasp_filter.inverse_kinematics_single(
                    candidate.pose, check_collision=True
                ).is_success(),
            )
            for candidate in candidates_to_evaluate
        ]
        if self.config.candidate_filter == "ik_collision":
            evaluated = [(candidate, feasible) for candidate, feasible in evaluated if feasible]
        else:
            evaluated.extend(
                (candidate, False) for candidate in ranked[len(candidates_to_evaluate) :]
            )
        if self.config.candidate_ranking == "ik_feasibility":
            evaluated.sort(key=lambda item: (not item[1], -item[0].score))
        else:
            evaluated.sort(key=lambda item: -item[0].score)
        return GraspCandidateArray(candidates.header, [candidate for candidate, _ in evaluated])

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        """Return proposals from the most recent ``select_grasp`` call."""
        return self._grasp_candidates

    @skill(uses=[CAP_MOVEMENT])
    def pick_selected(
        self, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Pick the object selected by ``select_grasp``.

        Args:
            planning_group: Opaque planning-group ID. Omit when only one arm exists.
        """
        grasp = self._selected_grasp
        pregrasp = self._selected_pregrasp
        if grasp is None or pregrasp is None:
            return SkillResult.fail("INVALID_STATE", "Select a grasp before starting a pick")
        lifted = self._execution.lift_if_low(planning_group=planning_group)
        if not lifted.succeeded:
            return SkillResult.fail("PLANNING_FAILED", lifted.message)
        opened = self._execution.open_gripper_and_settle(planning_group)
        if not opened.succeeded:
            return SkillResult.fail("GRIPPER_FAILED", f"pre-grasp open: {opened.message}")
        approach = self._move(pregrasp, planning_group)
        if not approach.is_success():
            return approach
        contact = self._move(grasp, planning_group)
        if not contact.is_success():
            return contact
        grasped = self._execution.close_gripper_and_verify(planning_group)
        if not grasped.succeeded:
            self._recover_from_failed_grasp(pregrasp, planning_group)
            return SkillResult.fail("GRASP_VERIFICATION_FAILED", grasped.message)
        retreat = self._move(pregrasp, planning_group)
        if not retreat.is_success():
            return retreat
        self._holding_object = True
        return SkillResult.ok(
            f"Pick complete — {grasped.message}", object_id=self._selected_object_id
        )

    def _recover_from_failed_grasp(
        self, pregrasp: PoseStamped, planning_group: PlanningGroupID | None
    ) -> None:
        """Release and back off so a failed pick leaves the arm clear of the object."""
        released = self._execution.open_gripper_and_settle(planning_group)
        if not released.succeeded:
            logger.error(f"Could not reopen after a failed grasp: {released.message}")
        retreat = self._move(pregrasp, planning_group)
        if not retreat.is_success():
            logger.error(f"Could not retreat after a failed grasp: {retreat.message}")

    @skill(uses=[CAP_MOVEMENT])
    def place_at(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Place the held object at an explicit position in the planning frame.

        Args:
            x: Target X position in metres.
            y: Target Y position in metres.
            z: Target Z position in metres.
            planning_group: Opaque planning-group ID. Omit when only one arm exists.
        """
        grasp = self._selected_grasp
        if grasp is None or not self._holding_object:
            return SkillResult.fail("INVALID_STATE", "Pick an object before placing")
        euler = grasp.orientation.to_euler()
        standoff = z + self._pregrasp_offset()
        approach = self._motion.move_to_pose(
            x, y, standoff, euler.x, euler.y, euler.z, planning_group
        )
        if not approach.is_success():
            return approach
        placed = self._motion.move_to_pose(x, y, z, euler.x, euler.y, euler.z, planning_group)
        if not placed.is_success():
            return placed
        released = self._execution.open_gripper_and_settle(planning_group)
        if not released.succeeded:
            return SkillResult.fail("GRIPPER_FAILED", f"release: {released.message}")
        retreat = self._motion.move_to_pose(
            x, y, standoff, euler.x, euler.y, euler.z, planning_group
        )
        if not retreat.is_success():
            return retreat
        self._holding_object = False
        return SkillResult.ok(f"Place complete at ({x:.3f}, {y:.3f}, {z:.3f})")

    def _move(
        self, pose: PoseStamped, planning_group: PlanningGroupID | None
    ) -> SkillResult[ManipulationSkillError]:
        euler = pose.orientation.to_euler()
        return self._motion.move_to_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            euler.x,
            euler.y,
            euler.z,
            planning_group,
        )
