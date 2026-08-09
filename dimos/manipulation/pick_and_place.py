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
import time
from typing import Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT, CAP_PERCEPTION
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.candidate_filter_spec import GraspCandidateFilterSpec
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, HeuristicGraspSpec
from dimos.manipulation.pick_execution_spec import PickExecutionSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.perception.experimental.object import Object
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec


class PickAndPlaceModuleConfig(ModuleConfig):
    """Configuration for the capability-composed pick-and-place workflow."""

    grasp: Literal["heuristic", "graspgenx"] = "heuristic"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    candidate_filter: Literal["off", "ik_collision"] = "ik_collision"
    candidate_ranking: Literal["confidence", "ik_feasibility"] = "confidence"
    candidate_ik_limit: int = Field(default=10, gt=0)
    grasp_empty_closed_threshold: float = Field(default=0.01, ge=0.0)
    grasp_feedback_delay: float = Field(default=0.5, ge=0.0)


class PickAndPlaceModule(Module):
    """Scan objects, select grasp proposals, and execute picks through capabilities."""

    config: PickAndPlaceModuleConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec | None
    _heuristic_grasp_generator: HeuristicGraspSpec | None
    _grasp_filter: GraspCandidateFilterSpec
    _pick_execution: PickExecutionSpec
    objects: In[list[Object]]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._objects_condition = threading.Condition()
        self._objects: tuple[Object, ...] = ()
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

    def _on_objects(self, objects: list[Object]) -> None:
        with self._objects_condition:
            self._objects = tuple(objects)
            self._objects_version += 1
            self._objects_condition.notify_all()

    @skill(uses=[CAP_PERCEPTION])
    def scan_objects(self, prompts: list[str]) -> SkillResult:
        """Scan for objects matching the supplied simple noun-phrase prompts."""
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            return SkillResult.fail("INVALID_INPUT", "At least one object prompt is required")
        try:
            with self._objects_condition:
                objects_version = self._objects_version
            self._scene.set_prompts(prompts)
            detections = self._scene.scan_scene()
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        with self._objects_condition:
            self._objects_condition.wait_for(
                lambda: self._objects_version > objects_version,
                timeout=5.0,
            )
            objects = self._objects
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)",
            prompts=prompts,
            objects=[
                {"object_id": obj.object_id, "name": obj.name, "confidence": obj.confidence}
                for obj in objects
            ],
        )

    @rpc
    def get_object(self, object_id: str) -> Object | None:
        """Return an object from the most recent scan by its stable object ID."""
        with self._objects_condition:
            return next((obj for obj in self._objects if obj.object_id == object_id), None)

    @rpc
    def select_grasp(self, object_id: str, rank: int = 0) -> PoseStamped | None:
        """Generate ranked proposals for an object and select one without moving."""
        if rank < 0:
            return None
        obj = self.get_object(object_id)
        if obj is None:
            return None
        provider: GraspGenSpec | None = (
            self._grasp_generator
            if self.config.grasp == "graspgenx"
            else self._heuristic_grasp_generator
        )
        if provider is None:
            raise RuntimeError("GraspGenX is not configured for this pick-and-place blueprint")
        candidates = provider.propose_grasps(obj.pointcloud)
        candidates = self._filter_candidates(candidates)
        self._grasp_candidates = candidates
        if rank >= len(candidates.candidates):
            self._selected_object_id = None
            self._selected_grasp = None
            self._selected_pregrasp = None
            return None
        candidates.selected_index = rank
        candidate = candidates.candidates[rank]
        self._selected_object_id = object_id
        self._selected_grasp = PoseStamped(
            ts=candidates.header.timestamp,
            frame_id=candidates.header.frame_id,
            position=candidate.pose.position,
            orientation=candidate.pose.orientation,
        )
        offset = self._selected_grasp.orientation.rotate_vector(
            Vector3(0.0, 0.0, -self.config.pregrasp_offset)
        )
        self._selected_pregrasp = PoseStamped(
            ts=self._selected_grasp.ts,
            frame_id=self._selected_grasp.frame_id,
            position=self._selected_grasp.position + offset,
            orientation=self._selected_grasp.orientation,
        )
        return self._selected_grasp

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
    def pick_selected(self, robot_name: str | None = None) -> SkillResult:
        """Pick the object selected by ``select_grasp``."""
        grasp = self._selected_grasp
        pregrasp = self._selected_pregrasp
        if grasp is None or pregrasp is None:
            return SkillResult.fail("INVALID_STATE", "Select a grasp before starting a pick")
        opened = self._pick_execution.open_gripper(robot_name)
        if not opened.is_success():
            return opened
        approach = self._move(pregrasp, robot_name)
        if not approach.is_success():
            return approach
        contact = self._move(grasp, robot_name)
        if not contact.is_success():
            return contact
        closed = self._pick_execution.close_gripper(robot_name)
        if not closed.is_success():
            return closed
        time.sleep(self.config.grasp_feedback_delay)
        gripper_position = self._pick_execution.get_gripper(robot_name)
        if gripper_position is None:
            return SkillResult.fail(
                "GRIPPER_FAILED", "Cannot verify pickup: gripper feedback unavailable"
            )
        if gripper_position <= self.config.grasp_empty_closed_threshold:
            self._pick_execution.open_gripper(robot_name)
            self._move(pregrasp, robot_name)
            return SkillResult.fail(
                "GRASP_VERIFICATION_FAILED", "Pickup failed: gripper reached empty-closed"
            )
        retreat = self._move(pregrasp, robot_name)
        if not retreat.is_success():
            return retreat
        self._holding_object = True
        return SkillResult.ok("Pick complete", object_id=self._selected_object_id)

    @skill(uses=[CAP_MOVEMENT])
    def place_at(self, x: float, y: float, z: float, robot_name: str | None = None) -> SkillResult:
        """Place the held object at an explicit position in the planning frame."""
        grasp = self._selected_grasp
        if grasp is None or not self._holding_object:
            return SkillResult.fail("INVALID_STATE", "Pick an object before placing")
        euler = grasp.orientation.to_euler()
        approach = self._pick_execution.move_to_pose(
            x, y, z + self.config.pregrasp_offset, euler.x, euler.y, euler.z, robot_name
        )
        if not approach.is_success():
            return approach
        placed = self._pick_execution.move_to_pose(x, y, z, euler.x, euler.y, euler.z, robot_name)
        if not placed.is_success():
            return placed
        opened = self._pick_execution.open_gripper(robot_name)
        if not opened.is_success():
            return opened
        retreat = self._pick_execution.move_to_pose(
            x, y, z + self.config.pregrasp_offset, euler.x, euler.y, euler.z, robot_name
        )
        if not retreat.is_success():
            return retreat
        self._holding_object = False
        return SkillResult.ok(f"Place complete at ({x:.3f}, {y:.3f}, {z:.3f})")

    def _move(self, pose: PoseStamped, robot_name: str | None) -> SkillResult:
        euler = pose.orientation.to_euler()
        return self._pick_execution.move_to_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            euler.x,
            euler.y,
            euler.z,
            robot_name,
        )
