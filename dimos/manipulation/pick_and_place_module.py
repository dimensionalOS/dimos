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

from dataclasses import dataclass
from typing import Any

import numpy as np
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
from dimos.manipulation.manipulation_spec import ExecutionResult, ManipulationSpec, PlanResult
from dimos.manipulation.planning.gripper_contact import GripperContactModel
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The one box a pick leaves out of the planner's map.
TARGET_BOX = "pick-and-place/target"


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # Margin (metres) added around the target's bounding box. Mapped cells in
    # that box are not obstacles during the pick, and the same box rides with
    # the gripper while the object is held. Too small, and a cell on the
    # object's edge or the table under it stops the plan; too large, and a
    # neighbour's cells are ignored too.
    target_clearance: float = Field(default=0.03, gt=0.0)
    # A learned provider returns a ranked spread whose best-scoring pose is not
    # always kinematically reachable; a single-candidate provider is unaffected.
    max_grasp_attempts: int = Field(default=5, gt=0)
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)
    # The robot, for checking grasps against its gripper's shape before moving:
    # a grasp whose gripper would touch the target anywhere but its contact
    # patches (the finger pads), open or as the fingers close, or would overlap
    # another scene point, is dropped. None, or a robot model without a gripper
    # config, tries every grasp the provider gives.
    robot: RobotModelConfig | None = None
    # A gripper part closer than this (metres) to the target touches it.
    grasp_contact_margin: float = Field(default=0.005, gt=0.0)
    # A gripper part closer than this (metres) to another scene point overlaps it.
    grasp_obstacle_margin: float = Field(default=0.003, gt=0.0)


def _status(result: PlanResult | ExecutionResult) -> str:
    """Status name and message of a planner or execution result, e.g. 'FAILED: no path'."""
    return f"{result.status.name}: {result.message}" if result.message else result.status.name


def _gripper_reading(settle: GripperSettle) -> str:
    """One sentence saying where the jaws stopped after a gripper command."""
    if settle.position is None:
        return "No gripper position readback."
    if not settle.settled:
        return (
            f"Gripper position {settle.position:.2f} had not settled after {settle.elapsed:.1f} s."
        )
    return f"Final gripper position {settle.position:.2f}."


@dataclass(frozen=True)
class TargetBox:
    """Axis-aligned bounds of a target's point cloud, in the cloud's frame, metres."""

    center: Vector3
    size: Vector3


def _target_box(pointcloud: PointCloud2) -> TargetBox | None:
    """The axis-aligned bounds of an object's points, or None for an empty cloud."""
    points = np.asarray(pointcloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    if not len(points) or not np.isfinite(points).all():
        return None
    low, high = points.min(axis=0), points.max(axis=0)
    return TargetBox(Vector3(*((low + high) / 2.0)), Vector3(*(high - low)))


class PickAndPlaceModule(Module):
    """Coordinate scene registration, grasp generation, and manipulation execution."""

    config: PickAndPlaceModuleConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec
    _manipulation: ManipulationSpec
    # Leaves boxes out of the planner's map; a stack without a map has none.
    _box_filter: PointCloudBoxFilterSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, dict[str, Any]] = {}
        self._grasp_candidates = GraspCandidateArray()
        self._selected_object_id: str | None = None
        self._selected_grasp: PoseStamped | None = None
        self._holding_object = False
        # The gripper's shape for the grasp contact check, by tool tip link, built on first use.
        self._contact_models: dict[str, GripperContactModel] = {}

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult:
        """Scan the latest RGB-D frame for prompted objects.

        Args:
            prompts: Object labels to detect. Use an ID from this scan with pick_object.
        """
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            raise ValueError("At least one object prompt is required")
        if not self._holding_object:
            self._clear_selection()
        self._objects = {}
        detections = self._scene.scan_scene(text=prompts)
        objects = [
            {
                "object_id": str(detection.id),
                "name": str(detection.results[0].hypothesis.class_id),
            }
            for detection in detections.detections
            if detection.id and detection.results
        ]
        self._objects = {str(obj["object_id"]): obj for obj in objects if "object_id" in obj}
        return SkillResult(
            f"Detected {detections.detections_length} object(s)",
            metadata={"prompts": prompts, "objects": list(self._objects.values())},
        )

    @rpc
    def get_object(self, object_id: str) -> dict[str, Any] | None:
        return self._objects.get(object_id)

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> SkillResult:
        """Generate ranked grasps and pick one object from the latest scan.

        ``contact_rejected`` in the result counts the grasps dropped because their
        gripper would touch the object off its finger pads or hit the scene.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._holding_object:
            return SkillResult(
                f"Still holding object {self._selected_object_id}; "
                f"did not start a pick of object {object_id}. Use place_at to put it down first."
            )
        self._clear_selection()
        if object_id not in self._objects:
            scanned = ", ".join(self._objects) or "none"
            return SkillResult(
                f"No object with id {object_id} in the latest scan. Scanned ids: {scanned}. "
                "Use scan_objects to refresh the list."
            )
        pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
        if pointcloud is None:
            return SkillResult(
                f"Object {object_id} has no point cloud in the latest scan. Use scan_objects again."
            )
        group = self._gripper_group(planning_group)
        candidates = self._grasp_generator.propose_grasps(pointcloud)
        candidates, facts = self._drop_touching(object_id, pointcloud, candidates, group)
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        if candidates.header.frame_id != self.config.planning_frame:
            raise RuntimeError(
                f"Grasp candidates are in frame {candidates.header.frame_id!r}; "
                f"the planning frame is {self.config.planning_frame!r}"
            )
        if not candidates.candidates:
            note = ""
            if rejected := facts.get("contact_rejected"):
                note = (
                    f" The contact check dropped {rejected} grasp(s) whose gripper would touch "
                    "the object off its finger pads or hit the scene."
                )
            return SkillResult(
                f"Generated 0 grasp candidates for object {object_id}.{note}", metadata=facts
            )
        # The target is mapped geometry: with its cells as obstacles the planner
        # refuses the approach into it, and the held object blocks every move after.
        box = _target_box(pointcloud)
        if box is not None:
            self._leave_out(Pose(position=box.center), box.size, pointcloud.frame_id or None)
        try:
            result = self._pick_from(candidates, object_id, box, group)
        finally:
            if not self._holding_object:
                self._clear_target_box()
        result.metadata = {**facts, **result.metadata}
        return result

    def _drop_touching(
        self,
        object_id: str,
        pointcloud: PointCloud2,
        candidates: GraspCandidateArray,
        group: PlanningGroupID,
    ) -> tuple[GraspCandidateArray, dict[str, Any]]:
        """Drop the grasps whose gripper would touch the target off its finger pads, or hit the scene.

        Args:
            object_id: The object to grasp, from the latest scan.
            pointcloud: The target's points, planning frame, metres.
            candidates: The provider's grasps, same frame.
            group: Planning group whose tool tip the grasps pose.

        Returns the grasps that pass, and for the pick's result either
        ``contact_rejected`` (how many were dropped) or ``contact_check_skipped``
        (why every grasp was kept); nothing when the config has no robot model
        with a gripper config.
        """
        robot = self.config.robot
        if robot is None or robot.gripper is None or not candidates.candidates:
            return candidates, {}
        scene = self._scene.get_full_scene_pointcloud(exclude_object_id=object_id)
        if scene is None or not scene.frame_id == pointcloud.frame_id == candidates.header.frame_id:
            return candidates, {
                "contact_check_skipped": "no scene point cloud in the grasps' frame"
            }
        tips = [
            state.end_effector_pose.position
            for state in self._manipulation.get_state().groups.values()
            if state.end_effector_pose is not None
        ]
        try:
            tip = self._tip_frame(group)
            if tip not in self._contact_models:
                self._contact_models[tip] = GripperContactModel(
                    robot.model.load(), robot.gripper, tip
                )
            kept = self._contact_models[tip].keep_clear(
                candidates,
                pointcloud,
                scene,
                tips,
                self.config.grasp_contact_margin,
                self.config.grasp_obstacle_margin,
            )
        except Exception as exc:
            logger.warning("Grasp contact check failed; keeping every grasp", error=str(exc))
            return candidates, {"contact_check_skipped": f"{type(exc).__name__}: {exc}"}
        return kept, {"contact_rejected": len(candidates.candidates) - len(kept.candidates)}

    def _pick_from(
        self,
        candidates: GraspCandidateArray,
        object_id: str,
        box: TargetBox | None,
        group: PlanningGroupID,
    ) -> SkillResult:
        """Open the gripper and try the ranked grasps in order until one holds the object.

        Args:
            candidates: Ranked grasps in the planning frame, best first.
            object_id: ID of the object being picked, for the report.
            box: Bounds of the object's point cloud, or None when it has none.
            group: Gripper-capable planning group doing the pick.
        """
        if not_open := self._open_gripper(group, "before grasping"):
            return not_open

        last_plan = ""
        for rank, candidate in enumerate(candidates.candidates[: self.config.max_grasp_attempts]):
            grasp = PoseStamped(
                ts=candidates.header.timestamp,
                frame_id=candidates.header.frame_id,
                position=candidate.pose.position,
                orientation=candidate.pose.orientation,
            )
            pregrasp = self._offset_pose(grasp, self.config.pregrasp_offset)
            blocked = self._move(pregrasp, group) or self._servo(pregrasp, grasp, group)
            if isinstance(blocked, PlanResult):
                # The planner found no path to this candidate; the next one may
                # differ. A motion that stopped part-way would stop the same way
                # for every candidate, so that is not retried.
                last_plan = _status(blocked)
                continue
            if blocked is not None:
                return self._stopped(
                    f"Move to grasp candidate {rank} for object {object_id}", blocked
                )
            if not_held := self._close_and_verify(group, object_id):
                # Back out while the target's cells are still ignored: left at
                # the grasp pose, the arm would start the next plan inside them.
                # Only with the jaws open again, so nothing half-gripped is dragged.
                if not_held.metadata.get("reopened") and (
                    blocked := self._servo(grasp, pregrasp, group)
                ):
                    stopped = self._stopped("Retract to the pre-grasp pose", blocked)
                    return SkillResult(f"{not_held.message} {stopped.message}")
                return not_held

            self._selected_object_id = object_id
            self._selected_grasp = grasp
            self._holding_object = True
            if box is not None:
                # The box in the tool tip's frame, as it is at the grasp pose:
                # it rides with the tip from here.
                inverse = grasp.orientation.inverse()
                self._leave_out(
                    Pose(
                        position=inverse.rotate_vector(box.center - grasp.position),
                        orientation=inverse,
                    ),
                    box.size,
                    self._tip_frame(group),
                )
            if blocked := self._servo(grasp, pregrasp, group):
                return self._stopped(f"Retract after grasping object {object_id}", blocked)
            return SkillResult(
                "Pick complete",
                metadata={
                    "object_id": object_id,
                    "rank": rank,
                    "score": candidate.score,
                    "candidates": len(candidates.candidates),
                },
            )
        attempted = min(len(candidates.candidates), self.config.max_grasp_attempts)
        return SkillResult(
            f"The planner found no path to any of the {attempted} grasp candidate(s) tried "
            f"for object {object_id}; last planner result {last_plan}."
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
    ) -> SkillResult:
        """Place the held object at an explicit planning-frame position.

        Args:
            x: Planning-frame X coordinate in meters.
            y: Planning-frame Y coordinate in meters.
            z: Planning-frame Z coordinate in meters.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        target = f"({x:.2f}, {y:.2f}, {z:.2f})"
        if self._selected_grasp is None or not self._holding_object:
            return SkillResult(
                f"Not holding any object; nothing was placed at {target}. Use pick_object first."
            )
        group = self._gripper_group(planning_group)
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=self._selected_grasp.orientation,
        )
        preplace = self._offset_pose(place, self.config.pregrasp_offset)
        if blocked := self._move(preplace, group):
            return self._stopped(f"Move to the pre-place pose above {target}", blocked)
        if blocked := self._servo(preplace, place, group):
            return self._stopped(f"Move down to the place pose {target}", blocked)
        if not_open := self._open_gripper(group, "to release the object"):
            return SkillResult(f"{not_open.message} The arm stayed at the place pose.")
        self._holding_object = False
        self._clear_selection()
        # Released, the object no longer rides with the tool, so neither does its box.
        self._clear_target_box()
        if blocked := self._servo(place, preplace, group):
            return self._stopped(f"Retract from {target} after releasing", blocked)
        return SkillResult("Place complete")

    def _clear_selection(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._manipulation.show_grasp_proposals(GraspCandidateArray())
        self._selected_object_id = None
        self._selected_grasp = None

    def _leave_out(self, pose: Pose, size: Vector3, frame_id: str | None = None) -> None:
        """Leave the target's box, grown by target_clearance, out of the planner's map.

        Args:
            pose: The box's centre and orientation in ``frame_id``, metres.
            size: The target's edge lengths, metres.
            frame_id: The frame the box is fixed in, or None for the map's own
                frame. A moving frame, such as the tool tip, carries the box.
        """
        if self._box_filter is None:
            return
        margin = 2.0 * self.config.target_clearance
        self._box_filter.set_box(
            TARGET_BOX, pose, Vector3(size.x + margin, size.y + margin, size.z + margin), frame_id
        )

    def _clear_target_box(self) -> None:
        if self._box_filter is not None:
            self._box_filter.clear_box(TARGET_BOX)

    def _tip_frame(self, group: PlanningGroupID) -> str:
        """The tool tip frame of a planning group that has one."""
        return next(
            info.tip_frame
            for info in self._manipulation.list_planning_groups()
            if info.id == group and info.tip_frame is not None
        )

    def _gripper_group(self, planning_group: PlanningGroupID | None) -> PlanningGroupID:
        """Pick the planning group that has both a gripper and a tool frame.

        Args:
            planning_group: Group ID to use, or None to use the only such group.

        Raises ValueError when the ID names no such group, or when it is omitted
        and there is not exactly one.
        """
        groups = [
            group.id
            for group in self._manipulation.list_planning_groups()
            if group.has_gripper and group.tip_frame is not None
        ]
        if planning_group is None:
            if len(groups) == 1:
                return groups[0]
            raise ValueError(
                "Expected exactly one gripper-capable planning group when planning_group "
                f"is omitted; found {groups}"
            )
        if planning_group not in groups:
            raise ValueError(
                f"planning_group {planning_group!r} is not gripper-capable; "
                f"gripper-capable groups: {groups}"
            )
        return planning_group

    @staticmethod
    def _stopped(step: str, result: PlanResult | ExecutionResult) -> SkillResult:
        """Report a motion step that did not finish, and what stopped it.

        Args:
            step: The motion that was attempted, as the start of a sentence.
            result: The planner or execution result that did not succeed.
        """
        source = "planner" if isinstance(result, PlanResult) else "execution"
        return SkillResult(f"{step} did not complete; {source} returned {_status(result)}")

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + pose.orientation.rotate_vector(Vector3(0.0, 0.0, -offset)),
            orientation=pose.orientation,
        )

    def _servo(
        self, start: PoseStamped, end: PoseStamped, planning_group: PlanningGroupID
    ) -> PlanResult | ExecutionResult | None:
        """Drive the last leg as a straight line with collision checking off.

        The object being grasped is itself mapped geometry once a voxel map feeds
        the planner, so a collision-checked plan into it can only ever be
        rejected. This leg is short, straight, and deliberately ends in contact.

        Returns the planner or execution result that stopped the leg, or None
        when the arm arrived.
        """
        result = self._manipulation.move_linear(
            end.position.x - start.position.x,
            end.position.y - start.position.y,
            end.position.z - start.position.z,
            planning_group,
            check_collision=False,
        )
        if not result.plan.succeeded:
            return result.plan
        if result.execution is None:
            raise RuntimeError("Linear move was planned but never executed")
        return None if result.execution.succeeded else result.execution

    def _move(
        self, pose: PoseStamped, planning_group: PlanningGroupID
    ) -> PlanResult | ExecutionResult | None:
        """Plan a collision-checked path to ``pose`` and run it.

        Returns the planner or execution result that stopped the leg, or None
        when the arm arrived.
        """
        plan = self._manipulation.plan_to_poses({planning_group: pose})
        if not plan.succeeded:
            return plan
        execution = self._manipulation.execute(blocking=True)
        return None if execution.succeeded else execution

    def _command_and_settle(
        self,
        position: float,
        planning_group: PlanningGroupID,
        arrival_tolerance: float | None = None,
    ) -> GripperSettle:
        """Command the gripper to ``position`` and wait for its readback to stop moving.

        Args:
            position: Target opening, 0.0 fully closed to 1.0 fully open.
            planning_group: Group whose gripper to command.
            arrival_tolerance: How close to ``position`` counts as arrived, in the
                same 0.0 to 1.0 units; None uses the settle tolerance.

        Raises RuntimeError when the gripper command is not accepted.
        """
        result = self._manipulation.set_gripper_position(position, planning_group)
        if not result.succeeded:
            raise RuntimeError(
                f"Gripper command to {position:.2f} on {planning_group} was not accepted: "
                f"{result.message}"
            )
        return await_gripper_settle(
            lambda: self._gripper_position(planning_group),
            position,
            self.config.grasp_verification,
            arrival_tolerance=arrival_tolerance,
        )

    def _open_gripper(self, planning_group: PlanningGroupID, step: str) -> SkillResult | None:
        """Open the gripper and wait for it to settle.

        Args:
            planning_group: Group whose gripper to open.
            step: Why it is being opened, completing "Commanded the gripper open ...".

        Returns None when the jaws reached open (or there is no readback), else a
        statement of where they stopped.
        """
        # Jaws resting against the open stop never move and never reach the
        # commanded extreme; open_tolerance is the band that already decides
        # whether where they stopped counts as open.
        settle = self._command_and_settle(
            self.config.grasp_verification.open_position,
            planning_group,
            arrival_tolerance=self.config.grasp_verification.open_tolerance,
        )
        if settle.position is None or open_failure(settle, self.config.grasp_verification) is None:
            return None
        return SkillResult(f"Commanded the gripper open {step}. {_gripper_reading(settle)}")

    def _close_and_verify(
        self, planning_group: PlanningGroupID, object_id: str
    ) -> SkillResult | None:
        """Close the gripper on the object and check the jaws stopped on something.

        Args:
            planning_group: Group whose gripper to close.
            object_id: ID of the object being grasped, for the report.

        Returns None when an object is held, else a statement of where the jaws
        stopped. Raises RuntimeError when verification is on but the gripper
        gives no position readback.
        """
        config = self.config.grasp_verification
        settle = self._command_and_settle(config.closed_position, planning_group)
        if not config.enabled:
            return None
        if settle.position is None:
            raise RuntimeError(
                f"No gripper position readback from {planning_group} "
                f"after closing on object {object_id}"
            )
        failure = grasp_failure(settle, config)
        if failure is None:
            return None
        report = f"Closed the gripper on object {object_id}. {_gripper_reading(settle)}"
        if "nothing in the jaws" in failure:
            reopened = self._open_gripper(planning_group, "after closing on nothing")
            if reopened is None:
                return SkillResult(f"{report} Reopened the gripper.", metadata={"reopened": True})
            report = f"{report} {reopened.message}"
        return SkillResult(report)

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None
