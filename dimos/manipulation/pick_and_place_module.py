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

from collections.abc import Sequence
from dataclasses import dataclass
import math
import time
from typing import Any, Literal

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
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec

# The one box of mapped cells a pick asks the planner to ignore.
VOXEL_MAP_EXCLUSION_ID = "pick-and-place/target"
# How often (seconds) a check waiting for a new camera frame asks again.
_FRESH_FRAME_POLL = 0.1


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
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)
    # After the lift, the camera must see the object moved along the lift by at
    # least this share (0 to 1) of the gripper's lift distance, else it stayed down.
    lift_follow_fraction: float = Field(default=0.5, gt=0.0, le=1.0)
    # Detections of the target's label farther than this (metres) from where it
    # was last seen (and, after a lift, from where it would be had it come along)
    # are other objects. A released object is looked for this far from the gripper.
    lift_check_radius: float = Field(default=0.15, gt=0.0)
    # How far (metres, horizontally) a released object may rest from the place target.
    place_tolerance: float = Field(default=0.07, gt=0.0)
    # How long (seconds) a check after a lift or a release waits for a camera
    # frame taken after the arm stopped.
    fresh_frame_timeout: float = Field(default=2.0, gt=0.0)


@dataclass(frozen=True)
class ScannedObject:
    """One object from the latest scan.

    Args:
        object_id: ID the scene registration gave it.
        name: Detected label, e.g. "cup".
        position: Centre of its 3D box in the planning frame, metres.
    """

    object_id: str
    name: str
    position: Vector3


@dataclass(frozen=True)
class HeldObject:
    """The object a pick left in the gripper.

    Args:
        object_id: ID from the scan it was picked from.
        name: Detected label.
        grasp: Gripper pose that took it, in the planning frame.
    """

    object_id: str
    name: str
    grasp: PoseStamped


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
    """Axis-aligned bounds of a target's point cloud, planning frame, metres."""

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

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, ScannedObject] = {}
        self._grasp_candidates = GraspCandidateArray()
        self._held: HeldObject | None = None
        # True while a pick has left the gripper inside the target's ignored box;
        # the box then stays so the next plan can start from there.
        self._arm_in_target = False

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult:
        """Scan the latest RGB-D frame for prompted objects.

        Args:
            prompts: Object labels to detect. Use an ID from this scan with pick_object.
        """
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            raise ValueError("At least one object prompt is required")
        if self._held is None:
            self._clear_proposals()
        self._objects = {}
        detections = self._scene.scan_scene(text=prompts)
        self._objects = _scanned(detections)
        return SkillResult(
            f"Detected {detections.detections_length} object(s)",
            metadata={
                "prompts": prompts,
                "objects": [_summary(obj) for obj in self._objects.values()],
            },
        )

    @rpc
    def get_object(self, object_id: str) -> dict[str, Any] | None:
        obj = self._objects.get(object_id)
        return None if obj is None else _summary(obj)

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> SkillResult:
        """Grasp one object from the latest scan and lift it.

        After the lift the camera checks the object came up with the gripper.
        ``grasped: false`` in the result means it did not: the object is still on
        the table and the gripper is open. Scan again before another pick.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._held is not None:
            return SkillResult(
                f"Still holding object {self._held.object_id}; "
                f"did not start a pick of object {object_id}. Use place_at to put it down first."
            )
        self._clear_proposals()
        target = self._objects.get(object_id)
        if target is None:
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
        candidates = self._grasp_generator.propose_grasps(pointcloud)
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        if candidates.header.frame_id != self.config.planning_frame:
            raise RuntimeError(
                f"Grasp candidates are in frame {candidates.header.frame_id!r}; "
                f"the planning frame is {self.config.planning_frame!r}"
            )
        if not candidates.candidates:
            return SkillResult(f"Generated 0 grasp candidates for object {object_id}.")
        group = self._gripper_group(planning_group)
        # The target is mapped geometry: with its cells as obstacles the planner
        # refuses the approach into it, and the held object blocks every move after.
        box = _target_box(pointcloud)
        self._exclude_target(box)
        self._arm_in_target = False
        try:
            return self._pick_from(candidates, target, box, group)
        finally:
            if self._held is None and not self._arm_in_target:
                self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)

    def _pick_from(
        self,
        candidates: GraspCandidateArray,
        target: ScannedObject,
        box: TargetBox | None,
        group: PlanningGroupID,
    ) -> SkillResult:
        """Open the gripper and try the ranked grasps in order until one holds the object.

        Args:
            candidates: Ranked grasps in the planning frame, best first.
            target: The object being picked, with where it was scanned.
            box: Bounds of the object's point cloud, or None when it has none.
            group: Gripper-capable planning group doing the pick.
        """
        object_id = target.object_id
        if not_open := self._open_gripper(group, "before grasping"):
            return not_open

        last_plan = ""
        for rank, candidate in enumerate(candidates.candidates[: self.config.max_grasp_attempts]):
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
                if blocked := self._servo(grasp, pregrasp, group):
                    self._arm_in_target = True
                    stopped = self._stopped("Retract to the pre-grasp pose", blocked)
                    return SkillResult(
                        f"{not_held.message} {stopped.message}", metadata=not_held.metadata
                    )
                return not_held

            self._held = HeldObject(object_id, target.name, grasp)
            self._carry_target(box, grasp, group)
            if blocked := self._servo(grasp, pregrasp, group):
                return self._stopped(f"Retract after grasping object {object_id}", blocked)
            result = self._verify_lift(target, pregrasp.position - grasp.position, group)
            result.metadata.update(
                rank=rank, score=candidate.score, candidates=len(candidates.candidates)
            )
            return result
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

        After the release the camera checks where the object rests. ``placed``
        in the result says whether it is within the place tolerance of the target.

        Args:
            x: Planning-frame X coordinate in meters.
            y: Planning-frame Y coordinate in meters.
            z: Planning-frame Z coordinate in meters.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        target = f"({x:.2f}, {y:.2f}, {z:.2f})"
        held = self._held
        if held is None:
            return SkillResult(
                f"Not holding any object; nothing was placed at {target}. Use pick_object first.",
                metadata={"placed": False},
            )
        group = self._gripper_group(planning_group)
        if lost := self._lost_in_transit(held, group, target):
            return lost
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=held.grasp.orientation,
        )
        preplace = self._offset_pose(place, self.config.pregrasp_offset)
        if blocked := self._move(preplace, group):
            return self._stopped(f"Move to the pre-place pose above {target}", blocked)
        if blocked := self._servo(preplace, place, group):
            return self._stopped(f"Move down to the place pose {target}", blocked)
        if not_open := self._open_gripper(group, "to release the object"):
            return SkillResult(f"{not_open.message} The arm stayed at the place pose.")
        self._held = None
        self._clear_proposals()
        tip = self._tip_position(group)
        released_at = place.position if tip is None else tip
        if blocked := self._servo(place, preplace, group):
            # The gripper is still around the object; its box stays ignored.
            return self._stopped(f"Retract from {target} after releasing", blocked)
        self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)
        return self._verify_place(held, place.position, released_at)

    def _verify_lift(
        self, target: ScannedObject, lift: Vector3, planning_group: PlanningGroupID
    ) -> SkillResult:
        """Check that the object came up with the gripper, and report the pick.

        Jaws that stop on an edge, or on an object pushed aside, read the same as
        a hold, so after the lift the jaws are read again and the camera looks for
        the object.

        Args:
            target: The object being picked, with where it was scanned.
            lift: How the gripper moved from the grasp pose to the pre-grasp pose,
                metres, in the planning frame.
            planning_group: Group whose gripper closed on the object.

        Returns "Pick complete" with ``grasped: True``, else opens the gripper,
        forgets the held object and returns ``grasped: False``.
        """
        lifted_at = time.time()
        facts: dict[str, Any] = {
            **_summary(target),
            "gripper_lift_m": round(lift.length(), 3),
        }
        verification = self.config.grasp_verification
        jaws = self._gripper_position(planning_group)
        if verification.enabled and jaws is not None and jaws <= verification.held_low:
            return self._missed_grasp(
                planning_group,
                f"After lifting object {target.object_id}, the gripper reads {jaws:.2f}: "
                "the jaws are closed on nothing.",
                jaws=round(jaws, 3),
                **facts,
            )
        detections = self._detect_after(target.name, lifted_at)
        moved = _distance_along(
            [obj.position for obj in self._sightings(target, detections)],
            target.position,
            lift,
            self.config.lift_check_radius,
        )
        facts["object_lift_m"] = None if moved is None else round(moved, 3)
        if moved is not None and moved < self.config.lift_follow_fraction * lift.length():
            return self._missed_grasp(
                planning_group,
                f"Lifted the gripper {lift.length():.2f} m, but the camera sees object "
                f"{target.object_id} moved {moved:.2f} m with it; it stayed on the table.",
                **facts,
            )
        return SkillResult("Pick complete", metadata={**facts, "grasped": True})

    def _sightings(
        self, target: ScannedObject, detections: Detection3DArray
    ) -> list[ScannedObject]:
        """The detections that may be the target.

        The scene registration keeps an object's ID while it stays put, so a
        detection with the target's own ID is the target, and one with the ID of
        another object from the latest scan is not.

        Args:
            target: The object being looked for.
            detections: A scan for the target's label.
        """
        seen = _scanned(detections)
        if target.object_id in seen:
            return [seen[target.object_id]]
        return [obj for obj in seen.values() if obj.object_id not in self._objects]

    def _detect_after(self, label: str, since: float) -> Detection3DArray:
        """Detect a label in a camera frame taken after a given time.

        Args:
            label: Object label to detect.
            since: Wall-clock time (seconds, as ``time.time()``) the frame must be
                newer than, e.g. when the arm stopped.

        Raises TimeoutError when no such frame arrives within ``fresh_frame_timeout``.
        """
        deadline = time.monotonic() + self.config.fresh_frame_timeout
        while True:
            detections = self._scene.scan_scene(text=[label])
            if detections.ts >= since:
                return detections
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"No camera frame newer than {since:.2f} arrived within "
                    f"{self.config.fresh_frame_timeout:.1f} s; the newest is from {detections.ts:.2f}"
                )
            time.sleep(_FRESH_FRAME_POLL)

    def _missed_grasp(
        self, planning_group: PlanningGroupID, report: str, **facts: Any
    ) -> SkillResult:
        """Open the gripper after a grasp that did not take the object.

        Args:
            planning_group: Group whose gripper to open.
            report: What showed the grasp failed, as one or more sentences.
            facts: Measured values to include in the result.
        """
        # pick_object clears the voxel-map box once the arm has backed out.
        note = self._let_go(planning_group, "after the failed grasp")
        return SkillResult(f"{report} {note}", metadata={**facts, "grasped": False})

    def _lost_in_transit(
        self, held: HeldObject, planning_group: PlanningGroupID, target: str
    ) -> SkillResult | None:
        """Report an object that fell out of the gripper since the pick, or None.

        Args:
            held: The object the last pick took.
            planning_group: Group whose gripper holds it.
            target: The requested place position, as text for the report.
        """
        verification = self.config.grasp_verification
        jaws = self._gripper_position(planning_group)
        if not verification.enabled or jaws is None or jaws > verification.held_low:
            return None
        self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)
        note = self._let_go(planning_group, "after finding it empty")
        return SkillResult(
            f"The gripper reads {jaws:.2f}: object {held.object_id} is no longer in the "
            f"jaws, so nothing was placed at {target}. {note}",
            metadata={**_summary(held), "placed": False, "jaws": round(jaws, 3)},
        )

    def _let_go(self, planning_group: PlanningGroupID, step: str) -> str:
        """Forget the held object and the proposals, open the gripper, and say how that went.

        Args:
            planning_group: Group whose gripper to open.
            step: Why it is being opened, completing "Commanded the gripper open ...".
        """
        self._held = None
        self._clear_proposals()
        reopened = self._open_gripper(planning_group, step)
        return "Reopened the gripper." if reopened is None else reopened.message

    def _verify_place(self, held: HeldObject, target: Vector3, released_at: Vector3) -> SkillResult:
        """Look for the released object and report how far it rests from the target.

        Args:
            held: The object that was released.
            target: Requested place position, metres, in the planning frame.
            released_at: Where the gripper tip was when the jaws opened, metres.
                Only detections within ``lift_check_radius`` of it count, and it
                stands in for the object when the camera sees none.
        """
        detections = self._detect_after(held.name, time.time())
        nearby = [
            center
            for center in _centers(detections)
            if center.distance(released_at) <= self.config.lift_check_radius
        ]
        if nearby:
            rest = min(nearby, key=lambda center: center.distance(released_at))
            located_by = "camera"
        else:
            # Right under the gripper the fingers can hide the object from the
            # wrist camera; where the jaws opened is then the best estimate.
            rest, located_by = released_at, "gripper"
        offset = _horizontal_distance(rest, target)
        placed = offset <= self.config.place_tolerance
        facts = {
            **_summary(held),
            "placed": placed,
            "offset_m": round(offset, 3),
            "located_by": located_by,
        }
        if placed:
            return SkillResult("Place complete", metadata=facts)
        return SkillResult(
            f"Released object {held.object_id}; it rests {offset:.2f} m from the target, "
            f"farther than the {self.config.place_tolerance:.2f} m place tolerance.",
            metadata=facts,
        )

    def _clear_proposals(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._manipulation.show_grasp_proposals(GraspCandidateArray())

    def _exclude_target(self, box: TargetBox | None) -> None:
        """Keep the target's own cells, and the table right under it, out of the planner."""
        if box is None:
            return
        self._manipulation.set_voxel_map_exclusion(
            VOXEL_MAP_EXCLUSION_ID, box.center, self._padded(box.size)
        )

    def _carry_target(
        self, box: TargetBox | None, grasp: PoseStamped, group: PlanningGroupID
    ) -> None:
        """Move the ignored box from the table to the gripper, where the object now is."""
        if box is None:
            return
        # The box centre relative to the tool tip, in the tip's own frame.
        offset = grasp.orientation.inverse().rotate_vector(box.center - grasp.position)
        self._manipulation.set_voxel_map_exclusion(
            VOXEL_MAP_EXCLUSION_ID, offset, self._padded(box.size), planning_group=group
        )

    def _padded(self, size: Vector3) -> Vector3:
        """Box edge lengths (metres) grown by target_clearance on every side."""
        margin = 2.0 * self.config.target_clearance
        return Vector3(size.x + margin, size.y + margin, size.z + margin)

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
        if not settle.settled and config.held_low < settle.position < config.held_high:
            # Still closing on something, e.g. a soft object: treat it as a hold
            # and let the check after the lift decide.
            return None
        report = f"Closed the gripper on object {object_id}. {_gripper_reading(settle)}"
        if "nothing in the jaws" not in failure:
            return SkillResult(report)
        return self._missed_grasp(
            planning_group, report, object_id=object_id, jaws=round(settle.position, 3)
        )

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None

    def _tip_position(self, planning_group: PlanningGroupID) -> Vector3 | None:
        """Where the gripper tip is now, metres, in the planning frame, or None."""
        state = self._manipulation.get_state().groups.get(planning_group)
        if state is None or state.end_effector_pose is None:
            return None
        return state.end_effector_pose.position


def _center(detection: Any) -> Vector3:
    """Centre of a detection's 3D box, metres."""
    position = detection.bbox.center.position
    return Vector3(position.x, position.y, position.z)


def _centers(detections: Detection3DArray) -> list[Vector3]:
    return [_center(detection) for detection in detections.detections]


def _scanned(detections: Detection3DArray) -> dict[str, ScannedObject]:
    """Index a scan's detections by object ID, skipping any without an ID or label."""
    return {
        str(detection.id): ScannedObject(
            object_id=str(detection.id),
            name=str(detection.results[0].hypothesis.class_id),
            position=_center(detection),
        )
        for detection in detections.detections
        if detection.id and detection.results
    }


def _summary(obj: ScannedObject | HeldObject) -> dict[str, Any]:
    """ID and label of an object, as skill results and RPCs report it."""
    return {"object_id": obj.object_id, "name": obj.name}


def _distance_along(
    centers: Sequence[Vector3], start: Vector3, lift: Vector3, radius: float
) -> float | None:
    """How far a lifted object moved with the gripper, as the camera sees it.

    Args:
        centers: Centres of the detections of the object's label after the lift,
            metres, in the planning frame.
        start: Centre of the object before the grasp, metres.
        lift: How the gripper moved from the grasp pose to the pre-grasp pose, metres.
        radius: Detections farther than this (metres) from both ``start`` and
            ``start + lift`` are taken to be other objects.

    Returns the farthest any remaining detection moved along the lift direction,
    metres, or None when no detection remains.
    """
    lifted_to = start + lift
    nearby = [
        center
        for center in centers
        if min(center.distance(start), center.distance(lifted_to)) <= radius
    ]
    if not nearby:
        return None
    direction = lift.normalize()
    return max((center - start).dot(direction) for center in nearby)


def _horizontal_distance(a: Vector3, b: Vector3) -> float:
    """Distance between two points ignoring height, metres."""
    return math.hypot(a.x - b.x, a.y - b.y)
