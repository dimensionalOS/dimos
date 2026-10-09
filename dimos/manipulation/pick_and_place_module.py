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
import math
from typing import Any, Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.manipulation.grasp_verification import (
    GraspVerificationConfig,
    GripperSettle,
    await_gripper_settle,
    grasp_failure,
    open_failure,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    PlanResult,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningGroupID
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # The pregrasp backs off along the tool's -Z. Grippers whose grasp frame
    # points Z out of the back of the palm need +Z, or the approach starts
    # underneath the object.
    pregrasp_along_tool_z: bool = False
    # Lift above the place pose before lowering; None reuses pregrasp_offset. A
    # short arm dropping into a bin needs less headroom than it needs over a grasp.
    preplace_offset: float | None = Field(default=None, gt=0.0)
    # A learned provider returns a ranked spread whose best-scoring pose is not
    # always kinematically reachable; a single-candidate provider is unaffected.
    max_grasp_attempts: int = Field(default=5, gt=0)
    # Judge a staged grasp by the camera after the lift instead of by the jaws
    # after the close: the object is held when it is no longer at its start
    # position. Soft objects squash to the empty reading, so the jaws alone
    # cannot tell a held banana from an empty close.
    verify_lift_by_scan: bool = True
    # How far the object must have moved from its scanned position to count
    # as lifted, in metres, in the plane of the table.
    grasp_displacement_tolerance: float = Field(default=0.03, gt=0.0)
    # How far from its scanned position an object may be found, still at table
    # height, and count as pushed aside rather than lifted.
    pushed_search_radius: float = Field(default=0.15, gt=0.0)
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)


@dataclass
class _StagedLeg:
    """One step of a staged job: a planned motion, or a gripper action.

    A motion leg keeps its goal so it can be planned again from wherever the
    arm actually is when the controller refuses the staged trajectory.
    """

    label: str
    plan: GeneratedPlan | None = None
    gripper: Literal["open", "close", "release"] | None = None
    mode: Literal["pose", "linear", "joints"] | None = None
    target: PoseStamped | None = None
    joints: JointState | None = None


@dataclass
class _StagedProgram:
    object_id: str
    planning_group: PlanningGroupID
    grasp: PoseStamped
    legs: list[_StagedLeg]
    rank: int
    score: float
    candidates: int
    name: str = ""
    start: Vector3 | None = None

    @property
    def motion_seconds(self) -> float:
        return sum(leg.plan.trajectory.duration for leg in self.legs if leg.plan is not None)


class _UnplannableLegError(Exception):
    """A leg of a staged job could not be planned."""


# Wrist turns about vertical tried for the place pose, the grasp's own heading first.
logger = setup_logger()

_PLACE_YAW_DELTAS = (0.0, math.pi / 2, -math.pi / 2, math.pi, math.pi / 4, -math.pi / 4)
# Tilts about the jaw axis, in the tool frame, tried after every heading has
# failed upright. A tilted wrist reaches farther than a vertical one, and the
# jaws stay level so the object stays held.
_PLACE_TILT_DELTAS = (0.0, 0.35, -0.35, 0.9, -0.9)


def _merge_joint_state(start: JointState, end: JointState) -> JointState:
    """The full model state after a plan: *start* with the planned joints at *end*."""
    positions = dict(zip(start.name, start.position, strict=True))
    positions.update(zip(end.name, end.position, strict=True))
    return JointState(name=list(positions), position=list(positions.values()))


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


class PickAndPlaceModule(Module):
    """Coordinate scene registration, grasp generation, and manipulation execution."""

    config: PickAndPlaceModuleConfig
    # For the viewer: every proposal of the latest pick, ranked, and the one
    # being attempted, both in the planning frame.
    grasp_candidates: Out[PoseArray]
    grasp_target: Out[PoseStamped]

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
        self._staged: _StagedProgram | None = None

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
                # Centre in the planning frame, so the caller can pick the arm
                # on the object's side without another call.
                "x": round(detection.bbox.center.position.x, 3),
                "y": round(detection.bbox.center.position.y, 3),
                "z": round(detection.bbox.center.position.z, 3),
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

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._holding_object:
            return SkillResult(
                f"Still holding object {self._selected_object_id}; "
                f"did not start a pick of object {object_id}. Use place_at to put it down first."
            )
        self._staged = None
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
        candidates = self._grasp_generator.propose_grasps(pointcloud)
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        self.grasp_candidates.publish(
            PoseArray(
                Header(candidates.header.timestamp, candidates.header.frame_id),
                [candidate.pose for candidate in candidates.candidates],
            )
        )
        if candidates.header.frame_id != self.config.planning_frame:
            raise RuntimeError(
                f"Grasp candidates are in frame {candidates.header.frame_id!r}; "
                f"the planning frame is {self.config.planning_frame!r}"
            )
        if not candidates.candidates:
            return SkillResult(f"Generated 0 grasp candidates for object {object_id}.")
        group = self._gripper_group(planning_group)
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
            self.grasp_target.publish(grasp)
            pregrasp = self._offset_pose(grasp, self._pregrasp_offset())
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
                return not_held

            self._selected_object_id = object_id
            self._selected_grasp = grasp
            self._holding_object = True
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

    @skill
    def stage_pick_and_place(
        self,
        object_id: str,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult:
        """Plan a whole pick and place without moving: approach, grasp, lift,
        carry, place, retreat and return home, each leg from the predicted end
        of the one before. The viewer shows the full motion; nothing runs
        until proceed is called.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            x: Planning-frame X of the place pose in meters.
            y: Planning-frame Y of the place pose in meters.
            z: Planning-frame Z of the place pose in meters.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        self._staged = None
        if self._holding_object:
            return SkillResult(
                f"Still holding object {self._selected_object_id}; put it down before staging "
                f"a pick of object {object_id}."
            )
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
        candidates = self._grasp_generator.propose_grasps(pointcloud)
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        self.grasp_candidates.publish(
            PoseArray(
                Header(candidates.header.timestamp, candidates.header.frame_id),
                [candidate.pose for candidate in candidates.candidates],
            )
        )
        if candidates.header.frame_id != self.config.planning_frame:
            raise RuntimeError(
                f"Grasp candidates are in frame {candidates.header.frame_id!r}; "
                f"the planning frame is {self.config.planning_frame!r}"
            )
        if not candidates.candidates:
            return SkillResult(f"Generated 0 grasp candidates for object {object_id}.")
        group = self._gripper_group(planning_group)
        start = self._manipulation.get_current_joint_state()
        if start is None:
            return SkillResult("No joint state yet; nothing was staged.")
        place = Vector3(x, y, z)
        last_failure = ""
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
            try:
                legs = self._plan_program(start, group, grasp, place)
            except _UnplannableLegError as exc:
                last_failure = str(exc)
                continue
            scanned = self._objects.get(object_id, {})
            program = _StagedProgram(
                object_id=object_id,
                planning_group=group,
                grasp=grasp,
                legs=legs,
                rank=rank,
                score=candidate.score,
                candidates=len(candidates.candidates),
                name=str(scanned.get("name", "")),
                start=Vector3(scanned["x"], scanned["y"], scanned["z"])
                if {"x", "y", "z"} <= set(scanned)
                else None,
            )
            self.grasp_target.publish(grasp)
            self._manipulation.preview_plans([leg.plan for leg in legs if leg.plan is not None])
            self._staged = program
            return SkillResult(
                f"Staged a pick of object {object_id} with {group} and a place at "
                f"({x:.2f}, {y:.2f}, {z:.2f}): {len(legs)} legs, {program.motion_seconds:.1f} s "
                f"of motion, grasp candidate {rank} of {program.candidates} "
                f"(score {candidate.score:.2f}). The viewer is playing the full motion. "
                "Nothing has moved; call proceed to run it or discard_staged to drop it.",
                metadata={
                    "object_id": object_id,
                    "planning_group": group,
                    "legs": [leg.label for leg in legs],
                    "motion_seconds": round(program.motion_seconds, 1),
                    "rank": rank,
                },
            )
        attempted = min(len(candidates.candidates), self.config.max_grasp_attempts)
        return SkillResult(
            f"Could not stage a pick of object {object_id}: none of the {attempted} grasp "
            f"candidate(s) tried plans all the way through; last failure: {last_failure}."
        )

    def _plan_program(
        self,
        start: JointState,
        group: PlanningGroupID,
        grasp: PoseStamped,
        place_position: Vector3,
    ) -> list[_StagedLeg]:
        """Plan every leg of a pick and place from *start*, chaining predicted states."""
        pregrasp = self._offset_pose(grasp, self._pregrasp_offset())
        legs: list[_StagedLeg] = [_StagedLeg("open the gripper", gripper="open")]
        state = start

        def planned(
            label: str,
            result: PlanResult,
            *,
            mode: Literal["pose", "linear", "joints"] | None = None,
            target: PoseStamped | None = None,
            joints: JointState | None = None,
        ) -> None:
            nonlocal state
            if result.plan is None or not result.plan.path:
                raise _UnplannableLegError(f"{label}: {_status(result)}")
            legs.append(
                _StagedLeg(label, plan=result.plan, mode=mode, target=target, joints=joints)
            )
            state = _merge_joint_state(state, result.plan.path[-1])

        def linear(label: str, a: PoseStamped, b: PoseStamped) -> None:
            planned(
                label,
                self._manipulation.plan_linear(
                    b.position.x - a.position.x,
                    b.position.y - a.position.y,
                    b.position.z - a.position.z,
                    group,
                    check_collision=False,
                    start=state,
                ),
                mode="linear",
                target=b,
            )

        planned(
            "approach above the object",
            self._manipulation.plan_to_poses({group: pregrasp}, start=state),
            mode="pose",
            target=pregrasp,
        )
        linear("descend to the grasp", pregrasp, grasp)
        legs.append(_StagedLeg("close and verify the grasp", gripper="close"))
        linear("lift", grasp, pregrasp)
        # The object's heading does not matter for a drop, so the wrist may turn
        # about vertical, and then tilt, until the carry and the lowering both plan.
        checkpoint = (len(legs), state)
        failures: list[str] = []
        for tilt in _PLACE_TILT_DELTAS:
            for delta in _PLACE_YAW_DELTAS:
                place = PoseStamped(
                    frame_id=self.config.planning_frame,
                    position=place_position,
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, delta))
                    * grasp.orientation
                    * Quaternion.from_euler(Vector3(0.0, tilt, 0.0)),
                )
                preplace = self._offset_pose(place, self._preplace_offset())
                try:
                    planned(
                        "carry above the place",
                        self._manipulation.plan_to_poses({group: preplace}, start=state),
                        mode="pose",
                        target=preplace,
                    )
                    linear("lower to the place", preplace, place)
                    break
                except _UnplannableLegError as exc:
                    failures.append(f"yaw {delta:+.2f} tilt {tilt:+.2f}: {exc}")
                    del legs[checkpoint[0] :]
                    state = checkpoint[1]
            else:
                continue
            break
        else:
            raise _UnplannableLegError("; ".join(failures[-2:]))
        legs.append(_StagedLeg("release", gripper="release"))
        linear("retreat", place, preplace)
        group_state = self._manipulation.get_state().groups.get(group)
        home = group_state.joint_presets.get("home") if group_state is not None else None
        if home is not None:
            planned(
                "return home",
                self._manipulation.plan_to_joints({group: home}, start=state),
                mode="joints",
                joints=home,
            )
        return legs

    def _replan_leg(self, group: PlanningGroupID, leg: _StagedLeg) -> GeneratedPlan | None:
        """Plan a staged leg again from the arm's live state, or None when it cannot."""
        if leg.mode == "pose" and leg.target is not None:
            result = self._manipulation.plan_to_poses({group: leg.target})
        elif leg.mode == "linear" and leg.target is not None:
            # get_state is an RPC; the tool pose getter is not, and this module
            # may run in another process from the manipulation module.
            group_state = self._manipulation.get_state().groups.get(group)
            current = group_state.end_effector_pose if group_state is not None else None
            if current is None:
                result = self._manipulation.plan_to_poses({group: leg.target})
            else:
                result = self._manipulation.plan_linear(
                    leg.target.position.x - current.position.x,
                    leg.target.position.y - current.position.y,
                    leg.target.position.z - current.position.z,
                    group,
                    check_collision=False,
                )
        elif leg.mode == "joints" and leg.joints is not None:
            result = self._manipulation.plan_to_joints({group: leg.joints})
        else:
            return None
        if result.plan is None or not result.plan.path:
            logger.warning(
                "Could not plan %s again from the live state: %s", leg.label, _status(result)
            )
            return None
        logger.info("Planned %s again from the live state", leg.label)
        return result.plan

    @skill(uses=[CAP_MOVEMENT])
    def proceed(self) -> SkillResult:
        """Run the staged pick and place leg by leg. Stops and reports at the
        first leg that fails."""
        program = self._staged
        if program is None:
            return SkillResult("Nothing is staged. Use stage_pick_and_place first.")
        self._staged = None
        group = program.planning_group
        jaw_note = ""
        for index, leg in enumerate(program.legs, 1):
            if leg.gripper == "open":
                if not_open := self._open_gripper(group, "before grasping"):
                    return not_open
            elif leg.gripper == "close":
                if self._lift_check_by_scan(program):
                    # The camera judges the grasp after the lift; the jaws only close.
                    settle = self._command_and_settle(
                        self.config.grasp_verification.closed_position, group
                    )
                    jaw_note = _gripper_reading(settle)
                elif not_held := self._close_and_verify(group, program.object_id):
                    return SkillResult(
                        f"Pick and place of object {program.object_id} STOPPED at leg {index} "
                        f"of {len(program.legs)} ({leg.label}): {not_held.message} The object "
                        "was not picked and the arm stayed at the grasp pose with the gripper "
                        "open.",
                        metadata={"stopped_at_leg": index, "legs": len(program.legs)},
                    )
                self._selected_object_id = program.object_id
                self._selected_grasp = program.grasp
                self._holding_object = True
            elif leg.gripper == "release":
                if not_open := self._open_gripper(group, "to release the object"):
                    return SkillResult(
                        f"Pick and place of object {program.object_id} STOPPED at leg {index} "
                        f"of {len(program.legs)} ({leg.label}): {not_open.message} The arm "
                        "stayed at the place pose.",
                        metadata={"stopped_at_leg": index, "legs": len(program.legs)},
                    )
                self._holding_object = False
                self._clear_selection()
            elif leg.plan is not None:
                execution = self._manipulation.execute_plan(leg.plan, blocking=True)
                if execution.status == ExecutionStatus.REJECTED:
                    # The arm settled short of the previous leg's end; plan this
                    # leg again from where it actually is.
                    replanned = self._replan_leg(group, leg)
                    if replanned is not None:
                        execution = self._manipulation.execute_plan(replanned, blocking=True)
                if not execution.succeeded:
                    return self._stopped(
                        f"Leg {index} of {len(program.legs)} ({leg.label}) of the staged "
                        f"pick and place of object {program.object_id}",
                        execution,
                    )
                if leg.label == "lift" and self._lift_check_by_scan(program):
                    still_there = self._object_still_at_start(program)
                    if still_there is not None:
                        self._holding_object = False
                        self._clear_selection()
                        return SkillResult(
                            f"Pick and place of object {program.object_id} STOPPED at leg "
                            f"{index} of {len(program.legs)} ({leg.label}): after the lift the "
                            f"camera still sees the {program.name or 'object'} on the table "
                            f"{still_there:.3f} m from where it was scanned, so the jaws "
                            f"missed or pushed it ({jaw_note}). The object was not picked; "
                            "the arm is at the lift pose with the gripper closed on nothing.",
                            metadata={"stopped_at_leg": index, "legs": len(program.legs)},
                        )
        return SkillResult(
            "Pick and place complete",
            metadata={
                "object_id": program.object_id,
                "legs": len(program.legs),
                "rank": program.rank,
                "score": program.score,
            },
        )

    @skill
    def forget_held_object(self) -> SkillResult:
        """Clear the record of a held object after it was dropped or never gripped,
        so a new pick can be staged. Nothing moves."""
        held = self._selected_object_id
        self._holding_object = False
        self._clear_selection()
        self._staged = None
        return SkillResult(
            f"Forgot held object {held}. Scan again before the next pick."
            if held
            else "No object was recorded as held."
        )

    @skill
    def discard_staged(self) -> SkillResult:
        """Drop the staged pick and place without moving."""
        if self._staged is None:
            return SkillResult("Nothing was staged.")
        self._staged = None
        self._manipulation.clear_planned_path()
        return SkillResult("Discarded the staged pick and place. Nothing moved.")

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
        self._staged = None
        group = self._gripper_group(planning_group)
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=self._selected_grasp.orientation,
        )
        preplace = self._offset_pose(place, self._preplace_offset())
        if blocked := self._move(preplace, group):
            return self._stopped(f"Move to the pre-place pose above {target}", blocked)
        if blocked := self._servo(preplace, place, group):
            return self._stopped(f"Move down to the place pose {target}", blocked)
        if not_open := self._open_gripper(group, "to release the object"):
            return SkillResult(f"{not_open.message} The arm stayed at the place pose.")
        self._holding_object = False
        self._clear_selection()
        if blocked := self._servo(place, preplace, group):
            return self._stopped(f"Retract from {target} after releasing", blocked)
        return SkillResult("Place complete")

    def _clear_selection(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._manipulation.show_grasp_proposals(GraspCandidateArray())
        self._selected_object_id = None
        self._selected_grasp = None

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

    def _pregrasp_offset(self) -> float:
        offset = self.config.pregrasp_offset
        return -offset if self.config.pregrasp_along_tool_z else offset

    def _preplace_offset(self) -> float:
        offset = self.config.preplace_offset
        if offset is None:
            return self._pregrasp_offset()
        return -offset if self.config.pregrasp_along_tool_z else offset

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

    def _lift_check_by_scan(self, program: _StagedProgram) -> bool:
        """Whether this staged job is judged by the camera after the lift."""
        return self.config.verify_lift_by_scan and program.start is not None

    def _object_still_at_start(self, program: _StagedProgram) -> float | None:
        """Rescan for the object; its distance from the scanned position when it is
        still on the table near there, else None.

        A lifted object leaves the table, so anything of its kind found within
        pushed_search_radius of the start and still at the scanned height was
        pushed aside by the jaws, not picked."""
        assert program.start is not None
        detections = self._scene.scan_scene(text=[program.name] if program.name else None)
        nearest: float | None = None
        for detection in detections.detections:
            position = detection.bbox.center.position
            distance = math.hypot(position.x - program.start.x, position.y - program.start.y)
            on_table = abs(position.z - program.start.z) <= self.config.grasp_displacement_tolerance
            if (
                distance <= self.config.pushed_search_radius
                and on_table
                and (nearest is None or distance < nearest)
            ):
                nearest = distance
        return nearest

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
            report = f"{report} {'Reopened the gripper.' if reopened is None else reopened.message}"
        return SkillResult(report)

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None
