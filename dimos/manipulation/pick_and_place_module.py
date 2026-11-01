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
from typing import Any

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
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
    ManipulationSpec,
    PickSceneSpec,
    PlanResult,
)
from dimos.manipulation.planning.gripper_contact import GraspContactChecker
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The one box a pick leaves out of the planner's map.
TARGET_BOX = "pick-and-place/target"
# The planning group of a base that may drive as part of a reach, where there is one.
MOVING_BASE = "moving_base"


class ArmReach(BaseConfig):
    """How far one planning group's arm gets on its own: a sphere about its shoulder.

    Args:
        radius: Metres from the shoulder frame to the wrist with the arm straight.
        shoulder_frame: The tf frame at the shoulder; None for the group's base frame.
        wrist_to_tip: Metres from the wrist to the group's tip frame along the tool's
            approach axis (+z of the tip frame); 0.0 when the tip frame is the wrist.
    """

    radius: float = Field(gt=0.0)
    shoulder_frame: str | None = None
    wrist_to_tip: float = Field(default=0.0, ge=0.0)


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # Per group, how far its arm reaches on its own. A pre-grasp that puts the
    # wrist beyond this sphere fails at once instead of after seconds of IK; empty
    # means no such check. Only the group's own joints count: with auxiliary
    # groups (a base that may drive) the check is off.
    reach: dict[PlanningGroupID, ArmReach] = Field(default_factory=dict)
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
    # The robot, for checking grasps before moving: a grasp the arm cannot reach,
    # or where the robot would touch the target with anything but the gripper's
    # pad links, open or as the fingers close, or touch another scene point, is
    # dropped. None, or a robot model without a gripper config, tries every grasp
    # the provider gives.
    robot: RobotModelConfig | None = None
    # Only the gripper's pads may come within about this distance (metres) of the target.
    grasp_contact_margin: float = Field(default=0.005, gt=0.0)
    # Nothing may come within about this distance (metres) of another scene point.
    grasp_obstacle_margin: float = Field(default=0.003, gt=0.0)
    # When no grasp passes the check where it is, slide the grasps up to this
    # far (metres) along their approach and try the nearest depth where any
    # passes. 0 is off.
    grasp_depth_search: float = Field(default=0.0, ge=0.0)


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


def _delta(start: PoseStamped, end: PoseStamped) -> tuple[float, float, float]:
    """The translation from ``start`` to ``end``, metres, in their common frame."""
    return (
        end.position.x - start.position.x,
        end.position.y - start.position.y,
        end.position.z - start.position.z,
    )


def _behind(pose: Pose, distance: float) -> Vector3:
    """The point ``distance`` metres back from ``pose`` along the tool's approach axis (+z)."""
    return pose.position + pose.orientation.rotate_vector(Vector3(0.0, 0.0, -distance))


@dataclass(frozen=True)
class ReachSphere:
    """One group's reach in the planning frame: how far its wrist gets from its shoulder."""

    group: PlanningGroupID
    frame: str
    shoulder: Vector3
    radius: float
    wrist_to_tip: float

    def wrist_distance(self, pose: Pose, standoff: float) -> float:
        """How far the wrist is from the shoulder with the tip ``standoff`` metres short of ``pose``."""
        return _behind(pose, standoff + self.wrist_to_tip).distance(self.shoulder)

    def beyond(self, poses: Sequence[Pose], standoff: float) -> float | None:
        """The nearest wrist distance when every one of ``poses`` is out of reach, else None.

        Args:
            poses: Tool-tip poses in the planning frame.
            standoff: Metres the tip stops short of each pose along its approach axis.
        """
        nearest = min(self.wrist_distance(pose, standoff) for pose in poses)
        return nearest if nearest > self.radius else None


class PickAndPlaceModule(Module):
    """Coordinate scene registration, grasp generation, and manipulation execution."""

    # Checking grasps for contact takes seconds; a module sharing this process,
    # such as ControlCoordinator, would stall for as long.
    dedicated_worker = True

    config: PickAndPlaceModuleConfig
    # Where the arms' shoulders are, for choosing an arm and for its reach.
    tf: In[TFMessage]
    _scene: PickSceneSpec
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
        # The grasp contact check, by planning group, built on first use.
        self._contact_checkers: dict[PlanningGroupID, GraspContactChecker] = {}
        self._holding_group: PlanningGroupID | None = None

    @rpc
    def start(self) -> None:
        # The buffer fills from now on; one first built during a pick has no transforms.
        self.tfbuffer  # noqa: B018
        super().start()

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
        self,
        object_id: str,
        planning_group: PlanningGroupID | None = None,
        auxiliary_groups: list[str] | None = None,
        approach_only: bool = False,
    ) -> SkillResult:
        """Generate ranked grasps and pick one object from the latest scan.

        ``contact_rejected`` in the result counts the grasps dropped because the
        arm cannot reach them, or the robot would touch the object off its finger
        pads or hit the scene.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted, the arm nearest the object.
            auxiliary_groups: Planning groups the planner may also move to reach the
                pre-grasp pose. Pass ["moving_base"] when the object is beyond the
                arm's reach and the base may drive. The descent onto the object and
                the lift are arm-only either way.
            approach_only: Stop at the pre-grasp pose with the gripper open, the descent
                planned but not driven, and report how far and which way to advance: for
                a grasp finished by hand. Nothing is picked.
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
        # The scene may register objects in a frame the robot moves through (odom), so a
        # base move does not stale them. The grasps, the reach check and the choice of arm
        # want the object in the planning frame; the exclusion box stays in the scene's
        # frame, on the object, where one in the planning frame would ride along with the
        # base.
        scene_frame = pointcloud.frame_id or None
        scene_box = _target_box(pointcloud)
        if scene_frame is not None and scene_frame != self.config.planning_frame:
            transform = self.tfbuffer.get(self.config.planning_frame, scene_frame, warn=False)
            if transform is None:
                return SkillResult(
                    f"No transform from the scene frame {scene_frame!r} to the planning frame "
                    f"{self.config.planning_frame!r} on tf; cannot locate object {object_id} "
                    "for the arm."
                )
            pointcloud = pointcloud.transform(transform)
        candidates = self._grasp_generator.propose_grasps(pointcloud)
        if candidates.header.frame_id != self.config.planning_frame:
            raise RuntimeError(
                f"Grasp candidates are in frame {candidates.header.frame_id!r}; "
                f"the planning frame is {self.config.planning_frame!r}"
            )
        box = _target_box(pointcloud)
        facts: dict[str, Any] = {}
        group: PlanningGroupID | None = None
        centre = box.center if box is not None else None
        if candidates.candidates:
            # Where the object is, in the planning frame: its cloud's centre when the
            # cloud is in that frame, else the best grasp, which sits on it.
            in_planning_frame = (
                not pointcloud.frame_id or pointcloud.frame_id == self.config.planning_frame
            )
            if centre is None or not in_planning_frame:
                centre = candidates.candidates[0].pose.position
            group = self._gripper_group(planning_group, centre)
            # The contact check poses the chosen arm's gripper on each grasp.
            candidates, facts = self._drop_touching(object_id, pointcloud, candidates, group)
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        if group is None or centre is None or not candidates.candidates:
            note = ""
            if rejected := facts.get("contact_rejected"):
                note = (
                    f" The contact check dropped {rejected} grasp(s) the arm cannot reach or "
                    "where the robot would touch the object off its finger pads or hit the scene."
                )
            return SkillResult(
                f"Generated 0 grasp candidates for object {object_id}.{note}", metadata=facts
            )
        auxiliary = self._auxiliary_groups(auxiliary_groups, group)
        # With the arm alone moving, a pre-grasp its wrist cannot get to is known
        # now; asking IK takes seconds per candidate to find out.
        sphere = None if auxiliary else self._reach_sphere(group)
        if sphere is not None:
            attempts = [c.pose for c in candidates.candidates[: self.config.max_grasp_attempts]]
            wrist = sphere.beyond(attempts, self.config.pregrasp_offset)
            if wrist is not None:
                return SkillResult(
                    self._beyond_reach(
                        f"Object {object_id}", centre, "its closest pre-grasp pose", sphere, wrist
                    ),
                    metadata={
                        **facts,
                        "object_id": object_id,
                        "planning_group": group,
                        "object_distance": sphere.shoulder.distance(centre),
                        "wrist_distance": wrist,
                        "reach": sphere.radius,
                    },
                )
        # The target is mapped geometry: with its cells as obstacles the planner
        # refuses the approach into it, and the held object blocks every move after.
        if scene_box is not None:
            self._leave_out(Pose(position=scene_box.center), scene_box.size, scene_frame)
        try:
            result = self._pick_from(
                candidates, object_id, box, group, auxiliary, sphere, approach_only
            )
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
        """Drop the grasps the arm cannot reach, that touch the target off the pads, or hit the scene.

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
        states = self._manipulation.get_state().groups.values()
        tips = [s.end_effector_pose.position for s in states if s.end_effector_pose is not None]
        joints = {
            name: position
            for s in states
            if s.joints is not None
            for name, position in zip(s.joints.name, s.joints.position, strict=True)
        }
        try:
            if group not in self._contact_checkers:
                self._contact_checkers[group] = GraspContactChecker(robot, group)
            kept = self._contact_checkers[group].keep_clear(
                candidates,
                pointcloud,
                scene,
                tips,
                self.config.grasp_contact_margin,
                self.config.grasp_obstacle_margin,
                JointState(name=list(joints), position=list(joints.values())),
                self.config.grasp_depth_search,
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
        auxiliary: Sequence[PlanningGroupID] = (),
        sphere: ReachSphere | None = None,
        approach_only: bool = False,
    ) -> SkillResult:
        """Open the gripper and try the ranked grasps in order until one holds the object.

        Args:
            candidates: Ranked grasps in the planning frame, best first.
            object_id: ID of the object being picked, for the report.
            box: Bounds of the object's point cloud, or None when it has none.
            group: Gripper-capable planning group doing the pick.
            auxiliary: Groups the planner may move as well on the way to each
                pre-grasp pose; never on the straight legs at the object.
            sphere: The group's reach, when known; candidates beyond it are not planned.
            approach_only: Stop at the first pre-grasp reached, jaws open, instead of
                grasping; the report says how to finish by hand.
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
            if sphere is not None and sphere.beyond([grasp], self.config.pregrasp_offset):
                # Known unreachable; IK would take seconds to agree.
                continue
            blocked = self._approach(
                pregrasp, grasp, group, auxiliary, "the descent from the pre-grasp pose"
            )
            if blocked is None and approach_only:
                return self._at_pregrasp(object_id, group, pregrasp, grasp)
            if blocked is None:
                blocked = self._servo(pregrasp, grasp, group)
            if isinstance(blocked, PlanResult):
                # The planner found no path to this candidate, or no descent from
                # its pre-grasp; the next one may differ. A motion that stopped
                # part-way would stop the same way for every candidate, so that is
                # not retried.
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
            self._holding_group = group
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

    def _at_pregrasp(
        self,
        object_id: str,
        group: PlanningGroupID,
        pregrasp: PoseStamped,
        grasp: PoseStamped,
    ) -> SkillResult:
        """Report the arm waiting at a pre-grasp, jaws open, for a grasp finished by hand.

        Args:
            object_id: ID of the object the pre-grasp is above, for the report.
            group: The group whose tool tip is at ``pregrasp``.
            pregrasp: Where the tip is, in the planning frame.
            grasp: Where it would close on the object: straight ahead along the tool's
                approach axis, which is the way to advance.
        """
        approach = grasp.position - pregrasp.position
        distance = pregrasp.position.distance(grasp.position)
        return SkillResult(
            f"At the pregrasp for {object_id} with {group}, gripper open: advance "
            f"{100.0 * distance:.0f} cm along the grasp axis to grasp.",
            metadata={
                "object_id": object_id,
                "planning_group": group,
                "approach_m": distance,
                "approach_dir": (approach / distance).to_list(),
                "grasp": {
                    "frame_id": grasp.frame_id,
                    "position": grasp.position.to_list(),
                    "orientation": grasp.orientation.to_list(),
                },
            },
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
        auxiliary_groups: list[str] | None = None,
    ) -> SkillResult:
        """Place the held object at an explicit planning-frame position.

        Args:
            x: Planning-frame X coordinate in meters.
            y: Planning-frame Y coordinate in meters.
            z: Planning-frame Z coordinate in meters.
            planning_group: Gripper-capable pose group; omitted, the arm holding the object.
            auxiliary_groups: Planning groups the planner may also move to reach the
                pre-place pose above the target. Pass ["moving_base"] when the spot
                is beyond the arm's reach and the base may drive. The descent to the
                place pose and the retreat are arm-only either way.
        """
        target = f"({x:.2f}, {y:.2f}, {z:.2f})"
        if self._selected_grasp is None or not self._holding_object:
            return SkillResult(
                f"Not holding any object; nothing was placed at {target}. Use pick_object first."
            )
        group = self._gripper_group(planning_group or self._holding_group, Vector3(x, y, z))
        auxiliary = self._auxiliary_groups(auxiliary_groups, group)
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=self._selected_grasp.orientation,
        )
        sphere = None if auxiliary else self._reach_sphere(group)
        if sphere is not None and (wrist := sphere.beyond([place], self.config.pregrasp_offset)):
            return SkillResult(
                self._beyond_reach(
                    f"Place target {target}", place.position, "the pre-place pose", sphere, wrist
                )
                + f" Still holding object {self._selected_object_id}.",
                metadata={
                    "planning_group": group,
                    "target_distance": sphere.shoulder.distance(place.position),
                    "wrist_distance": wrist,
                    "reach": sphere.radius,
                },
            )
        preplace = self._offset_pose(place, self.config.pregrasp_offset)
        if blocked := self._approach(
            preplace, place, group, auxiliary, "the descent to the place pose"
        ):
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
        self._holding_group = None

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

    def _base_frame(self, group: PlanningGroupID) -> str:
        """The frame a planning group's chain starts from."""
        return next(
            info.base_frame
            for info in self._manipulation.list_planning_groups()
            if info.id == group
        )

    def _gripper_group(
        self, planning_group: PlanningGroupID | None, target: Vector3 | None = None
    ) -> PlanningGroupID:
        """Pick the planning group that has both a gripper and a tool frame.

        Args:
            planning_group: Group ID to use, or None for the one nearest ``target``.
            target: Planning-frame point the group is to reach; with several
                gripper-capable groups, the one whose arm is nearest does it.

        Raises ValueError when the ID names no such group, or when it is omitted
        and no single group stands out.
        """
        groups = [
            group.id
            for group in self._manipulation.list_planning_groups()
            if group.has_gripper and group.tip_frame is not None
        ]
        if planning_group is None:
            if len(groups) == 1:
                return groups[0]
            nearest = self._nearest_group(groups, target) if target is not None else None
            if nearest is not None:
                return nearest
            raise ValueError(
                "planning_group is omitted and there is no single gripper-capable planning "
                f"group to use; found {groups}"
            )
        if planning_group not in groups:
            raise ValueError(
                f"planning_group {planning_group!r} is not gripper-capable; "
                f"gripper-capable groups: {groups}"
            )
        return planning_group

    def _nearest_group(
        self, groups: Sequence[PlanningGroupID], target: Vector3
    ) -> PlanningGroupID | None:
        """The group whose arm is nearest ``target``, or None when no arm's position is known.

        An arm is where its shoulder is when its reach is configured and the
        shoulder is on tf, else where its tool tip is now.
        """
        snapshot = self._manipulation.get_state()
        distances: dict[PlanningGroupID, float] = {}
        for group in groups:
            shoulder = self._shoulder(group)
            if shoulder is not None:
                distances[group] = shoulder[1].distance(target)
                continue
            state = snapshot.groups.get(group)
            if state is not None and state.end_effector_pose is not None:
                distances[group] = state.end_effector_pose.position.distance(target)
        return min(distances, key=distances.__getitem__) if distances else None

    def _shoulder(self, group: PlanningGroupID) -> tuple[str, Vector3] | None:
        """A group's shoulder frame and where it is in the planning frame.

        None when the group's reach is not configured, or tf has no transform to
        the frame yet.
        """
        reach = self.config.reach.get(group)
        if reach is None:
            return None
        frame = reach.shoulder_frame or self._base_frame(group)
        transform = self.tfbuffer.get(self.config.planning_frame, frame, warn=False)
        return None if transform is None else (frame, transform.translation)

    def _reach_sphere(self, group: PlanningGroupID) -> ReachSphere | None:
        """The group's reach about its shoulder, or None when it is not configured.

        Also None, with a warning, when the shoulder frame is not on tf: the
        check cannot be made, and the planner is left to find out.
        """
        reach = self.config.reach.get(group)
        if reach is None:
            return None
        shoulder = self._shoulder(group)
        if shoulder is None:
            logger.warning(
                "No transform from %s to the %s shoulder frame %r; its reach goes unchecked",
                self.config.planning_frame,
                group,
                reach.shoulder_frame or self._base_frame(group),
            )
            return None
        frame, origin = shoulder
        return ReachSphere(group, frame, origin, reach.radius, reach.wrist_to_tip)

    def _beyond_reach(
        self, subject: str, target: Vector3, pose: str, sphere: ReachSphere, wrist: float
    ) -> str:
        """One sentence saying how far beyond an arm a target is, and what to do about it.

        Args:
            subject: What is out of reach, e.g. "Object cup-1".
            target: Its position in the planning frame.
            pose: The tool pose that was to be reached, e.g. "the pre-place pose".
            sphere: The arm's reach.
            wrist: How far from the shoulder the wrist would have to be.
        """
        ids = [info.id for info in self._manipulation.list_planning_groups()]
        hint = (
            f"drive closer or pass auxiliary_groups=['{MOVING_BASE}']"
            if MOVING_BASE in ids
            else "bring it closer"
        )
        return (
            f"{subject} is {sphere.shoulder.distance(target):.2f} m from the {sphere.group} "
            f"shoulder ({sphere.frame}) and {pose} needs the wrist {wrist:.2f} m out, beyond "
            f"that arm's {sphere.radius:.2f} m reach: {hint}."
        )

    def _auxiliary_groups(
        self, auxiliary_groups: Sequence[PlanningGroupID] | None, group: PlanningGroupID
    ) -> tuple[PlanningGroupID, ...]:
        """The groups the planner may move besides the one doing the pick.

        Args:
            auxiliary_groups: Group IDs from the caller, or None for none.
            group: The gripper-capable group doing the pick; it is not auxiliary.

        Raises ValueError when an ID names no planning group, or names ``group``.
        """
        if not auxiliary_groups:
            return ()
        known = [info.id for info in self._manipulation.list_planning_groups()]
        unknown = [group_id for group_id in auxiliary_groups if group_id not in known]
        if unknown:
            raise ValueError(
                f"auxiliary_groups {unknown} are not planning groups; planning groups: {known}"
            )
        if group in auxiliary_groups:
            raise ValueError(
                f"auxiliary_groups must not include the group doing the pick, {group!r}"
            )
        return tuple(dict.fromkeys(auxiliary_groups))

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
            position=_behind(pose, offset),
            orientation=pose.orientation,
        )

    def _servo(
        self, start: PoseStamped, end: PoseStamped, planning_group: PlanningGroupID
    ) -> PlanResult | ExecutionResult | None:
        """Drive the last leg as a straight line with collision checking off.

        The object being grasped is itself mapped geometry once a voxel map feeds
        the planner, so a collision-checked plan into it can only ever be
        rejected. This leg is short, straight, and deliberately ends in contact.
        Only the group's own joints move: a base that drove in for the approach
        stays put while the hand is at the object.

        Returns the planner or execution result that stopped the leg, or None
        when the arm arrived.
        """
        result = self._manipulation.move_linear(
            *_delta(start, end), planning_group, check_collision=False
        )
        if not result.plan.succeeded:
            return result.plan
        if result.execution is None:
            raise RuntimeError("Linear move was planned but never executed")
        return None if result.execution.succeeded else result.execution

    def _approach(
        self,
        pose: PoseStamped,
        then: PoseStamped,
        planning_group: PlanningGroupID,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        leg: str = "the straight leg after it",
    ) -> PlanResult | ExecutionResult | None:
        """Plan a collision-checked path to ``pose`` and run it, once the straight leg on
        to ``then`` is known to be plannable from where that path ends.

        An approach that may drive the base can leave the arm somewhere the hand cannot
        go straight on from (a joint at its limit, a singularity). Found out after the
        approach, that strands the arm over the object; so the leg is planned first,
        from the approach's goal joints, without moving anything.

        Args:
            pose: Where the group's tool tip goes, in the planning frame.
            then: Where the tip goes next, in a straight line, with the group's own
                joints alone.
            planning_group: The group whose tip reaches ``pose``.
            auxiliary_groups: Groups the planner may move as well, such as a mobile
                base, each relative to where it stands at plan time.
            leg: What the straight leg is, for the report, e.g. "the descent".

        Returns the planner result that found no path (to ``pose``, or on to ``then``
        from where the approach ends), the execution result that stopped the approach,
        or None when the arm arrived.
        """
        plan = self._manipulation.plan_to_poses(
            {planning_group: pose}, auxiliary_groups=auxiliary_groups
        )
        if not plan.succeeded:
            return plan
        if plan.plan is None or not plan.plan.path:
            raise RuntimeError("The approach was planned but came back without a path")
        check = self._manipulation.plan_linear(
            *_delta(pose, then),
            planning_group,
            check_collision=False,
            start_joints=plan.plan.path[-1],
        )
        if not check.succeeded:
            return PlanResult(
                check.status,
                f"{leg} is not plannable from where the approach ends: {check.message}",
            )
        execution = self._manipulation.execute(blocking=True, plan_id=plan.plan.plan_id)
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
