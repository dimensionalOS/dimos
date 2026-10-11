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
from dataclasses import dataclass, field
import math
import time
from typing import Any, Literal

import numpy as np
from pydantic import Field
from scipy.spatial import cKDTree

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
from dimos.manipulation.grasping.contact_filter import add_facts, check_in_scene, contact_note
from dimos.manipulation.grasping.grasp_fallback import (
    GraspProposals,
    SceneCheck,
    SourcedGrasp,
    grasp_sources,
    propose_with_fallback,
    same_grasp,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, SceneGraspGenSpec
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspConfig
from dimos.manipulation.manipulation_spec import ExecutionResult, ManipulationSpec, PlanResult
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec

# The one box of mapped cells a pick asks the planner to ignore.
VOXEL_MAP_EXCLUSION_ID = "pick-and-place/target"
# How often (seconds) a check waiting for a new camera frame asks again.
_FRESH_FRAME_POLL = 0.1
# Joints this close (radians) to the scan pose count as being there already.
_AT_SCAN_POSE_TOLERANCE = 0.01
# How far (metres) beyond the target's points the scene check looks: far enough
# for the fingers, knuckles and most of the palm of the xArm gripper, which
# reaches 0.18 m back from its tool tip.
_SCENE_MARGIN = 0.15
# Scene points this close (metres) to the target's own points are the target.
_TARGET_POINT_RADIUS = 0.01
# Scene points this close (metres) to a gripper's tool tip are that gripper: the
# wrist camera sees its own fingers.
_GRIPPER_SELF_RADIUS = 0.15


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # Margin (metres) added around the target's bounding box. Mapped cells in
    # that box are not obstacles during the pick, and the same box rides with
    # the gripper while the object is held. Too small, and a cell on the
    # object's edge or the table under it stops the plan; too large, and a
    # neighbour's cells are ignored too.
    target_clearance: float = Field(default=0.03, gt=0.0)
    # Most grasp candidates each try plans to from each source: the grasp
    # provider's best, then the heuristic's. Many of a learned provider's
    # best-scoring poses are out of the arm's reach; the planner turns each of
    # those down in about 0.3 s.
    max_grasp_attempts: int = Field(default=25, gt=0)
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)
    # After the lift, the camera must see the object rise by at least this share
    # (0 to 1) of the height the gripper rose, else it stayed down.
    lift_follow_fraction: float = Field(default=0.5, gt=0.0, le=1.0)
    # Detections of the target's label farther than this (metres) from where it
    # was last seen (and, after a lift, from where it would be had it come along)
    # are other objects. A released object is looked for this far from the gripper.
    lift_check_radius: float = Field(default=0.15, gt=0.0)
    # Grasp tries on one object, counting the first. After a verified miss the
    # arm backs out, scans the object again and tries a grasp not tried yet.
    max_grasp_retries: int = Field(default=3, gt=0)
    # A new candidate within both of these of a grasp already tried counts as the
    # same grasp and is skipped: distance (metres) between the grasp positions
    # relative to the object's centre, and angle (radians) between orientations.
    retry_skip_distance: float = Field(default=0.02, ge=0.0)
    retry_skip_angle: float = Field(default=0.35, ge=0.0)
    # How far (metres, horizontally) a released object may rest from the place target.
    place_tolerance: float = Field(default=0.07, gt=0.0)
    # How long (seconds) a check after a lift, a miss or a release waits for a
    # camera frame taken after the arm stopped. A scan can take a second or more.
    fresh_frame_timeout: float = Field(default=8.0, gt=0.0)
    # Also try simple top-down heuristic grasps after the provider's, for when it
    # gives none, raises, or its grasps cannot be reached or miss. False uses the
    # provider's grasps only.
    heuristic_fallback: bool = True
    # How those heuristic grasps fit the gripper.
    heuristic_grasp: HeuristicGraspConfig = Field(default_factory=HeuristicGraspConfig)
    # Drop the grasp provider's grasps whose open gripper, at the grasp pose,
    # would overlap the table or another object in the camera's view. Works with
    # a provider that can check against a scene (GraspGenX), which models the
    # gripper as boxes. Off, the contact check (grasp_contact_check) checks the
    # table and other objects with the gripper's real shape instead. Heuristic
    # grasps are never checked here.
    grasp_scene_filter: bool = False
    # A gripper part closer than this (metres) to a scene point is a collision.
    # The scene cloud has one point per 1 cm cell, so less than about 1 cm lets a
    # finger slip between two table points unseen; much more turns down grasps
    # on short objects, whose fingertips must come close to the table.
    scene_collision_threshold: float = Field(default=0.01, gt=0.0)
    # Drop grasps, from every source, whose gripper would touch the target
    # anywhere but its contact patches (the finger pads), at the open pose or as
    # the fingers close, or would overlap the table or another object. Needs a
    # gripper spec in the robot model; without one the pick skips the check and
    # says so. False tries every grasp the sources give.
    grasp_contact_check: bool = True
    # A gripper part closer than this (metres) to the target touches it.
    grasp_contact_margin: float = Field(default=0.005, gt=0.0)
    # A gripper part closer than this (metres) to a table or object point overlaps it.
    grasp_obstacle_margin: float = Field(default=0.003, gt=0.0)
    # With the contact check, also drop grasps that would close on nothing:
    # fewer than grasp_min_gap_points target points (one per 5 mm cube) in the
    # gap the finger pads close across.
    grasp_require_object_in_gap: bool = True
    grasp_min_gap_points: int = Field(default=10, gt=0)
    # With the contact check, a grasp that fails where its source put it may
    # slide up to this far (metres) along its approach, either way, to the
    # nearest depth where the gripper's modelled close passes and the finger
    # pads hold the target. Moved grasps are tried only when no grasp passes
    # unmoved. 0 checks each grasp only where it is.
    grasp_depth_search: float = Field(default=0.0, ge=0.0)
    # Spacing (metres) of the depths tried.
    grasp_depth_step: float = Field(default=0.005, gt=0.0)


@dataclass(frozen=True)
class ScannedObject:
    """One object from the latest scan.

    Args:
        object_id: ID the scene registration gave it.
        name: Detected label, e.g. "cup".
        position: Centre of its 3D box in the planning frame, metres.
        orientation: Orientation of that box in the planning frame.
        size: Edge lengths of that box along its own axes, metres.
    """

    object_id: str
    name: str
    position: Vector3
    orientation: Quaternion
    size: Vector3


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


@dataclass(frozen=True)
class _TriedGrasp:
    """A grasp pick_object already tried on the current object.

    Args:
        offset: Grasp position minus the object's centre at the time, metres.
        orientation: Grasp orientation, planning frame.
    """

    offset: Vector3
    orientation: Quaternion


@dataclass(frozen=True)
class _GraspTry:
    """What one grasp try did.

    Args:
        result: The report of the try.
        candidate: The grasp the arm went for, or None when it went for none.
        missed: True when the grasp verifiably missed, the arm backed out to the
            pre-grasp pose and the gripper was told to open, so another try may follow.
        rejected: Per source, how many grasps the planner found no path to.
    """

    result: SkillResult
    candidate: SourcedGrasp | None = None
    missed: bool = False
    rejected: dict[str, int] = field(default_factory=dict)


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
    # The grasp provider again when it can check its grasps against the scene.
    _scene_grasp_generator: SceneGraspGenSpec | None = None
    _manipulation: ManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, ScannedObject] = {}
        self._grasp_candidates = GraspCandidateArray()
        self._held: HeldObject | None = None
        # True while a pick has left the gripper inside the target's ignored box;
        # the box then stays so the next plan can start from there.
        self._arm_in_target = False
        # Joint positions of each planning group when the latest scan_objects ran.
        self._scan_joints: dict[PlanningGroupID, JointState] = {}
        # What the grasp contact check did over the current pick (see drop_touching).
        self._contact: dict[str, Any] = {}

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
        self._scan_joints = {
            group_id: state.joints
            for group_id, state in self._manipulation.get_state().groups.items()
            if state.joints is not None
        }
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

    @rpc
    def scene_grasp_counts(self, object_id: str, num_samples: int) -> dict[str, Any]:
        """Count how many grasps on a scanned object stay clear of the scene.

        Asks the grasp provider for grasps with the scene check, from the same
        object and scene points pick_object would use, but moves nothing and
        keeps nothing. For measuring the provider; not an agent skill.

        Args:
            object_id: Object ID from the latest scan_objects.
            num_samples: How many grasps the provider samples for this call.

        Returns ``num_samples``; ``sampled`` (grasps proposed), ``clear`` (left
        after the scene check), ``clear_in_top`` (clear among the best-scored
        grasps the provider keeps), ``kept`` (grasps returned); ``infer_s``,
        ``check_s`` and ``call_s`` (the whole call), seconds; and the object's
        and scene's point counts. Holds ``error`` instead when the check could
        not run.
        """
        report: dict[str, Any] = {"num_samples": num_samples}
        target = self._objects.get(object_id)
        if target is None:
            return {**report, "error": f"no object {object_id} in the latest scan"}
        pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
        if pointcloud is None:
            return {**report, "error": f"object {object_id} has no point cloud"}
        check, skipped = self._scene_check(target, pointcloud)
        if check is None:
            return {**report, "error": skipped or "the scene check is off or unsupported"}
        started = time.monotonic()
        try:
            result = check.provider.propose_grasps_in_scene(
                pointcloud, check.scene, check.collision_threshold, num_samples=num_samples
            )
        except Exception as exc:
            return {**report, "error": f"{type(exc).__name__}: {exc}"}
        return {
            **report,
            "sampled": result.sampled,
            "clear": result.sampled - result.removed,
            "clear_in_top": result.clear_in_top,
            "kept": len(result.grasps.candidates),
            "infer_s": round(result.infer_s, 3),
            "check_s": round(result.check_s, 3),
            "call_s": round(time.monotonic() - started, 3),
            "object_points": len(pointcloud.points_f32()),
            "scene_points": len(check.scene.points_f32()),
        }

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> SkillResult:
        """Grasp one object from the latest scan and lift it.

        After the lift the camera checks the object came up with the gripper. On
        a miss the arm backs out, returns to where scan_objects saw the object,
        scans again and tries another grasp, a limited number of times. ``tries``
        in the result lists each grasp and its ``source``; ``handoff: true`` means
        it stopped trying after misses and the object is not held.
        ``grasp_sources`` names where the grasps came from; ``contact_rejected``
        counts the grasps dropped for touching the object off the finger pads
        (``target``), hitting the ``table`` or other ``objects``, or closing on
        nothing (``empty_gap``).

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        result = self._pick(object_id, planning_group)
        result.metadata.setdefault(
            "grasp_sources", grasp_sources(self._grasp_generator, self.config.heuristic_fallback)
        )
        if self._scene_grasp_generator is not None and not self.config.grasp_scene_filter:
            result.metadata.setdefault("scene_filter_skipped", "off")
        if "skipped" in self._contact:
            result.metadata.setdefault("contact_check_skipped", self._contact["skipped"])
        elif self._contact:
            result.metadata.setdefault("contact_rejected", dict(self._contact))
        return result

    def _pick(self, object_id: str, planning_group: PlanningGroupID | None) -> SkillResult:
        """Do the work of pick_object, which then adds the grasp_sources fact."""
        if self._held is not None:
            return SkillResult(
                f"Still holding object {self._held.object_id}; "
                f"did not start a pick of object {object_id}. Use place_at to put it down first."
            )
        self._clear_proposals()
        self._contact = {}
        target = self._objects.get(object_id)
        if target is None:
            scanned = ", ".join(self._objects) or "none"
            return SkillResult(
                f"No object with id {object_id} in the latest scan. Scanned ids: {scanned}. "
                "Use scan_objects to refresh the list."
            )
        group = self._gripper_group(planning_group)
        proposal = self._propose(target, group)
        if isinstance(proposal, SkillResult):
            return proposal
        proposals, box = proposal
        # The target is mapped geometry: with its cells as obstacles the planner
        # refuses the approach into it, and the held object blocks every move after.
        self._exclude_target(box)
        self._arm_in_target = False
        try:
            return self._pick_from(proposals, target, box, group)
        finally:
            if self._held is None and not self._arm_in_target:
                self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)

    def _propose(
        self, target: ScannedObject, group: PlanningGroupID, crop: bool = False
    ) -> tuple[GraspProposals, TargetBox | None] | SkillResult:
        """Generate ranked grasps for an object and show them in the viewer.

        The provider's grasps come first, then, with ``heuristic_fallback``,
        heuristic grasps from the same points. With ``grasp_contact_check``,
        grasps whose gripper would touch the object off its pads or hit the
        scene are dropped.

        Args:
            target: The object to grasp, from the latest scan.
            group: Gripper-capable planning group doing the pick.
            crop: Keep only the points inside the target's detected box (plus
                ``target_clearance``). The scene registration merges a new view
                into the old cloud, so after a push the cloud holds both places.

        Returns the candidates and the bounds of the points used, or a statement
        of why there are none.
        """
        pointcloud = self._scene.get_object_pointcloud_by_object_id(target.object_id)
        if pointcloud is None:
            return SkillResult(
                f"Object {target.object_id} has no point cloud in the latest scan. "
                "Use scan_objects again."
            )
        if crop:
            pointcloud = _crop(pointcloud, target, self.config.target_clearance)
            if not len(pointcloud.points_f32()):
                return SkillResult(
                    f"Object {target.object_id} has no points inside its detected box."
                )
        scene, scene_skipped = self._scene_check(target, pointcloud)
        proposals = propose_with_fallback(
            self._grasp_generator,
            pointcloud,
            self.config.heuristic_fallback,
            self.config.heuristic_grasp,
            scene,
            scene_skipped,
        )
        proposals = self._drop_touching(target, pointcloud, proposals, group)
        candidates = proposals.all()
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        proposals.check_frame(self.config.planning_frame)
        if not candidates.candidates:
            return SkillResult(
                f"Generated 0 grasp candidates for object {target.object_id}.{proposals.note()}"
                f"{contact_note(self._contact)}",
                metadata=proposals.facts(),
            )
        return proposals, _target_box(pointcloud)

    def _scene_check(
        self, target: ScannedObject, pointcloud: PointCloud2
    ) -> tuple[SceneCheck | None, str | None]:
        """How to check the provider's grasps against the scene around the target.

        The scene is the latest scan's camera cloud with the target masked out,
        cut to the target's surroundings (see ``_scene_near``).

        Args:
            target: The object to grasp.
            pointcloud: The target's points, planning frame, metres.

        Returns the check, or None and why it does not run ("off" when
        ``grasp_scene_filter`` is off). Both are None when the provider cannot
        check grasps against a scene.
        """
        provider = self._scene_grasp_generator
        if provider is None:
            return None, None
        if not self.config.grasp_scene_filter:
            return None, "off"
        try:
            scene = self._scene.get_full_scene_pointcloud(exclude_object_id=target.object_id)
        except Exception as exc:
            return None, f"the scene point cloud failed: {type(exc).__name__}: {exc}"
        if scene is None:
            return None, "the scene registration gave no scene point cloud"
        if scene.frame_id != pointcloud.frame_id:
            return None, (
                f"the scene point cloud is in frame {scene.frame_id!r}, "
                f"the object's in {pointcloud.frame_id!r}"
            )
        tips = [
            state.end_effector_pose.position
            for state in self._manipulation.get_state().groups.values()
            if state.end_effector_pose is not None
        ]
        nearby = _scene_near(scene, pointcloud, tips)
        return SceneCheck(provider, nearby, self.config.scene_collision_threshold), None

    def _drop_touching(
        self,
        target: ScannedObject,
        pointcloud: PointCloud2,
        proposals: GraspProposals,
        group: PlanningGroupID,
    ) -> GraspProposals:
        """Keep the grasps whose gripper touches the target only at its finger pads.

        See ``contact_filter.check_in_scene``; what it did adds to ``_contact``.

        Args:
            target: The object to grasp.
            pointcloud: The target's points, planning frame, metres.
            proposals: Grasps from each source.
            group: Planning group whose tool tip the grasps pose.
        """
        if not self.config.grasp_contact_check:
            self._contact = {"skipped": "off"}
            return proposals
        kept, facts = check_in_scene(
            self._manipulation,
            self._scene,
            target.object_id,
            pointcloud,
            proposals,
            self.config.grasp_contact_margin,
            self.config.grasp_obstacle_margin,
            group,
            self.config.grasp_min_gap_points if self.config.grasp_require_object_in_gap else 0,
            self.config.grasp_depth_search,
            self.config.grasp_depth_step,
            # The pick tries at most this many grasps per scan: the depth
            # search can stop once that many pass.
            self.config.max_grasp_attempts,
        )
        add_facts(self._contact, facts)
        return kept

    def _pick_from(
        self,
        proposals: GraspProposals,
        target: ScannedObject,
        box: TargetBox | None,
        group: PlanningGroupID,
    ) -> SkillResult:
        """Grasp the object, and after each verified miss scan again and try a new grasp.

        Up to ``max_grasp_retries`` tries. A candidate from either source that
        repeats a grasp already tried on this object is skipped.

        Args:
            proposals: Grasps in the planning frame from each source.
            target: The object being picked, with where it was scanned.
            box: Bounds of the object's point cloud, or None when it has none.
            group: Gripper-capable planning group doing the pick.

        Returns the last try's report with ``tries`` (one record per grasp),
        ``handoff`` (true when it stopped trying after misses),
        ``rejected_by_source`` (per source, grasps the planner found no path to
        over the whole pick) and ``scene_filtered_by_source`` (per source, grasps
        dropped by the scene check over the whole pick) in its metadata.
        """
        tries: list[dict[str, Any]] = []
        tried: list[_TriedGrasp] = []
        rejected = dict.fromkeys(proposals.sources, 0)
        scene_filtered = dict(proposals.scene_filtered)
        ranked, skipped = self._untried(proposals, target, tried)
        while True:
            attempt = self._grasp_once(
                proposals.all(), proposals.sources, ranked, target, box, group
            )
            for source, count in attempt.rejected.items():
                rejected[source] += count
            if attempt.candidate is None:
                last = attempt.result
                facts = {**proposals.facts(), **last.metadata}
                return self._stop_trying(
                    SkillResult(last.message, metadata=facts), tries, rejected, scene_filtered
                )
            choice = attempt.candidate
            candidate = choice.candidate
            facts = proposals.facts()
            record = {
                "try": len(tries) + 1,
                "object_id": target.object_id,
                "source": choice.source,
                "rank": choice.rank,
                "score": candidate.score,
                "skipped": skipped,
                "grasped": attempt.result.metadata.get("grasped"),
                "candidates_by_source": facts["candidates_by_source"],
                "scene_filtered_by_source": facts["scene_filtered_by_source"],
                "rejected_by_source": attempt.rejected,
            }
            for fact in ("grasp_provider_error", "scene_filter_skipped"):
                if fact in facts:
                    record[fact] = facts[fact]
            tries.append(record)
            tried.append(
                _TriedGrasp(candidate.pose.position - target.position, candidate.pose.orientation)
            )
            last = attempt.result
            if not attempt.missed:
                return self._stop_trying(last, tries, rejected, scene_filtered)
            if len(tries) >= self.config.max_grasp_retries:
                return self._stop_trying(
                    last,
                    tries,
                    rejected,
                    scene_filtered,
                    f"Object {target.object_id} is still not held after {len(tries)} "
                    f"{'try' if len(tries) == 1 else 'tries'}.",
                )
            if moved := self._return_to_scan_pose(group):
                stopped = self._stopped(
                    f"Move back to where object {target.object_id} was scanned from", moved
                )
                return self._stop_trying(
                    SkillResult(f"{last.message} {stopped.message}", metadata=last.metadata),
                    tries,
                    rejected,
                    scene_filtered,
                )
            found = self._rescan(target)
            if isinstance(found, str):
                return self._stop_trying(last, tries, rejected, scene_filtered, found)
            proposal = self._propose(found, group, crop=True)
            if isinstance(proposal, SkillResult):
                _add_counts(scene_filtered, proposal.metadata.get("scene_filtered_by_source", {}))
                return self._stop_trying(last, tries, rejected, scene_filtered, proposal.message)
            target = found
            proposals, box = proposal
            _add_counts(scene_filtered, proposals.scene_filtered)
            ranked, skipped = self._untried(proposals, target, tried)
            if not ranked:
                return self._stop_trying(
                    last,
                    tries,
                    rejected,
                    scene_filtered,
                    f"Every new grasp candidate for object {target.object_id} repeats a "
                    "grasp already tried.",
                )
            # Re-centre the ignored box on where the object is now.
            self._exclude_target(box)

    def _untried(
        self, proposals: GraspProposals, target: ScannedObject, tried: Sequence[_TriedGrasp]
    ) -> tuple[list[SourcedGrasp], int]:
        """The grasps for the next try, and how many candidates repeat one already tried.

        Args:
            proposals: Grasps from each source for the object as it is now.
            target: The object as it is now, with its centre.
            tried: Grasps already tried on it.
        """

        def repeats(candidate: GraspCandidate) -> bool:
            return any(self._repeats(candidate, target, grasp) for grasp in tried)

        ranked = proposals.to_try(
            self.config.max_grasp_attempts,
            skip=repeats,
            distance=self.config.retry_skip_distance,
            angle=self.config.retry_skip_angle,
        )
        return ranked, sum(repeats(candidate) for candidate in proposals.all().candidates)

    def _grasp_once(
        self,
        candidates: GraspCandidateArray,
        sources: list[str],
        ranked: Sequence[SourcedGrasp],
        target: ScannedObject,
        box: TargetBox | None,
        group: PlanningGroupID,
    ) -> _GraspTry:
        """Open the gripper and grasp with the first candidate the planner can reach.

        Args:
            candidates: Every candidate generated for the object, for its frame,
                time stamp and count.
            sources: Grasp sources in effect, e.g. ["graspgenx", "heuristic"].
            ranked: Grasps to try, in order.
            target: The object being picked, with where it was scanned.
            box: Bounds of the object's point cloud, or None when it has none.
            group: Gripper-capable planning group doing the pick.
        """
        object_id = target.object_id
        if not_open := self._open_gripper(group, "before grasping"):
            return _GraspTry(not_open)

        last_plan = ""
        rejected = dict.fromkeys(sources, 0)
        for choice in ranked:
            rank, candidate = choice.rank, choice.candidate
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
                rejected[choice.source] += 1
                continue
            if blocked is not None:
                return _GraspTry(
                    self._stopped(
                        f"Move to grasp candidate {rank} for object {object_id}", blocked
                    ),
                    choice,
                    rejected=rejected,
                )
            if not_held := self._close_and_verify(group, object_id):
                # Back out while the target's cells are still ignored: left at
                # the grasp pose, the arm would start the next plan inside them.
                if blocked := self._servo(grasp, pregrasp, group):
                    self._arm_in_target = True
                    stopped = self._stopped("Retract to the pre-grasp pose", blocked)
                    return _GraspTry(
                        SkillResult(
                            f"{not_held.message} {stopped.message}", metadata=not_held.metadata
                        ),
                        choice,
                        rejected=rejected,
                    )
                missed = not_held.metadata.get("grasped") is False
                return _GraspTry(not_held, choice, missed, rejected)

            self._held = HeldObject(object_id, target.name, grasp)
            self._carry_target(box, grasp, group)
            if blocked := self._servo(grasp, pregrasp, group):
                return _GraspTry(
                    self._stopped(f"Retract after grasping object {object_id}", blocked),
                    choice,
                    rejected=rejected,
                )
            result = self._verify_lift(target, grasp, pregrasp, group)
            result.metadata.update(
                source=choice.source,
                rank=rank,
                score=candidate.score,
                candidates=len(candidates.candidates),
            )
            return _GraspTry(result, choice, result.metadata["grasped"] is False, rejected)
        return _GraspTry(
            SkillResult(
                f"The planner found no path to any of the {len(ranked)} grasp candidate(s) "
                f"tried for object {object_id}; last planner result {last_plan}."
            ),
            rejected=rejected,
        )

    def _return_to_scan_pose(self, group: PlanningGroupID) -> PlanResult | ExecutionResult | None:
        """Move the arm back to its joint positions from the latest scan_objects call.

        At the pre-grasp pose the wrist camera is too close to the object to see
        it whole; from the scan pose it has already seen it once. The move is
        collision checked and carries a held object along.

        Args:
            group: Planning group to move.

        Returns the planner or execution result that stopped the move, or None
        when the arm arrived, was already there, or no scan pose was recorded
        for the group.
        """
        joints = self._scan_joints.get(group)
        if joints is None:
            return None
        state = self._manipulation.get_state().groups.get(group)
        if (
            state is not None
            and state.joints is not None
            and np.allclose(state.joints.position, joints.position, atol=_AT_SCAN_POSE_TOLERANCE)
        ):
            return None
        return self._run(self._manipulation.plan_to_joints({group: joints}))

    def _rescan(self, target: ScannedObject) -> ScannedObject | str:
        """Detect the object again after a missed grasp and record it in the latest scan.

        Args:
            target: The object as it was last seen.

        Returns the detection that may be the target (see ``_sightings``) nearest
        to where it was, within ``lift_check_radius``, or a statement of why
        there is none.
        """
        detections = self._detect_after(target.name, time.time())
        if detections is None:
            return f"A new scan for {target.name} found nothing: {self._no_frame()}."
        found = [
            obj
            for obj in self._sightings(target, detections)
            if obj.position.distance(target.position) <= self.config.lift_check_radius
        ]
        if not found:
            return (
                f"A new scan for {target.name} found none within "
                f"{self.config.lift_check_radius:.2f} m of where object {target.object_id} was."
            )
        nearest = min(found, key=lambda obj: obj.position.distance(target.position))
        self._objects.pop(target.object_id, None)
        self._objects[nearest.object_id] = nearest
        return nearest

    def _repeats(
        self, candidate: GraspCandidate, target: ScannedObject, tried: _TriedGrasp
    ) -> bool:
        """Whether a candidate grasps the object the same way as a grasp already tried.

        Positions are compared relative to the object's centre, so a grasp on an
        object that was pushed aside still matches.

        Args:
            candidate: New grasp candidate, planning frame.
            target: The object as it is now, with its centre.
            tried: A grasp already tried, with its offset from the object's centre then.
        """
        return same_grasp(
            Pose(candidate.pose.position - target.position, candidate.pose.orientation),
            Pose(tried.offset, tried.orientation),
            self.config.retry_skip_distance,
            self.config.retry_skip_angle,
        )

    @staticmethod
    def _stop_trying(
        last: SkillResult,
        tries: list[dict[str, Any]],
        rejected: dict[str, int],
        scene_filtered: dict[str, int],
        gave_up: str | None = None,
    ) -> SkillResult:
        """Report the end of a pick, with one record per grasp tried.

        Args:
            last: The report of the last step.
            tries: One record per grasp tried, in order; empty when no grasp was.
            rejected: Per source, how many grasps the planner found no path to
                over the whole pick.
            scene_filtered: Per source, how many grasps the scene check dropped
                over the whole pick; empty when it never ran.
            gave_up: Why pick_object stopped trying after a verified miss, or None
                when it stopped for another reason (success, a fault, no path).
        """
        metadata = {
            **last.metadata,
            "rejected_by_source": rejected,
            "scene_filtered_by_source": scene_filtered,
        }
        if not tries:
            return SkillResult(last.message, metadata=metadata)
        message = last.message if gave_up is None else f"{last.message} {gave_up}"
        metadata.update(tries=tries, handoff=gave_up is not None)
        return SkillResult(message, metadata=metadata)

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
        self,
        target: ScannedObject,
        grasp: PoseStamped,
        pregrasp: PoseStamped,
        planning_group: PlanningGroupID,
    ) -> SkillResult:
        """Check that the object came up with the gripper, and report the pick.

        Jaws that stop on an edge, or on an object pushed aside, read the same as
        a hold, so after the lift the jaws are read again, the arm carries the
        object to the scan pose (when one was recorded), the jaws are read once
        more and the camera looks for the object. If that move fails the camera
        looks from where the arm stopped.

        Args:
            target: The object being picked, with where it was scanned.
            grasp: Gripper pose that closed on the object, planning frame.
            pregrasp: Gripper pose the arm lifted to, planning frame.
            planning_group: Group whose gripper closed on the object.

        Returns "Pick complete" with ``grasped: True``, else opens the gripper,
        forgets the held object and returns ``grasped: False``.
        """
        # How far (metres) and which way the gripper, and a held object, moved.
        lift = pregrasp.position - grasp.position
        facts: dict[str, Any] = {
            **_summary(target),
            "gripper_lift_m": round(lift.z, 3),
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
        note = ""
        if self._scan_joints.get(planning_group) is not None:
            if moved_arm := self._return_to_scan_pose(planning_group):
                stopped = self._stopped(
                    f"Move to the scan pose to look for object {target.object_id}", moved_arm
                )
                note = f" {stopped.message}; the camera looked from where the arm stopped."
            tip = self._tip_position(planning_group)
            if tip is not None:
                lift = tip - grasp.position
                facts["gripper_lift_m"] = round(lift.z, 3)
            jaws = self._gripper_position(planning_group)
            if verification.enabled and jaws is not None and jaws <= verification.held_low:
                return self._missed_grasp(
                    planning_group,
                    f"After carrying object {target.object_id} up, the gripper reads "
                    f"{jaws:.2f}: the object fell out.{note}",
                    jaws=round(jaws, 3),
                    **facts,
                )
        detections = self._detect_after(target.name, time.time())
        if detections is None:
            # Raising here would leave the gripper closed on an object nobody tracks.
            return SkillResult(
                f"Pick complete on the jaw reading alone: {self._no_frame()}.{note}",
                metadata={**facts, "object_lift_m": None, "grasped": True},
            )
        sightings = self._sightings(target, detections)
        rose = _height_gained(
            [obj.position for obj in sightings],
            target.position,
            lift,
            self.config.lift_check_radius,
        )
        facts["object_lift_m"] = None if rose is None else round(rose, 3)
        if rose is not None and rose < self.config.lift_follow_fraction * lift.z:
            return self._missed_grasp(
                planning_group,
                f"Lifted the gripper {lift.z:.2f} m, but the camera sees object "
                f"{target.object_id} rose {rose:.2f} m with it; it stayed on the table.{note}",
                **facts,
            )
        complete = f"Pick complete.{note}" if note else "Pick complete"
        return SkillResult(complete, metadata={**facts, "grasped": True})

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

    def _detect_after(self, label: str, since: float) -> Detection3DArray | None:
        """Detect a label in a camera frame taken after a given time.

        Args:
            label: Object label to detect.
            since: Wall-clock time (seconds, as ``time.time()``) the frame must be
                newer than, e.g. when the arm stopped.

        Returns None when no such frame arrives within ``fresh_frame_timeout``.
        """
        deadline = time.monotonic() + self.config.fresh_frame_timeout
        while True:
            detections = self._scene.scan_scene(text=[label])
            if detections.ts >= since:
                return detections
            if time.monotonic() >= deadline:
                return None
            time.sleep(_FRESH_FRAME_POLL)

    def _no_frame(self) -> str:
        """Why a check had no camera frame, as the end of a sentence."""
        return (
            "the camera gave no frame taken after the arm stopped within "
            f"{self.config.fresh_frame_timeout:.1f} s"
        )

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
            for center in ([] if detections is None else _centers(detections))
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
        return self._run(self._manipulation.plan_to_poses({planning_group: pose}))

    def _run(self, plan: PlanResult) -> PlanResult | ExecutionResult | None:
        """Run a plan to the end.

        Returns the plan when it failed, the execution result when the motion
        stopped part-way, or None when the arm arrived.
        """
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
            orientation=Quaternion(detection.bbox.center.orientation),
            size=Vector3(detection.bbox.size),
        )
        for detection in detections.detections
        if detection.id and detection.results
    }


def _summary(obj: ScannedObject | HeldObject) -> dict[str, Any]:
    """ID and label of an object, as skill results and RPCs report it."""
    return {"object_id": obj.object_id, "name": obj.name}


def _height_gained(
    centers: Sequence[Vector3], start: Vector3, lift: Vector3, radius: float
) -> float | None:
    """How high a lifted object rose with the gripper, as the camera sees it.

    Args:
        centers: Centres of the detections of the object's label after the lift,
            metres, in the planning frame (Z up).
        start: Centre of the object before the grasp, metres.
        lift: How the gripper moved from the grasp pose to where it is now, metres.
        radius: Detections farther than this (metres) from both ``start`` and
            ``start + lift`` are taken to be other objects.

    Returns the most any remaining detection rose above ``start``, metres, or
    None when no detection remains. Sliding sideways does not count.
    """
    lifted_to = start + lift
    nearby = [
        center
        for center in centers
        if min(center.distance(start), center.distance(lifted_to)) <= radius
    ]
    if not nearby:
        return None
    return max(center.z - start.z for center in nearby)


def _crop(pointcloud: PointCloud2, box: ScannedObject, margin: float) -> PointCloud2:
    """The points of a cloud inside an object's detected 3D box, grown on every side.

    Args:
        pointcloud: Points in the planning frame, metres.
        box: The detection whose box (centre, orientation, size) to keep.
        margin: How far (metres) outside each face of the box a point may lie.
    """
    points = np.asarray(pointcloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    # Each point in the box's own axes, R^T (p - c), written for row vectors.
    local = (points - box.position.to_numpy()) @ box.orientation.to_rotation_matrix()
    half = np.asarray(box.size.to_tuple(), dtype=np.float64) / 2.0 + margin
    inside = np.flatnonzero(np.all(np.abs(local) <= half, axis=1))
    return PointCloud2(
        pointcloud.pointcloud.select_by_index(inside.tolist()),
        frame_id=pointcloud.frame_id,
        ts=pointcloud.ts,
    )


def _scene_near(scene: PointCloud2, target: PointCloud2, tips: Sequence[Vector3]) -> PointCloud2:
    """The scene points a grasp on the target could hit.

    Keeps the points within ``_SCENE_MARGIN`` of the target's bounds, and drops
    those within ``_TARGET_POINT_RADIUS`` of a target point (the target itself)
    and within ``_GRIPPER_SELF_RADIUS`` of a tool tip (the robot's own gripper).

    Args:
        scene: Camera points, metres, in the target's frame.
        target: The target's points, metres; with none, no scene point is kept.
        tips: Where each gripper's tool tip is, metres, same frame.
    """
    points = np.asarray(scene.points_f32(), dtype=np.float64).reshape((-1, 3))
    own = np.asarray(target.points_f32(), dtype=np.float64).reshape((-1, 3))
    own = own[np.all(np.isfinite(own), axis=1)]
    keep = np.zeros(len(points), dtype=bool)
    if len(own):
        low = own.min(axis=0) - _SCENE_MARGIN
        high = own.max(axis=0) + _SCENE_MARGIN
        keep = np.all((points >= low) & (points <= high), axis=1)
    for tip in tips:
        keep &= np.linalg.norm(points - tip.to_numpy(), axis=1) > _GRIPPER_SELF_RADIUS
    if keep.any():
        distance, _ = cKDTree(own).query(points[keep])
        keep[np.flatnonzero(keep)[distance <= _TARGET_POINT_RADIUS]] = False
    return PointCloud2.from_numpy(
        points[keep].astype(np.float32), frame_id=scene.frame_id, timestamp=scene.ts
    )


def _add_counts(total: dict[str, int], more: dict[str, int]) -> None:
    """Add per-source counts into a running total, in place."""
    for source, count in more.items():
        total[source] = total.get(source, 0) + count


def _horizontal_distance(a: Vector3, b: Vector3) -> float:
    """Distance between two points ignoring height, metres."""
    return math.hypot(a.x - b.x, a.y - b.y)
