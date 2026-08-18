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

"""Closed-loop execution of ranked grasp candidates."""

from __future__ import annotations

import time

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ManipulationSpec,
)
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.grounded_segmentation import GroundedSegmentationSpec
from dimos.perception.rgbd import RGBDObservation, project_depth
from dimos.utils.transform_utils import offset_distance

_MAX_GRASP_CANDIDATES = 6
_APPROACH_DISTANCE_METERS = 0.08
_OPEN_POSITION_METERS = 0.04
_CLOSED_POSITION_METERS = 0.0
_GRIPPER_SETTLE_SECONDS = 0.25
_MIN_HELD_APERTURE_METERS = 0.001
_GEOMETRIC_CLEARANCE_METERS = 0.08
_RECEPTACLE_RIM_CLEARANCE_METERS = 0.005
_TALL_RECEPTACLE_METERS = 0.06
_MIN_PLANAR_ANISOTROPY = 1.5
_MIN_AXIS_NORM = 1e-6
_GRASP_HEIGHT_FRACTION = 0.5


class GraspExecutionModule(Module):
    """Execute grasp proposals through the ordinary manipulation interface."""

    _manipulation: ManipulationSpec
    _grounded_segmentation: GroundedSegmentationSpec

    @rpc
    def execute_grasp_candidates(
        self,
        grasps: GraspCandidateArray,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult:
        """Approach, close on, and retract from the first reachable ranked grasp."""
        return execute_grasp_candidates(self._manipulation, grasps, planning_group)

    @rpc
    def pick_and_place_pointclouds(
        self,
        source: PointCloud2,
        destination: PointCloud2,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult:
        """Pick a tabletop object and place it on or inside observed destination geometry."""
        return pick_and_place_pointclouds(
            self._manipulation,
            source,
            destination,
            planning_group,
        )

    @rpc
    def grounded_pick_and_place(
        self,
        observation: RGBDObservation,
        source_description: str,
        destination_description: str,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult:
        """Ground two described objects in supplied RGB-D data, then pick and place."""
        return grounded_pick_and_place(
            self._manipulation,
            self._grounded_segmentation,
            observation,
            source_description,
            destination_description,
            planning_group,
        )


def grounded_pick_and_place(
    manipulation: ManipulationSpec,
    grounded_segmentation: GroundedSegmentationSpec,
    observation: RGBDObservation,
    source_description: str,
    destination_description: str,
    planning_group: PlanningGroupID | None = None,
) -> CommandResult:
    """Ground two described objects in supplied RGB-D data, then pick and place."""
    source_masks = grounded_segmentation.segment_best(
        observation.color,
        source_description,
    )
    destination_masks = grounded_segmentation.segment_best(
        observation.color,
        destination_description,
    )
    if len(source_masks) != 1:
        return CommandResult(
            CommandStatus.FAILED,
            f"Expected one source mask, received {len(source_masks)}",
        )
    if len(destination_masks) != 1:
        return CommandResult(
            CommandStatus.FAILED,
            f"Expected one destination mask, received {len(destination_masks)}",
        )
    sources = project_depth(source_masks, observation)
    destinations = project_depth(destination_masks, observation)
    if len(sources) != 1 or len(destinations) != 1:
        return CommandResult(
            CommandStatus.FAILED,
            "Grounded masks did not produce one source and one destination point cloud",
        )
    return pick_and_place_pointclouds(
        manipulation,
        sources[0].pointcloud,
        destinations[0].pointcloud,
        planning_group,
    )


def execute_grasp_candidates(
    manipulation: ManipulationSpec,
    grasps: GraspCandidateArray,
    planning_group: PlanningGroupID | None = None,
) -> CommandResult:
    """Execute the first reachable grasp using collision-free approach planning."""
    if not grasps.candidates:
        return CommandResult(CommandStatus.REJECTED, "No grasp candidates were provided")
    group_id = _resolve_group(manipulation, planning_group)
    if group_id is None:
        return CommandResult(CommandStatus.REJECTED, "A unique gripper planning group is required")

    opened = manipulation.set_gripper_position(_OPEN_POSITION_METERS, group_id)
    if not opened.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Opening gripper failed: {opened.message}")
    time.sleep(_GRIPPER_SETTLE_SECONDS)

    last_error = "No candidate was reachable"
    for index, candidate in enumerate(grasps.candidates[:_MAX_GRASP_CANDIDATES], start=1):
        target = _stamped(candidate.pose, grasps)
        pre_grasp = _stamped(
            offset_distance(candidate.pose, _APPROACH_DISTANCE_METERS),
            grasps,
        )
        plan = manipulation.plan_to_poses({group_id: pre_grasp})
        if not plan.succeeded:
            last_error = f"Candidate {index} pre-grasp failed: {plan.message}"
            continue
        approach = manipulation.execute(blocking=True)
        if not approach.succeeded:
            return CommandResult(
                CommandStatus.FAILED,
                f"Candidate {index} approach execution failed: {approach.message}",
            )
        contact = manipulation.move_to_pose(
            target,
            planning_group=group_id,
            check_collision=False,
            speed_scale=0.25,
            blocking=True,
        )
        if not contact.succeeded:
            last_error = f"Candidate {index} contact move failed: {contact.plan.message}"
            continue
        closed = manipulation.set_gripper_position(_CLOSED_POSITION_METERS, group_id)
        if not closed.succeeded:
            return CommandResult(CommandStatus.FAILED, f"Closing gripper failed: {closed.message}")
        time.sleep(_GRIPPER_SETTLE_SECONDS)
        state = manipulation.get_state().groups.get(group_id)
        held_aperture = state.gripper_position if state is not None else None
        retract = manipulation.move_to_pose(
            pre_grasp,
            planning_group=group_id,
            check_collision=False,
            speed_scale=0.25,
            blocking=True,
        )
        if not retract.succeeded:
            return CommandResult(
                CommandStatus.FAILED,
                f"Candidate {index} retract failed: {retract.plan.message}",
            )
        if held_aperture is not None and held_aperture <= _MIN_HELD_APERTURE_METERS:
            last_error = f"Candidate {index} closed without retaining an object"
            reopened = manipulation.set_gripper_position(_OPEN_POSITION_METERS, group_id)
            if not reopened.succeeded:
                return CommandResult(
                    CommandStatus.FAILED,
                    f"Reopening after candidate {index} failed: {reopened.message}",
                )
            time.sleep(_GRIPPER_SETTLE_SECONDS)
            continue
        detail = "" if held_aperture is None else f" at {held_aperture:.4f} m aperture"
        return CommandResult(
            CommandStatus.SUCCEEDED,
            f"Executed grasp candidate {index}{detail}",
        )

    return CommandResult(CommandStatus.FAILED, last_error)


def pick_and_place_pointclouds(
    manipulation: ManipulationSpec,
    source: PointCloud2,
    destination: PointCloud2,
    planning_group: PlanningGroupID | None = None,
) -> CommandResult:
    """Execute a geometry-derived top-down pick and placement.

    The current end-effector orientation is retained, which makes this primitive
    portable across downward-facing tabletop manipulators without embedding a
    robot-specific quaternion.
    """
    if not len(source) or not len(destination):
        return CommandResult(CommandStatus.REJECTED, "Source and destination must be non-empty")
    group_id = _resolve_group(manipulation, planning_group)
    if group_id is None:
        return CommandResult(CommandStatus.REJECTED, "A unique gripper planning group is required")
    state = manipulation.get_state().groups.get(group_id)
    if state is None or state.end_effector_pose is None:
        return CommandResult(CommandStatus.FAILED, "End-effector pose is unavailable")

    orientation = _geometry_aligned_orientation(
        source,
        state.end_effector_pose.orientation,
    )
    source_center = source.center
    source_bounds = source.axis_aligned_bounding_box
    source_height = float(source.bounding_box_dimensions[2])
    destination_bounds = destination.axis_aligned_bounding_box
    destination_min = destination_bounds.min_bound
    destination_max = destination_bounds.max_bound
    destination_x = float((destination_min[0] + destination_max[0]) / 2.0)
    destination_y = float((destination_min[1] + destination_max[1]) / 2.0)
    destination_height = float(destination.bounding_box_dimensions[2])
    destination_top = float(destination_max[2])

    pick = PoseStamped(
        ts=source.ts,
        frame_id=source.frame_id,
        position=Vector3(
            source_center.x,
            source_center.y,
            float(source_bounds.min_bound[2])
            + _GRASP_HEIGHT_FRACTION
            * float(source_bounds.max_bound[2] - source_bounds.min_bound[2]),
        ),
        orientation=orientation,
    )
    pre_pick = _above(pick, _GEOMETRIC_CLEARANCE_METERS)
    # Tall destination geometry is treated as a receptacle; thin geometry is a
    # supporting surface whose target must include half the held object's height.
    if destination_height >= _TALL_RECEPTACLE_METERS:
        place_z = (
            destination_top
            - source_height / 2.0
            - _RECEPTACLE_RIM_CLEARANCE_METERS
        )
    else:
        place_z = destination_top + source_height / 2.0
    place = PoseStamped(
        ts=destination.ts,
        frame_id=destination.frame_id,
        position=Vector3(destination_x, destination_y, place_z),
        orientation=orientation,
    )
    pre_place = _above(place, _GEOMETRIC_CLEARANCE_METERS)

    opened = manipulation.set_gripper_position(_OPEN_POSITION_METERS, group_id)
    if not opened.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Opening gripper failed: {opened.message}")
    approach = _planned_move(manipulation, group_id, pre_pick, "pick approach")
    if not approach.succeeded:
        return approach
    contact = manipulation.move_to_pose(
        pick,
        planning_group=group_id,
        check_collision=False,
        speed_scale=1.0,
        blocking=True,
    )
    if not contact.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Pick contact failed: {contact.plan.message}")
    closed = manipulation.set_gripper_position(_CLOSED_POSITION_METERS, group_id)
    if not closed.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Closing gripper failed: {closed.message}")
    time.sleep(_GRIPPER_SETTLE_SECONDS)
    retract = manipulation.move_to_pose(
        pre_pick,
        planning_group=group_id,
        check_collision=False,
        speed_scale=1.0,
        blocking=True,
    )
    if not retract.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Pick retract failed: {retract.plan.message}")
    held = manipulation.get_state().groups.get(group_id)
    if (
        held is not None
        and held.gripper_position is not None
        and held.gripper_position <= _MIN_HELD_APERTURE_METERS
    ):
        manipulation.set_gripper_position(_OPEN_POSITION_METERS, group_id)
        return CommandResult(CommandStatus.FAILED, "Gripper closed without retaining an object")
    transit = _planned_move(manipulation, group_id, pre_place, "place approach")
    if not transit.succeeded:
        return transit
    lower = manipulation.move_to_pose(
        place,
        planning_group=group_id,
        check_collision=False,
        speed_scale=1.0,
        blocking=True,
    )
    if not lower.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Place contact failed: {lower.plan.message}")
    released = manipulation.set_gripper_position(_OPEN_POSITION_METERS, group_id)
    if not released.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Releasing gripper failed: {released.message}")
    time.sleep(_GRIPPER_SETTLE_SECONDS)
    retreat = manipulation.move_to_pose(
        pre_place,
        planning_group=group_id,
        check_collision=False,
        speed_scale=1.0,
        blocking=True,
    )
    if not retreat.succeeded:
        return CommandResult(CommandStatus.FAILED, f"Place retract failed: {retreat.plan.message}")
    return CommandResult(CommandStatus.SUCCEEDED, "Picked and placed using observed geometry")


def _planned_move(
    manipulation: ManipulationSpec,
    group_id: PlanningGroupID,
    target: PoseStamped,
    label: str,
) -> CommandResult:
    plan = manipulation.plan_to_poses({group_id: target})
    if not plan.succeeded:
        return CommandResult(CommandStatus.FAILED, f"{label} planning failed: {plan.message}")
    execution = manipulation.execute(blocking=True)
    if not execution.succeeded:
        return CommandResult(CommandStatus.FAILED, f"{label} execution failed: {execution.message}")
    return CommandResult(CommandStatus.SUCCEEDED, f"Completed {label}")


def _above(target: PoseStamped, distance: float) -> PoseStamped:
    return PoseStamped(
        ts=target.ts,
        frame_id=target.frame_id,
        position=Vector3(target.position.x, target.position.y, target.position.z + distance),
        orientation=target.orientation,
    )


def _geometry_aligned_orientation(
    source: PointCloud2,
    current: Quaternion,
) -> Quaternion:
    """Align a top-down gripper's TCP-X closing axis to the short planar axis."""
    points, _ = source.as_numpy()
    if len(points) < 4:
        return current
    covariance = np.cov(points[:, :2], rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.isfinite(eigenvalues).all() or eigenvalues[1] <= 0.0:
        return current
    anisotropy = eigenvalues[1] / max(eigenvalues[0], np.finfo(float).eps)
    if anisotropy < _MIN_PLANAR_ANISOTROPY:
        return current

    rotation = current.to_rotation_matrix()
    tool_z = rotation[:, 2]
    tool_z /= np.linalg.norm(tool_z)
    if abs(tool_z[2]) < 0.8:
        return current
    tool_x = np.array([*eigenvectors[:, 0], 0.0])
    tool_x -= np.dot(tool_x, tool_z) * tool_z
    norm = np.linalg.norm(tool_x)
    if norm < _MIN_AXIS_NORM:
        return current
    tool_x /= norm
    if np.dot(tool_x, rotation[:, 0]) < 0.0:
        tool_x *= -1.0
    tool_y = np.cross(tool_z, tool_x)
    tool_y /= np.linalg.norm(tool_y)
    tool_x = np.cross(tool_y, tool_z)
    aligned = np.column_stack((tool_x, tool_y, tool_z))
    return Quaternion.from_rotation_matrix(aligned)


def _resolve_group(
    manipulation: ManipulationSpec,
    planning_group: PlanningGroupID | None,
) -> PlanningGroupID | None:
    if planning_group is not None:
        return planning_group
    groups = [group for group in manipulation.list_planning_groups() if group.has_gripper]
    return groups[0].id if len(groups) == 1 else None


def _stamped(pose: Pose, grasps: GraspCandidateArray) -> PoseStamped:
    return PoseStamped(
        ts=grasps.header.timestamp,
        frame_id=grasps.header.frame_id,
        position=pose.position,
        orientation=pose.orientation,
    )
