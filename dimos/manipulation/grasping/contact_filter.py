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

"""Drop grasp candidates whose gripper would touch the object off its pads, hit the scene,
or close on nothing.

The check itself runs in the manipulation module, which holds the robot's gripper
shape (``ManipulationSpec.check_grasp_contacts``). This module prepares what it
checks against and keeps the candidates that pass.
"""

from __future__ import annotations

from collections.abc import Sequence
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.planning.spec.gripper import UNCHECKED, GraspContact
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

logger = setup_logger()

# Names the obstacle point sets go by in the check and in the report.
TABLE = "table"
OBJECTS = "objects"
# How far (metres) beyond the object's points scene points are kept: farther
# than any part of the gripper reaches from a grasp on the object.
_REACH = 0.25
# Scene points this close (metres) to a gripper's tool tip are that gripper: the
# wrist camera sees its own fingers.
_GRIPPER_SELF_RADIUS = 0.15
# Scene points this close (metres) to the object's points, and not on the table,
# are the object's own edge left outside its mask.
_TARGET_EDGE = 0.01
# Height bins (metres) for finding the table, and how far (metres) from its
# height a point still lies on it.
_TABLE_BIN = 0.005
_TABLE_BAND = 0.005
# The table is looked for this far (metres) above the object's lowest point and below.
_TABLE_ABOVE_TARGET = 0.02


def scene_obstacles(
    scene: PointCloud2, target: PointCloud2, tips: Sequence[Vector3]
) -> tuple[dict[str, PointCloud2], float | None]:
    """Split the scene around an object into the table under it and everything else.

    The table is the most common height among the points no higher than 2 cm
    above the object's lowest point, so the frame's Z must point up; points
    within 5 mm of that height, and any below it, are table. With no such
    points there is no table and every point is an object.

    Args:
        scene: Camera points with the object masked out, metres, in the object's frame.
        target: The object's points, metres.
        tips: Where each gripper's tool tip is, metres, same frame; points near
            them are the robot's own gripper and are dropped.

    Returns {"table": ..., "objects": ...}, each possibly empty, and the
    table's height (metres; None when there is no table).
    """
    points = _xyz(scene)
    own = _xyz(target)
    keep = np.zeros(len(points), dtype=bool)
    if len(own):
        low, high = own.min(axis=0) - _REACH, own.max(axis=0) + _REACH
        keep = np.all((points >= low) & (points <= high), axis=1)
    for tip in tips:
        keep &= np.linalg.norm(points - tip.to_numpy(), axis=1) > _GRIPPER_SELF_RADIUS
    points = points[keep]
    table = np.zeros(len(points), dtype=bool)
    height: float | None = None
    if len(own) and len(points):
        floor = own[:, 2].min()
        low_points = points[points[:, 2] <= floor + _TABLE_ABOVE_TARGET, 2]
        if len(low_points):
            bins = np.floor(low_points / _TABLE_BIN).astype(np.int64)
            values, counts = np.unique(bins, return_counts=True)
            height = float((values[np.argmax(counts)] + 0.5) * _TABLE_BIN)
            table = points[:, 2] <= height + _TABLE_BAND
    objects = points[~table]
    if len(own) and len(objects):
        distance, _ = cKDTree(own).query(objects)
        objects = objects[distance > _TARGET_EDGE]
    return {
        TABLE: _cloud(points[table], scene),
        OBJECTS: _cloud(objects, scene),
    }, height


def drop_touching(
    manipulation: ManipulationSpec,
    candidates: GraspCandidateArray,
    target: PointCloud2,
    obstacles: dict[str, PointCloud2],
    contact_margin: float,
    obstacle_margin: float,
    planning_group: PlanningGroupID | None,
    min_gap_points: int = 0,
    depth_search: float = 0.0,
    depth_step: float = 0.005,
    floor: float | None = None,
    depth_enough: int = 0,
) -> tuple[GraspCandidateArray, dict[str, Any]]:
    """Keep the grasps whose gripper touches the object only at its contact patches.

    Args:
        manipulation: The manipulation module, which runs the check.
        candidates: Grasps, in the frame of the point clouds.
        target: The object's points, metres.
        obstacles: Points the gripper must not overlap, by name, metres.
        contact_margin: Closer than this (metres) to the object is a touch.
        obstacle_margin: Closer than this (metres) to an obstacle is an overlap.
        planning_group: Group whose tool tip the grasps pose.
        min_gap_points: Fewest object points (one per 5 mm cube) the gap
            between the finger pads must hold; 0 skips this.
        depth_search: How far (metres) a grasp that fails where it is may slide
            along its approach, each way, to the nearest depth that passes. Moved
            grasps come back at that depth, and only when no grasp passes where
            its source put it. 0 checks each grasp where it is.
        depth_step: Spacing (metres) of the depths tried.
        floor: Height (metres) of the table's top; a grasp with any gripper
            part below it is dropped. None skips this.
        depth_enough: With ``depth_search``, stop once this many grasps pass,
            taking them in order; the rest are dropped unchecked. 0 checks all.

    Returns the grasps that pass and the facts of the check: ``checked``,
    ``rejected``, how many touched each of "target" and the obstacles,
    ``empty_gap`` (too few object points between the pads; only when
    ``min_gap_points`` is set), ``while_closing`` (rejected only for what the
    closing fingers hit), ``moved`` (kept at another depth; only with
    ``depth_search``) and ``seconds``; or ``{"skipped": why}`` with every grasp
    kept when the robot model has no gripper spec.
    """
    if not candidates.candidates:
        return candidates, {"checked": 0, "rejected": 0}
    started = time.monotonic()
    verdicts = manipulation.check_grasp_contacts(
        candidates,
        target,
        obstacles,
        contact_margin,
        obstacle_margin,
        planning_group,
        min_gap_points=min_gap_points,
        depth_search=depth_search,
        depth_step=depth_step,
        floor=floor,
        depth_enough=depth_enough,
    )
    if verdicts is None:
        return candidates, {"skipped": "the robot model has no gripper spec"}
    if len(verdicts) != len(candidates.candidates):
        raise RuntimeError(
            f"The contact check answered for {len(verdicts)} of "
            f"{len(candidates.candidates)} grasp candidates"
        )
    facts: dict[str, Any] = {
        "checked": len(verdicts),
        "rejected": sum(
            not verdict.clear and UNCHECKED not in verdict.touches for verdict in verdicts
        ),
    }
    for name in ("target", *obstacles, *(["floor"] if floor is not None else [])):
        facts[name] = sum(name in verdict.touches for verdict in verdicts)
    if min_gap_points > 0:
        facts["empty_gap"] = sum(verdict.empty_gap for verdict in verdicts)
    facts["while_closing"] = sum(verdict.while_closing for verdict in verdicts)
    if depth_enough > 0:
        facts["unchecked"] = sum(UNCHECKED in verdict.touches for verdict in verdicts)
    # Moved grasps are a fallback: when any grasp passes where its source put
    # it, only those are kept.
    unmoved = any(verdict.clear and verdict.depth_shift == 0.0 for verdict in verdicts)
    keep = [verdict.clear and (verdict.depth_shift == 0.0 or not unmoved) for verdict in verdicts]
    if depth_search > 0.0:
        facts["moved"] = sum(
            kept and verdict.depth_shift != 0.0
            for kept, verdict in zip(keep, verdicts, strict=True)
        )
    facts["seconds"] = round(time.monotonic() - started, 3)
    return _kept(candidates, verdicts, keep), facts


def check_in_scene(
    manipulation: ManipulationSpec,
    scene_registration: ObjectSceneRegistrationSpec,
    object_id: str,
    target: PointCloud2,
    candidates: GraspCandidateArray,
    contact_margin: float,
    obstacle_margin: float,
    planning_group: PlanningGroupID,
    min_gap_points: int = 0,
    depth_search: float = 0.0,
    depth_step: float = 0.005,
    depth_enough: int = 0,
) -> tuple[GraspCandidateArray, dict[str, Any]]:
    """Check grasps against an object's points and the camera's view of the scene around it.

    Fetches the latest scan's scene with the object masked out, splits it with
    ``scene_obstacles`` and runs ``drop_touching``.

    Args:
        manipulation: The manipulation module, which runs the check.
        scene_registration: Where the latest scan's scene points come from.
        object_id: The object being grasped, from the latest scan.
        target: Its points, metres, in the planning frame.
        candidates: Grasps, same frame.
        contact_margin: Closer than this (metres) to the object is a touch.
        obstacle_margin: Closer than this (metres) to the table or another object is an overlap.
        planning_group: Group whose tool tip the grasps pose.
        min_gap_points: Fewest object points (one per 5 mm cube) the gap
            between the finger pads must hold; 0 skips this.
        depth_search: How far (metres) each grasp may slide along its approach;
            see ``drop_touching``.
        depth_step: Spacing (metres) of the depths tried.
        depth_enough: Stop the depth search once this many grasps pass; 0
            checks all.

    Returns what ``drop_touching`` returns; every grasp is kept, with
    ``{"skipped": why}``, when there is no scene in the object's frame or the
    check fails.
    """
    scene = scene_registration.get_full_scene_pointcloud(exclude_object_id=object_id)
    if scene is None or scene.frame_id != target.frame_id:
        return candidates, {"skipped": "no scene point cloud in the object's frame"}
    tips = [
        state.end_effector_pose.position
        for state in manipulation.get_state().groups.values()
        if state.end_effector_pose is not None
    ]
    try:
        obstacles, table_height = scene_obstacles(scene, target, tips)
        return drop_touching(
            manipulation,
            candidates,
            target,
            obstacles,
            contact_margin,
            obstacle_margin,
            planning_group,
            min_gap_points,
            depth_search,
            depth_step,
            table_height,
            depth_enough,
        )
    except Exception as exc:
        logger.warning("Grasp contact check failed; keeping every grasp", error=str(exc))
        return candidates, {"skipped": f"the check failed: {type(exc).__name__}: {exc}"}


def contact_note(facts: dict[str, Any]) -> str:
    """A sentence saying how many grasps the contact check dropped, with a leading space, or ""."""
    rejected = facts.get("rejected", 0)
    if not rejected:
        return ""
    if facts.get("empty_gap"):
        return (
            f" The contact check dropped {rejected} grasp(s) whose gripper would touch the "
            "object off its finger pads, hit the scene or close on nothing."
        )
    return (
        f" The contact check dropped {rejected} grasp(s) whose gripper would touch the "
        "object off its finger pads or hit the scene."
    )


def add_facts(total: dict[str, Any], more: dict[str, Any]) -> None:
    """Add one check's counts into a pick's running total, in place; seconds add too."""
    for name, value in more.items():
        if isinstance(value, int | float) and not isinstance(value, bool):
            total[name] = round(total.get(name, 0) + value, 3)
        else:
            total[name] = value


def _kept(
    array: GraspCandidateArray, verdicts: Sequence[GraspContact], keep: Sequence[bool]
) -> GraspCandidateArray:
    """The candidates of an array marked ``keep``, in order, each moved along its
    approach by its verdict's ``depth_shift``."""
    kept = [
        _shifted(candidate, verdict.depth_shift)
        for candidate, verdict, chosen in zip(array.candidates, verdicts, keep, strict=True)
        if chosen
    ]
    return GraspCandidateArray(array.header, kept)


def _shifted(candidate: GraspCandidate, shift: float) -> GraspCandidate:
    """The candidate moved ``shift`` metres along its tool tip's z axis (its approach)."""
    if shift == 0.0:
        return candidate
    position = candidate.pose.position.to_numpy() + shift * pose_to_matrix(candidate.pose)[:3, 2]
    return GraspCandidate(Pose(Vector3(*position), candidate.pose.orientation), candidate.score)


def _xyz(cloud: PointCloud2) -> NDArray[np.float64]:
    points = np.asarray(cloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    return points[np.all(np.isfinite(points), axis=1)]


def _cloud(points: NDArray[np.float64], like: PointCloud2) -> PointCloud2:
    return PointCloud2.from_numpy(
        points.astype(np.float32), frame_id=like.frame_id, timestamp=like.ts
    )
