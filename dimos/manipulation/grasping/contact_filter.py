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

"""Drop grasp candidates whose gripper would touch the object off its pads, or hit the scene.

The check itself runs in the manipulation module, which holds the robot's gripper
shape (``ManipulationSpec.check_grasp_contacts``). This module prepares what it
checks against and keeps the candidates that pass.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.manipulation.grasping.grasp_fallback import GraspProposals
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.planning.spec.gripper import GraspContact
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger

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
) -> dict[str, PointCloud2]:
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

    Returns {"table": ..., "objects": ...}, each possibly empty.
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
    if len(own) and len(points):
        floor = own[:, 2].min()
        low_points = points[points[:, 2] <= floor + _TABLE_ABOVE_TARGET, 2]
        if len(low_points):
            bins = np.floor(low_points / _TABLE_BIN).astype(np.int64)
            values, counts = np.unique(bins, return_counts=True)
            height = (values[np.argmax(counts)] + 0.5) * _TABLE_BIN
            table = points[:, 2] <= height + _TABLE_BAND
    objects = points[~table]
    if len(own) and len(objects):
        distance, _ = cKDTree(own).query(objects)
        objects = objects[distance > _TARGET_EDGE]
    return {
        TABLE: _cloud(points[table], scene),
        OBJECTS: _cloud(objects, scene),
    }


def drop_touching(
    manipulation: ManipulationSpec,
    proposals: GraspProposals,
    target: PointCloud2,
    obstacles: dict[str, PointCloud2],
    contact_margin: float,
    obstacle_margin: float,
    planning_group: PlanningGroupID | None,
) -> tuple[GraspProposals, dict[str, Any]]:
    """Keep the grasps whose gripper touches the object only at its contact patches.

    Args:
        manipulation: The manipulation module, which runs the check.
        proposals: Grasps from each source, in the frame of the point clouds.
        target: The object's points, metres.
        obstacles: Points the gripper must not overlap, by name, metres.
        contact_margin: Closer than this (metres) to the object is a touch.
        obstacle_margin: Closer than this (metres) to an obstacle is an overlap.
        planning_group: Group whose tool tip the grasps pose.

    Returns the grasps that pass and the facts of the check: ``checked``,
    ``rejected``, how many touched each of "target" and the obstacles,
    ``while_closing`` (rejected only for what the closing fingers hit) and
    ``seconds``; or ``{"skipped": why}`` with every grasp kept when the robot
    model has no gripper spec.
    """
    candidates = proposals.all()
    if not candidates.candidates:
        return proposals, {"checked": 0, "rejected": 0}
    started = time.monotonic()
    verdicts = manipulation.check_grasp_contacts(
        candidates, target, obstacles, contact_margin, obstacle_margin, planning_group
    )
    if verdicts is None:
        return proposals, {"skipped": "the robot model has no gripper spec"}
    if len(verdicts) != len(candidates.candidates):
        raise RuntimeError(
            f"The contact check answered for {len(verdicts)} of "
            f"{len(candidates.candidates)} grasp candidates"
        )
    facts: dict[str, Any] = {
        "checked": len(verdicts),
        "rejected": sum(not verdict.clear for verdict in verdicts),
    }
    for name in ("target", *obstacles):
        facts[name] = sum(name in verdict.touches for verdict in verdicts)
    facts["while_closing"] = sum(verdict.while_closing for verdict in verdicts)
    facts["seconds"] = round(time.monotonic() - started, 3)
    provided_count = 0 if proposals.provided is None else len(proposals.provided.candidates)
    return (
        replace(
            proposals,
            provided=_kept(proposals.provided, verdicts[:provided_count]),
            heuristic=_kept(proposals.heuristic, verdicts[provided_count:]),
        ),
        facts,
    )


def check_in_scene(
    manipulation: ManipulationSpec,
    scene_registration: ObjectSceneRegistrationSpec,
    object_id: str,
    target: PointCloud2,
    proposals: GraspProposals,
    contact_margin: float,
    obstacle_margin: float,
    planning_group: PlanningGroupID,
) -> tuple[GraspProposals, dict[str, Any]]:
    """Check grasps against an object's points and the camera's view of the scene around it.

    Fetches the latest scan's scene with the object masked out, splits it with
    ``scene_obstacles`` and runs ``drop_touching``.

    Args:
        manipulation: The manipulation module, which runs the check.
        scene_registration: Where the latest scan's scene points come from.
        object_id: The object being grasped, from the latest scan.
        target: Its points, metres, in the planning frame.
        proposals: Grasps from each source, same frame.
        contact_margin: Closer than this (metres) to the object is a touch.
        obstacle_margin: Closer than this (metres) to the table or another object is an overlap.
        planning_group: Group whose tool tip the grasps pose.

    Returns what ``drop_touching`` returns; every grasp is kept, with
    ``{"skipped": why}``, when there is no scene in the object's frame or the
    check fails.
    """
    scene = scene_registration.get_full_scene_pointcloud(exclude_object_id=object_id)
    if scene is None or scene.frame_id != target.frame_id:
        return proposals, {"skipped": "no scene point cloud in the object's frame"}
    tips = [
        state.end_effector_pose.position
        for state in manipulation.get_state().groups.values()
        if state.end_effector_pose is not None
    ]
    try:
        return drop_touching(
            manipulation,
            proposals,
            target,
            scene_obstacles(scene, target, tips),
            contact_margin,
            obstacle_margin,
            planning_group,
        )
    except Exception as exc:
        logger.warning("Grasp contact check failed; keeping every grasp", error=str(exc))
        return proposals, {"skipped": f"the check failed: {type(exc).__name__}: {exc}"}


def contact_note(facts: dict[str, Any]) -> str:
    """A sentence saying how many grasps the contact check dropped, with a leading space, or ""."""
    rejected = facts.get("rejected", 0)
    if not rejected:
        return ""
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
    array: GraspCandidateArray | None, verdicts: Sequence[GraspContact]
) -> GraspCandidateArray | None:
    """The candidates of an array whose verdict is clear, in order."""
    if array is None:
        return None
    kept = [c for c, verdict in zip(array.candidates, verdicts, strict=True) if verdict.clear]
    return GraspCandidateArray(array.header, kept)


def _xyz(cloud: PointCloud2) -> NDArray[np.float64]:
    points = np.asarray(cloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    return points[np.all(np.isfinite(points), axis=1)]


def _cloud(points: NDArray[np.float64], like: PointCloud2) -> PointCloud2:
    return PointCloud2.from_numpy(
        points.astype(np.float32), frame_id=like.frame_id, timestamp=like.ts
    )
