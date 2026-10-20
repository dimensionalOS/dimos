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

"""Drop grasp candidates whose gripper would touch the object off its pads, or hit the scene."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.manipulation.planning.gripper_contact import GripperContactModel
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.transform_utils import pose_to_matrix

# How far (metres) beyond the object's points scene points are kept: farther
# than any part of the gripper reaches from a grasp on the object.
_REACH = 0.25
# Scene points this close (metres) to a gripper's tool tip are that gripper: the
# wrist camera sees its own fingers.
_GRIPPER_SELF_RADIUS = 0.15
# Scene points this close (metres) to the object's points are the object's own
# edge left outside its mask.
_TARGET_EDGE = 0.01


def scene_obstacles(
    scene: PointCloud2, target: PointCloud2, tips: Sequence[Vector3]
) -> NDArray[np.float64]:
    """The scene points around an object that a gripper grasping it must keep clear of.

    Args:
        scene: Camera points with the object masked out, metres, in the object's frame.
        target: The object's points, metres.
        tips: Where each gripper's tool tip is, metres, same frame; points near
            them are the robot's own gripper and are dropped.

    Returns the points within reach of the object, less the robot's grippers
    and the object's own edge, shape (N, 3), metres.
    """
    points = _xyz(scene)
    own = _xyz(target)
    keep = np.zeros(len(points), dtype=np.bool_)
    if len(own):
        low, high = own.min(axis=0) - _REACH, own.max(axis=0) + _REACH
        keep = np.all((points >= low) & (points <= high), axis=1)
    for tip in tips:
        keep &= np.linalg.norm(points - tip.to_numpy(), axis=1) > _GRIPPER_SELF_RADIUS
    points = points[keep]
    if len(own) and len(points):
        distance, _ = cKDTree(own).query(points)
        points = points[distance > _TARGET_EDGE]
    return points


def drop_touching(
    model: GripperContactModel,
    candidates: GraspCandidateArray,
    target: PointCloud2,
    obstacles: NDArray[np.float64],
    contact_margin: float,
    obstacle_margin: float,
) -> GraspCandidateArray:
    """Keep the grasps whose gripper touches the object only at its contact patches.

    Args:
        model: The gripper's shape, posed from the tool tip the grasps give.
        candidates: Grasps, in the frame of the points.
        target: The object's points, metres.
        obstacles: Other scene points the gripper must not overlap, shape (N, 3), metres.
        contact_margin: Closer than this (metres) to the object is a touch.
        obstacle_margin: Closer than this (metres) to an obstacle is an overlap.
    """
    poses = np.asarray([pose_to_matrix(c.pose) for c in candidates.candidates])
    clear = model.check(
        poses.reshape((-1, 4, 4)), target.points_f32(), obstacles, contact_margin, obstacle_margin
    )
    kept = [c for c, ok in zip(candidates.candidates, clear, strict=True) if ok]
    return GraspCandidateArray(candidates.header, kept)


def _xyz(cloud: PointCloud2) -> NDArray[np.float64]:
    points = np.asarray(cloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    return points[np.all(np.isfinite(points), axis=1)]
