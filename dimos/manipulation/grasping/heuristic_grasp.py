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

"""Deterministic grasp proposals for segmented object point clouds."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.protocol.service.spec import BaseConfig

# Scores only rank the candidates: the centre grasp, then one in each half of
# the object along its length, then the centre grasp turned a quarter turn.
_CENTER_SCORE = 1.0
_HALF_SCORES = (0.8, 0.7)
_TURNED_SCORE = 0.5
# A footprint whose length and width differ by less than this (metres) is
# round: the grasp turned a quarter turn holds it the same way.
_ROUND_TOLERANCE = 0.01


class HeuristicGraspConfig(BaseConfig):
    """How the heuristic grasps fit the gripper."""

    # Opening (metres) of the open jaws. A grasp is offered only where the object
    # is narrower than this by jaw_clearance across the jaws; the centre grasp
    # is always offered.
    max_grasp_width: float = Field(default=0.085, gt=0.0)
    # Room (metres) the open jaws need beside the object, for point-cloud error
    # and a slightly off-centre approach.
    jaw_clearance: float = Field(default=0.01, ge=0.0)
    # Objects at least this long (metres) also get one grasp in the middle of
    # each half along their length. Shorter objects would get grasps too close
    # to the centre one to differ from it.
    min_split_length: float = Field(default=0.10, gt=0.0)
    # Put the closed finger pads, not the tool tip, at the object's mid-height,
    # keeping the fingertips above the surface under the object and the palm
    # above its top. False puts the tool tip at mid-height.
    heuristic_aim_pads: bool = True
    # How far (metres) the closed finger pads' centre sits behind the tool tip,
    # along the approach. 0 aims the tool tip itself at mid-height.
    pad_behind_tcp: float = 0.0
    # How far (metres) the fingertips reach past the tool tip, along the
    # approach, at most while the jaws close. Negative when they stay behind it.
    fingertip_reach: float = 0.0
    # How far (metres) the nearest gripper part other than the fingers (the palm
    # or a knuckle) sits behind the tool tip with the jaws open. None sets no
    # limit, so on a tall object the palm can come down onto its top.
    palm_behind_tcp: float | None = None
    # Least gap (metres) kept between the fingertips and the surface under the
    # object, and between the palm and the object's top.
    gripper_clearance: float = Field(default=0.005, ge=0.0)


class HeuristicGraspModuleConfig(ModuleConfig, HeuristicGraspConfig):
    """HeuristicGraspModule's settings: the grasp rules plus the usual module settings."""


class HeuristicGraspModule(Module, GraspGenSpec):
    """Generate a few top-down parallel-jaw grasps from a gravity-aligned point cloud.

    See ``heuristic_grasps`` for the grasps it returns.
    """

    config: HeuristicGraspModuleConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        return heuristic_grasps(object_pointcloud, self.config)


def heuristic_grasps(
    object_pointcloud: PointCloud2, config: HeuristicGraspConfig
) -> GraspCandidateArray:
    """A few straight-down parallel-jaw grasps for one object, best first.

    The first grasp is at the middle of the object with the jaws closing across
    its narrow side. When there is room, the others try each half of a long
    object, and the middle grasp turned a quarter turn, so a retry after a
    miss has something different to try. Each grasp puts the closed finger pads
    at mid-height of the points it holds, unless the fingertips would come too
    close to what the object stands on, or the palm to the object's top.

    Args:
        object_pointcloud: The object's points, metres, in a frame whose XY plane
            is horizontal and whose Z axis points up. In a tilted frame the
            grasps come out tilted too, with no error.
        config: Jaw opening, spacing rules and finger pad offsets, metres.

    Returns the grasps in the cloud's frame and with its timestamp. Raises
    ValueError for a cloud with fewer than three points, non-finite values,
    no frame_id or no finite timestamp.
    """
    if object_pointcloud.ts is None or not math.isfinite(float(object_pointcloud.ts)):
        raise ValueError("object pointcloud must have a finite timestamp")
    if not object_pointcloud.frame_id:
        raise ValueError("object pointcloud frame_id must not be empty")
    points = object_pointcloud.points_f32()
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("object pointcloud must contain at least three XYZ points")
    if not np.all(np.isfinite(points)):
        raise ValueError("object pointcloud XYZ values must be finite floats in metres")

    # Height (metres) of the surface the object stands on: its low end.
    support = float(np.quantile(points[:, 2], 0.05))
    xy = points[:, :2]
    yaw = _narrow_axis_yaw(xy)
    candidates = [GraspCandidate(_top_down(points, yaw, config, support), score=_CENTER_SCORE)]

    # The jaws close along jaw_axis; length_axis lies along the object.
    length_axis = np.array([math.cos(yaw), math.sin(yaw)])
    jaw_axis = np.array([-length_axis[1], length_axis[0]])
    # Each point's position (metres) along the object's length and across it.
    along = xy @ length_axis
    across = xy @ jaw_axis
    fits = config.max_grasp_width - config.jaw_clearance
    start, end = np.quantile(along, [0.01, 0.99])
    length = float(end - start)

    if length >= config.min_split_length:
        middle = (start + end) / 2.0
        halves = [points[along < middle], points[along >= middle]]
        halves = [
            half
            for half in sorted(halves, key=len, reverse=True)
            if len(half) >= 3 and _span(half[:, :2] @ jaw_axis) <= fits
        ]
        candidates += [
            GraspCandidate(_top_down(half, yaw, config, support), score=score)
            for half, score in zip(halves, _HALF_SCORES, strict=False)
        ]
    if length <= fits and abs(length - _span(across)) >= _ROUND_TOLERANCE:
        turned = _top_down(points, yaw + math.pi / 2.0, config, support)
        candidates.append(GraspCandidate(turned, score=_TURNED_SCORE))
    return GraspCandidateArray(
        Header(float(object_pointcloud.ts), object_pointcloud.frame_id), candidates
    )


def _narrow_axis_yaw(xy: NDArray[np.float32]) -> float:
    """Gripper yaw (radians) that closes the jaws across the narrow side of some XY
    points (metres); 0 when the footprint is about as wide as it is long."""
    centered = xy - np.mean(xy, axis=0)
    covariance = centered.T @ centered
    values, vectors = np.linalg.eigh(covariance)
    if values[1] <= 0.0 or np.isclose(values[0], values[1], rtol=0.05):
        return 0.0
    narrow_axis = vectors[:, 0]
    yaw = math.atan2(float(narrow_axis[1]), float(narrow_axis[0])) - math.pi / 2.0
    # A parallel-jaw grasp is unchanged by a 180-degree wrist rotation.
    return (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0


def _top_down(
    points: NDArray[np.float32], yaw: float, config: HeuristicGraspConfig, support: float
) -> Pose:
    """A straight-down grasp at the middle of some points.

    Args:
        points: XYZ points in metres, Z up.
        yaw: Gripper yaw about the vertical, radians. The jaws close at right
            angles to the horizontal direction (cos yaw, sin yaw).
        config: Where the grasp puts the finger pads; see HeuristicGraspConfig.
        support: Height (metres) of the surface the object stands on.

    The grasp is at the points' median XY. Halfway up between their low and
    high ends (5th and 95th height percentiles) go the closed finger pads,
    raised where needed to keep the fingertips above the support and the palm
    above the points' high end. With ``heuristic_aim_pads`` off the tool tip
    goes there instead.
    """
    center_xy = np.median(points[:, :2], axis=0)
    low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
    z = float((low_z + high_z) / 2.0)
    if config.heuristic_aim_pads:
        # The approach points straight down: the pads sit pad_behind_tcp above the
        # tool tip, the fingertips reach fingertip_reach below it, and the palm
        # sits palm_behind_tcp above it.
        floors = [support + config.gripper_clearance + config.fingertip_reach]
        if config.palm_behind_tcp is not None:
            floors.append(float(high_z) + config.gripper_clearance - config.palm_behind_tcp)
        z = max(z - config.pad_behind_tcp, *floors)
    return Pose(
        Vector3(float(center_xy[0]), float(center_xy[1]), z),
        Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)),
    )


def _span(values: NDArray[np.float64]) -> float:
    """Width (metres) of a set of distances, between their 1st and 99th percentiles."""
    low, high = np.quantile(values, [0.01, 0.99])
    return float(high - low)
