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

from dataclasses import dataclass
import math

import cv2
import numpy as np
from numpy.typing import NDArray

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

# Fraction of the XY points, farthest from the median, dropped as depth noise before the fit.
_TRIM_QUANTILE = 0.02
# An open container too wide for the jaws is held by a wall. A wall's own points lie within
# _WALL_BAND of the footprint's edge; one not seen sits _WALL_HALF beyond the floor's edge; a wall
# grasp closes _WALL_GRIP above the floor; and the footprint's middle must be seen at least
# _OPEN_DROP below the top to count as open. Walls are assumed this thick or less on each side of
# the footprint when deciding whether the jaws reach around an open container.
_WALL_BAND = 0.01
_WALL_HALF = 0.004
_WALL_GRIP = 0.03
_OPEN_DROP = 0.03
_WALL_MARGIN = 0.01
_MIN_WALL_POINTS = 5
# Least gap (metres) kept between the fingertips and what the object stands on,
# and between the palm and the object's top.
_CLEARANCE = 0.005


@dataclass(frozen=True)
class GripperAim:
    """Where the gripper's parts sit along its approach, measured from the tool tip, metres.
    Left at the defaults, the tool tip itself goes where the closed pads should be."""

    # The closed finger pads' centre, behind the tool tip.
    pad_behind_tcp: float = 0.0
    # The farthest the fingertips reach past the tool tip while closing.
    fingertip_reach: float | None = None
    # The nearest part other than the fingers (the palm), behind the tool tip.
    palm_behind_tcp: float | None = None

    def tip_height(self, pad_z: float, low_z: float, high_z: float) -> float:
        """Tool tip height (metres) that puts the closed pads at ``pad_z``, raised if needed
        so the fingertips stay above ``low_z`` (what the object stands on) and the palm
        above ``high_z`` (the top of what goes between the jaws)."""
        # Straight down: the pads and palm sit above the tool tip, the fingertips below.
        heights = [float(pad_z) - self.pad_behind_tcp]
        if self.fingertip_reach is not None:
            heights.append(float(low_z) + _CLEARANCE + self.fingertip_reach)
        if self.palm_behind_tcp is not None:
            heights.append(float(high_z) + _CLEARANCE - self.palm_behind_tcp)
        return max(heights)


class HeuristicGraspModuleConfig(ModuleConfig):
    # Widest object the open jaws close on, metres; the xArm gripper opens to 88.9 mm.
    max_jaw_span: float = 0.085
    # Least depth below the object's top the grasp goes, metres, when little more than the
    # top was seen.
    grasp_depth: float = 0.02
    # Where the gripper's parts sit along its approach (see GripperAim), metres.
    pad_behind_tcp: float = 0.0
    fingertip_reach: float | None = None
    palm_behind_tcp: float | None = None


class HeuristicGraspModule(Module, GraspGenSpec):
    """Top-down parallel-jaw grasps from a gravity-aligned point cloud.

    The input frame's XY plane must be horizontal and its -Z axis must point down.
    """

    config: HeuristicGraspModuleConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        config = self.config
        aim = GripperAim(config.pad_behind_tcp, config.fingertip_reach, config.palm_behind_tcp)
        return top_down_grasps(object_pointcloud, config.max_jaw_span, config.grasp_depth, aim)


def top_down_grasps(
    object_pointcloud: PointCloud2,
    max_jaw_span: float,
    grasp_depth: float = 0.02,
    aim: GripperAim = GripperAim(),
) -> GraspCandidateArray:
    """Grasps straight down at the centre of the cloud's footprint, half way up the surfaces
    seen and at least ``grasp_depth`` below the top: a camera above an object may see only its
    top, and a grasp there closes on nothing. The footprint is the smallest rectangle around the XY points, so its sides follow
    the object's faces at any yaw. The first candidate closes the jaws across the short side;
    a second closes them across the long side when the jaws open that wide.

    An object wider than the jaws on both sides is grasped on a wall when it is an open
    container (its inside seen well below its rim): jaws across the wall, a little below the
    rim, the long walls ahead of the short ones. A solid object that wide gets no grasp.

    ``aim`` places the tool tip so the closed finger pads, not the tip, are at those heights."""
    if object_pointcloud.ts is None or not math.isfinite(float(object_pointcloud.ts)):
        raise ValueError("object pointcloud must have a finite timestamp")
    if not object_pointcloud.frame_id:
        raise ValueError("object pointcloud frame_id must not be empty")
    points = object_pointcloud.points_f32()
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("object pointcloud must contain at least three XYZ points")
    if not np.all(np.isfinite(points)):
        raise ValueError("object pointcloud XYZ values must be finite floats in metres")

    (cx, cy), sides, short_yaw = _footprint(points[:, :2])
    low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
    # The jaws cannot close across it (or barely: an open container is seen from inside its
    # walls, so its outside is wider than its footprint): grasp a wall.
    if (sides[0] > max_jaw_span - 2 * _WALL_MARGIN and _is_open(points, high_z)) or sides[
        0
    ] > max_jaw_span:
        return GraspCandidateArray(
            Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
            _wall_grasps(points, (cx, cy), sides, short_yaw, float(high_z), aim),
        )
    pad_z = min((low_z + high_z) / 2.0, high_z - grasp_depth)
    position = Vector3(cx, cy, aim.tip_height(pad_z, low_z, high_z))
    candidates = [
        GraspCandidate(
            _top_down(position, short_yaw),
            score=1.0,
            note="footprint centre, across its narrow side",
        )
    ]
    if max(sides) <= max_jaw_span and not math.isclose(sides[0], sides[1], rel_tol=0.05):
        candidates.append(
            GraspCandidate(
                _top_down(position, short_yaw + math.pi / 2),
                score=0.5,
                note="footprint centre, across its long side",
            )
        )
    return GraspCandidateArray(
        Header(float(object_pointcloud.ts), object_pointcloud.frame_id), candidates
    )


def _wall_grasps(
    points: NDArray[np.float32],
    centre: tuple[float, float],
    sides: tuple[float, float],
    short_yaw: float,
    high_z: float,
    aim: GripperAim = GripperAim(),
) -> list[GraspCandidate]:
    """Top-down grasps on the walls of an open container, jaws across each wall at its middle,
    ``_WALL_GRIP`` above the floor seen inside (lower when the wall's seen top is lower). From
    above, a camera sees mostly the floor and little of the walls, so a wall with too few points
    of its own is placed just beyond the floor's edge. None when the footprint's middle is not
    seen below the top (a solid top)."""
    across = np.asarray([math.cos(short_yaw), math.sin(short_yaw)])  # along the short side
    along = np.asarray([-across[1], across[0]])  # along the long side
    xy = points[:, :2] - np.asarray(centre)
    v, u, z = xy @ across, xy @ along, points[:, 2]
    half_short, half_long = sides[0] / 2.0, sides[1] / 2.0
    middle = (np.abs(u) < half_long / 2.0) & (np.abs(v) < half_short / 2.0)
    if not middle.any() or np.quantile(z[middle], 0.9) > high_z - _OPEN_DROP:
        return []
    floor = float(np.median(z[middle]))
    candidates = []
    # Long walls first: held at its middle, a long wall leaves the load closest under the jaws.
    for normal, tangent, half_t, n_axis, jaw_yaw, score in (
        (v, u, half_long, across, short_yaw, 0.9),
        (u, v, half_short, along, short_yaw + math.pi / 2.0, 0.6),
    ):
        for sign in (1.0, -1.0):
            outward = sign * normal
            edge = float(np.quantile(outward, 0.98))
            # Away from the corners, where the neighbouring walls' points lie.
            wall = (
                (outward > edge - _WALL_BAND)
                & (z > floor + 0.01)
                & (np.abs(tangent) < half_t * 0.7)
            )
            grip = floor + _WALL_GRIP
            kind = "long wall" if score > 0.75 else "short wall"
            if wall.sum() >= _MIN_WALL_POINTS:
                n = float(np.median(outward[wall]))
                grip = min(grip, float(np.quantile(z[wall], 0.98)) - 0.005)
                note = f"{kind}, {int(wall.sum())} of its points seen"
            else:
                n = edge + _WALL_HALF
                note = (
                    f"{kind} not seen: placed {_WALL_HALF * 1000:.0f} mm beyond the floor's edge "
                    f"and {_WALL_GRIP * 100:.0f} cm above the floor; check it in an image"
                )
            if grip < floor + 0.015:
                continue
            xy_w = np.asarray(centre) + n_axis * (sign * n)
            top = float(np.quantile(z[wall], 0.98)) if wall.sum() >= _MIN_WALL_POINTS else high_z
            position = Vector3(float(xy_w[0]), float(xy_w[1]), aim.tip_height(grip, floor, top))
            candidates.append(GraspCandidate(_top_down(position, jaw_yaw), score=score, note=note))
    return candidates


def _is_open(points: NDArray[np.float32], high_z: float) -> bool:
    """Whether the middle of the cloud's footprint is seen well below its top: an open container."""
    (cx, cy), sides, short_yaw = _footprint(points[:, :2])
    across = np.asarray([math.cos(short_yaw), math.sin(short_yaw)])
    along = np.asarray([-across[1], across[0]])
    xy = points[:, :2] - np.asarray([cx, cy])
    middle = (np.abs(xy @ along) < sides[1] / 4.0) & (np.abs(xy @ across) < sides[0] / 4.0)
    return bool(middle.any()) and float(np.quantile(points[middle, 2], 0.9)) <= high_z - _OPEN_DROP


def _footprint(xy: NDArray[np.float32]) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Centre, side lengths and the short side's direction (radians) of the smallest
    rectangle around ``xy``, with the outermost points dropped as depth noise."""
    # Trimmed by distance from the median, not per axis: a per-axis cut clips a turned
    # object's corners unevenly and tilts the rectangle.
    distance = np.linalg.norm(xy - np.median(xy, axis=0), axis=1)
    kept = xy[distance <= np.quantile(distance, 1.0 - _TRIM_QUANTILE)]
    if len(kept) < 3:
        kept = xy
    corners = cv2.boxPoints(cv2.minAreaRect(kept.astype(np.float32)))
    edges = [corners[1] - corners[0], corners[2] - corners[1]]
    lengths = [float(np.hypot(*edge)) for edge in edges]
    short = edges[int(np.argmin(lengths))]
    centre = corners.mean(axis=0)
    return (
        (float(centre[0]), float(centre[1])),
        (min(lengths), max(lengths)),
        math.atan2(float(short[1]), float(short[0])),
    )


def _top_down(position: Vector3, jaw_yaw: float) -> Pose:
    """A TCP pose pointing straight down whose jaws close along ``jaw_yaw``. Flipped to point
    down, the TCP's y axis (the jaw axis) lies a quarter turn from its yaw; a parallel-jaw grasp
    is unchanged by a half turn of the wrist, so the yaw is folded into [-90, 90) degrees."""
    yaw = jaw_yaw % math.pi - math.pi / 2.0
    return Pose(position, Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)))
