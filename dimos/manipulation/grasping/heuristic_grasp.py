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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class HeuristicGraspConfig(ModuleConfig):
    """Configuration for the heuristic grasp generator.

    Attributes:
        tool_rotation_rpy: Fixed rotation from the canonical top-down grasp
            frame to the robot's own grasp frame, in radians. Grippers whose
            URDF frame does not point Z along the approach need this, or every
            proposal comes back unreachable.
    """

    tool_rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Extra wrist yaws to offer alongside the narrow-axis one. A parallel jaw
    # is symmetric under a half turn, and on a round object the narrow axis is
    # arbitrary, so these are the same physical grasp reached differently -
    # which matters when one wrist angle falls outside an arm's envelope.
    yaw_candidates: int = 1
    # A camera looking down from one side sees a tall object's top and its
    # near face, so the cloud's median sits toward the camera, off the
    # object's axis, and a jaw closing there pushes the object away. For an
    # object taller than tall_object_height the centre and the narrow axis
    # come from the top top_slice of the cloud instead, which lies on the axis.
    tall_object_height: float = 0.04
    top_slice: float = 0.02
    # An object whose cross-section fits between the open jaws along its wide
    # axis too can be grasped at any wrist yaw, so every yaw is offered for it;
    # a partly seen round lid rarely looks round enough for the axis test alone.
    jaw_opening: float = 0.09


class HeuristicGraspModule(Module, GraspGenSpec):
    """Generate one top-down parallel-jaw grasp from a gravity-aligned point cloud.

    The input frame's XY plane must be horizontal and its -Z axis must point down.
    """

    config: HeuristicGraspConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        if object_pointcloud.ts is None or not math.isfinite(float(object_pointcloud.ts)):
            raise ValueError("object pointcloud must have a finite timestamp")
        if not object_pointcloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")
        points = object_pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("object pointcloud must contain at least three XYZ points")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
        xy = points[:, :2]
        if high_z - low_z > self.config.tall_object_height:
            top = points[points[:, 2] >= high_z - self.config.top_slice]
            if len(top) >= 3:
                xy = top[:, :2]
        base_yaw, ambiguous = self._narrow_axis_yaw(xy)
        mean_xy = np.mean(xy, axis=0)
        centered = xy - mean_xy
        u_along = np.array([math.cos(base_yaw), math.sin(base_yaw)])
        u_across = np.array([-math.sin(base_yaw), math.cos(base_yaw)])
        along = centered @ u_along
        across = centered @ u_across
        # 5th to 95th percentile: a mask that spills onto the table adds a few
        # far points that must not decide the grasp.
        along_lo, along_hi = np.quantile(along, [0.05, 0.95])
        wide = float(along_hi - along_lo)
        narrow = float(np.diff(np.quantile(across, [0.05, 0.95]))[0])
        # Grasp the middle of the object's length, not the median point: a
        # camera to one side sees one end of a tube more densely than the
        # other, and the median then sits near that end, where the jaws close
        # on a cap or a crimp. Across the jaws the median is the right centre.
        center_xy = (
            mean_xy
            + float((along_lo + along_hi) / 2.0) * u_along
            + float(np.median(across)) * u_across
        )
        position = Vector3(float(center_xy[0]), float(center_xy[1]), float((low_z + high_z) / 2.0))
        if not ambiguous:
            ambiguous = wide <= self.config.jaw_opening
        logger.info(
            "Grasp cross-section: height %.3f m, %d points%s, wide %.3f m, narrow %.3f m, "
            "yaw %.0f deg, %s",
            high_z - low_z,
            len(xy),
            " (top slice)" if len(xy) < len(points) else "",
            wide,
            narrow,
            math.degrees(base_yaw),
            "all yaws offered" if ambiguous else "narrow-axis yaws only",
        )
        candidates = [
            GraspCandidate(
                Pose(
                    position,
                    self._apply_tool_rotation(
                        Quaternion.from_euler(Vector3(-math.pi, 0.0, base_yaw + delta))
                    ),
                ),
                score=score,
            )
            for delta, score in self._yaw_offsets(ambiguous)
        ]
        return GraspCandidateArray(
            Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
            candidates,
        )

    def _yaw_offsets(self, ambiguous: bool) -> list[tuple[float, float]]:
        """Wrist yaw deltas to try, best first. The narrow-axis grasp stays first.

        A half turn is always safe: parallel jaws are symmetric. Anything else
        would grasp across the wide axis, so it is offered only when the cross
        section has no narrow axis to speak of.
        """
        quarter = math.pi / 2.0
        eighth = math.pi / 4.0
        offsets = [(0.0, 1.0), (math.pi, 0.95)]
        if ambiguous:
            offsets += [
                (quarter, 0.9),
                (-quarter, 0.85),
                (eighth, 0.8),
                (-eighth, 0.75),
                (math.pi - eighth, 0.7),
                (eighth - math.pi, 0.65),
            ]
        return offsets[: max(1, self.config.yaw_candidates)]

    def _apply_tool_rotation(self, orientation: Quaternion) -> Quaternion:
        roll, pitch, yaw = self.config.tool_rotation_rpy
        if (roll, pitch, yaw) == (0.0, 0.0, 0.0):
            return orientation
        return orientation * Quaternion.from_euler(Vector3(roll, pitch, yaw))

    @staticmethod
    def _narrow_axis_yaw(xy: NDArray[np.float32]) -> tuple[float, bool]:
        """Yaw of the cross-section's narrow axis, and whether it is ambiguous."""
        centered = xy - np.mean(xy, axis=0)
        covariance = centered.T @ centered
        values, vectors = np.linalg.eigh(covariance)
        if values[1] <= 0.0 or np.isclose(values[0], values[1], rtol=0.05):
            return 0.0, True
        narrow_axis = vectors[:, 0]
        yaw = math.atan2(float(narrow_axis[1]), float(narrow_axis[0])) - math.pi / 2.0
        # A parallel-jaw grasp is unchanged by a 180-degree wrist rotation.
        return (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0, False
