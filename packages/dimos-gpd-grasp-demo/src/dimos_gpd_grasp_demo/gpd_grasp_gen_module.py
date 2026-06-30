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
from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Protocol, cast

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

GPD_RUNTIME_HELP = (
    "GPD grasp detection backend is unavailable. Prepare/install the "
    "dimos-gpd-grasp-demo project runtime (for example, prepare the Pixi runtime "
    "for packages/dimos-gpd-grasp-demo) so `from gpd.core import Cloud, "
    "GraspDetector` works in the placed worker."
)


@dataclass(frozen=True, slots=True)
class NormalizedGraspCandidate:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    score: float = 0.0
    width: float = 0.08


class _Backend(Protocol):
    def __call__(self, points_xyz: np.ndarray) -> Sequence[object]: ...


def pointcloud_to_gpd_xyz(pointcloud: PointCloud2) -> np.ndarray:
    """Convert DimOS PointCloud2 to contiguous finite float32 XYZ input for GPD."""
    points = pointcloud.points_f32()
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"PointCloud2 positions must have shape (N, 3), got {points.shape}")
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    finite = np.isfinite(points).all(axis=1)
    return np.ascontiguousarray(points[finite], dtype=np.float32)


class GPDGraspGenModule(Module):
    """Generate grasp poses from existing PointCloud2 inputs using a lazy GPD backend.

    Backend results without score/width metadata are published with safe debug defaults:
    score=0.0 (unknown quality) and width=0.08m (the Rerun visualization default).
    """

    grasp_candidates: Out[GraspCandidateArray]

    def __init__(
        self,
        backend: Callable[[np.ndarray], Sequence[object]] | None = None,
        default_score: float = 0.0,
        default_width_m: float = 0.08,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)
        self._backend = backend
        self._default_score = default_score
        self._default_width_m = default_width_m

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses from an object PointCloud2 and optional scene cloud."""
        del scene_pointcloud
        points = pointcloud_to_gpd_xyz(pointcloud)
        if points.shape[0] == 0:
            empty = self._candidate_array(pointcloud, [])
            self.grasp_candidates.publish(empty)
            return None

        raw_grasps = self._detect(points)
        normalized = [self._normalize_grasp(grasp) for grasp in raw_grasps]
        candidates = self._candidate_array(pointcloud, normalized)
        self.grasp_candidates.publish(candidates)
        if len(candidates) == 0:
            return None
        return candidates.to_pose_array()

    def _detect(self, points_xyz: np.ndarray) -> Sequence[object]:
        if self._backend is not None:
            return self._backend(points_xyz)
        return _run_gpd_backend(points_xyz)

    def _candidate_array(
        self, pointcloud: PointCloud2, grasps: Iterable[NormalizedGraspCandidate]
    ) -> GraspCandidateArray:
        timestamp = pointcloud.ts if pointcloud.ts is not None else 0.0
        header = Header(float(timestamp), pointcloud.frame_id)
        return GraspCandidateArray(
            header=header,
            candidates=[
                GraspCandidate(
                    pose=Pose(Vector3(grasp.position), Quaternion(grasp.orientation_xyzw)),
                    jaw_width=grasp.width,
                    score=grasp.score,
                )
                for grasp in grasps
            ],
        )

    def _normalize_grasp(self, grasp: object) -> NormalizedGraspCandidate:
        if isinstance(grasp, NormalizedGraspCandidate):
            return grasp
        position = _first_attr(grasp, ("position", "translation", "center"))
        orientation = _first_attr(grasp, ("orientation", "quaternion", "rotation"))
        score = _optional_float(grasp, ("score", "quality"), self._default_score)
        width = _optional_float(grasp, ("width", "jaw_width", "grasp_width"), self._default_width_m)
        return NormalizedGraspCandidate(
            position=_position_tuple(position),
            orientation_xyzw=_orientation_tuple(orientation),
            score=score,
            width=width,
        )


def _run_gpd_backend(points_xyz: np.ndarray) -> Sequence[object]:
    try:
        from gpd.core import Cloud, GraspDetector  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(GPD_RUNTIME_HELP) from exc

    try:
        cloud = Cloud(points_xyz)
        detector = GraspDetector.from_preset("eigen")
        return cast("Sequence[object]", detector.detect_grasps(cloud))
    except AttributeError as exc:
        raise RuntimeError(f"{GPD_RUNTIME_HELP} The installed gpd.core API is incompatible.") from exc


def _first_attr(grasp: object, names: tuple[str, ...]) -> object:
    if isinstance(grasp, dict):
        for name in names:
            if name in grasp:
                return grasp[name]
    for name in names:
        if hasattr(grasp, name):
            return getattr(grasp, name)
    raise ValueError(f"GPD grasp result missing one of: {', '.join(names)}")


def _optional_float(grasp: object, names: tuple[str, ...], default: float) -> float:
    try:
        return float(cast("int | float | str", _first_attr(grasp, names)))
    except ValueError:
        return default


def _position_tuple(value: object) -> tuple[float, float, float]:
    seq = _float_sequence(value)
    if len(seq) != 3:
        raise ValueError("GPD grasp position must contain 3 values")
    return (seq[0], seq[1], seq[2])


def _orientation_tuple(value: object) -> tuple[float, float, float, float]:
    array = np.asarray(value, dtype=np.float64)
    if array.shape == (3, 3):
        quat = Quaternion.from_rotation_matrix(array).normalize()
        return quat.to_tuple()
    seq = _float_sequence(value)
    if len(seq) != 4:
        raise ValueError("GPD grasp orientation must be a quaternion or 3x3 rotation matrix")
    quat = Quaternion(seq).normalize()
    return quat.to_tuple()


def _float_sequence(value: object) -> list[float]:
    if isinstance(value, np.ndarray):
        return [float(item) for item in value.reshape(-1).tolist()]
    if not isinstance(value, Sequence) or isinstance(value, str | bytes):
        raise ValueError(f"Expected numeric sequence, got {type(value).__name__}")
    return [float(item) for item in cast("Sequence[int | float]", value)]
