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

"""Depth-image -> world-frame point clouds -> dimos voxel map, plus lidar comparison.

The recording gives depth in ``camera_depth_optical_frame`` and point-lio odometry
as ``odom -> mid360_link``. World pose of the depth camera is therefore

    T_world_depth = T_odom_mid360 @ inv(T_base_mid360) @ T_base_depth

with the two ``T_base_*`` legs read from the ``tf_corrected`` tree.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from collections.abc import Iterator

DEPTH_UNIT_METERS = 0.001
"""RealSense DEPTH16 least-significant bit, in meters."""

BASE_FRAME = "base_link"
LIDAR_FRAME = "mid360_link"
DEPTH_FRAME = "camera_depth_optical_frame"


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, info: CameraInfo) -> Intrinsics:
        k = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        return cls(
            fx=float(k[0, 0]),
            fy=float(k[1, 1]),
            cx=float(k[0, 2]),
            cy=float(k[1, 2]),
            width=int(info.width),
            height=int(info.height),
        )


class PoseTrack:
    """Interpolated ``odom -> mid360_link`` poses from the point-lio odometry stream."""

    def __init__(self, timestamps: np.ndarray, positions: np.ndarray, quaternions: np.ndarray):
        self.timestamps = timestamps
        self.positions = positions
        self.quaternions = quaternions

    @classmethod
    def from_store(cls, store: Any, stream: str = "odometry") -> PoseTrack:
        timestamps: list[float] = []
        positions: list[tuple[float, float, float]] = []
        quaternions: list[tuple[float, float, float, float]] = []
        for obs in store.stream(stream, Odometry):
            odom = obs.data
            timestamps.append(float(obs.ts))
            positions.append((odom.x, odom.y, odom.z))
            orientation = odom.orientation
            quaternions.append((orientation.x, orientation.y, orientation.z, orientation.w))
        quaternion_array = np.asarray(quaternions, dtype=np.float64)
        # Hemisphere-align so slerp between neighbours never takes the long way.
        flips = np.sign(np.einsum("ij,ij->i", quaternion_array[:-1], quaternion_array[1:]))
        flips[flips == 0] = 1.0
        quaternion_array[1:] *= np.cumprod(flips)[:, None]
        return cls(
            np.asarray(timestamps, dtype=np.float64),
            np.asarray(positions, dtype=np.float64),
            quaternion_array,
        )

    def at(self, timestamp: float) -> Transform | None:
        """``odom -> mid360_link`` at ``timestamp``, or None if outside the track."""
        times = self.timestamps
        if timestamp < times[0] or timestamp > times[-1]:
            return None
        upper = int(np.searchsorted(times, timestamp))
        if upper == 0:
            position, quaternion = self.positions[0], self.quaternions[0]
        else:
            lower = upper - 1
            span = times[upper] - times[lower]
            weight = 0.0 if span <= 0 else (timestamp - times[lower]) / span
            position = self.positions[lower] * (1 - weight) + self.positions[upper] * weight
            quaternion = _slerp(self.quaternions[lower], self.quaternions[upper], weight)
        matrix = np.eye(4)
        matrix[:3, :3] = _quaternion_to_matrix(quaternion)
        matrix[:3, 3] = position
        return Transform.from_matrix(matrix, frame_id="odom", child_frame_id=LIDAR_FRAME)


def _slerp(start: np.ndarray, end: np.ndarray, weight: float) -> np.ndarray:
    dot = float(np.dot(start, end))
    if dot < 0:
        end, dot = -end, -dot
    if dot > 0.9995:
        result = start + weight * (end - start)
        return result / np.linalg.norm(result)
    angle = np.arccos(dot)
    sin_angle = np.sin(angle)
    return (
        np.sin((1 - weight) * angle) / sin_angle * start + np.sin(weight * angle) / sin_angle * end
    )


def _quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class DepthProjector:
    """Turns DEPTH16 images into world-frame :py:class:`PointCloud2` clouds."""

    def __init__(
        self,
        intrinsics: Intrinsics,
        base_to_depth: np.ndarray,
        base_to_lidar: np.ndarray,
        poses: PoseTrack,
    ):
        self.intrinsics = intrinsics
        self.poses = poses
        self.lidar_to_depth = np.linalg.inv(base_to_lidar) @ base_to_depth
        columns, rows = np.meshgrid(
            np.arange(intrinsics.width, dtype=np.float32),
            np.arange(intrinsics.height, dtype=np.float32),
        )
        self.ray_x = (columns - np.float32(intrinsics.cx)) / np.float32(intrinsics.fx)
        self.ray_y = (rows - np.float32(intrinsics.cy)) / np.float32(intrinsics.fy)

    @classmethod
    def from_store(cls, store: Any, tf: StreamTF, poses: PoseTrack) -> DepthProjector:
        info = next(iter(store.stream("depth_camera_info", CameraInfo)))
        reference_time = float(poses.timestamps[len(poses.timestamps) // 2])
        base_to_depth = tf.get(BASE_FRAME, DEPTH_FRAME, reference_time)
        base_to_lidar = tf.get(BASE_FRAME, LIDAR_FRAME, reference_time)
        if base_to_depth is None or base_to_lidar is None:
            raise ValueError("tf_corrected is missing the camera or lidar mount transform")
        return cls(
            Intrinsics.from_camera_info(info.data),
            base_to_depth.to_matrix(),
            base_to_lidar.to_matrix(),
            poses,
        )

    def camera_points(self, depth: np.ndarray) -> np.ndarray:
        """Unproject a DEPTH16 array to (N, 3) points in the depth optical frame."""
        valid = depth > 0
        if not valid.any():
            return np.empty((0, 3), dtype=np.float32)
        z = depth[valid].astype(np.float32) * np.float32(DEPTH_UNIT_METERS)
        return np.stack([self.ray_x[valid] * z, self.ray_y[valid] * z, z], axis=1)

    def world_cloud(self, depth: np.ndarray, timestamp: float) -> PointCloud2 | None:
        odom_to_lidar = self.poses.at(timestamp)
        if odom_to_lidar is None:
            return None
        points = self.camera_points(depth)
        if not len(points):
            return None
        matrix = odom_to_lidar.to_matrix() @ self.lidar_to_depth
        world = points @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(np.float32)
        return PointCloud2.from_numpy(world, frame_id="world", timestamp=timestamp)


def depth_clouds(
    store: Any,
    projector: DepthProjector,
    start: float,
    end: float,
    depth_filter: Any = None,
) -> Iterator[PointCloud2]:
    """World-frame clouds for every depth frame in ``[start, end]``."""
    stream = store.stream("depth_image", Image).after(start).before(end)
    for obs in stream:
        depth = obs.data.data
        if depth_filter is not None:
            depth = depth_filter(depth)
        cloud = projector.world_cloud(depth, float(obs.ts))
        if cloud is not None:
            yield cloud


def lidar_clouds(store: Any, poses: PoseTrack, start: float, end: float) -> Iterator[PointCloud2]:
    """Point-lio lidar sweeps transformed from ``mid360_link`` into the odom frame."""
    stream = store.stream("lidar", PointCloud2).after(start).before(end)
    for obs in stream:
        pose = poses.at(float(obs.ts))
        if pose is None:
            continue
        points = obs.data.points_f32()
        if not len(points):
            continue
        matrix = pose.to_matrix()
        world = points @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(np.float32)
        yield PointCloud2.from_numpy(world, frame_id="world", timestamp=float(obs.ts))
