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

"""Build depth and lidar voxel maps with the dimos voxel mapper and compare them."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from tools.depth_voxel.pipeline import DepthProjector, PoseTrack, lidar_clouds

if TYPE_CHECKING:
    from tools.depth_voxel.filters import DepthFilter

VOXEL_SIZE_METERS = 0.05
MAP_DEVICE = "CPU:0"
NEIGHBOR_TOLERANCE_VOXELS = 1


def shared_window(store: Any) -> tuple[float, float]:
    """The interval covered by depth, lidar and odometry alike."""
    spans = []
    for name, payload in (("depth_image", Image), ("lidar", PointCloud2)):
        stream = store.stream(name, payload)
        spans.append((stream.first().ts, stream.order_by("ts", desc=True).first().ts))
    poses = store.stream("odometry")
    spans.append((poses.first().ts, poses.order_by("ts", desc=True).first().ts))
    return max(lo for lo, _ in spans), min(hi for _, hi in spans)


def build_map(clouds: Any, voxel_size: float = VOXEL_SIZE_METERS) -> VoxelGrid:
    grid = VoxelGrid(
        voxel_size=voxel_size,
        device=MAP_DEVICE,
        carve_columns=True,
        show_startup_log=False,
    )
    for cloud in clouds:
        grid.add_frame(cloud)
    return grid


def build_depth_map(
    store: Any,
    projector: DepthProjector,
    start: float,
    end: float,
    depth_filter: DepthFilter,
    frame_stride: int,
    voxel_size: float = VOXEL_SIZE_METERS,
) -> VoxelGrid:
    depth_filter.reset()

    def clouds() -> Any:
        stream = store.stream("depth_image", Image).after(start).before(end)
        for index, obs in enumerate(stream):
            if index % frame_stride:
                continue
            cloud = projector.world_cloud(depth_filter(obs.data.data), float(obs.ts))
            if cloud is not None:
                yield cloud

    return build_map(clouds(), voxel_size)


def build_lidar_map(
    store: Any,
    poses: PoseTrack,
    start: float,
    end: float,
    voxel_size: float = VOXEL_SIZE_METERS,
) -> VoxelGrid:
    return build_map(lidar_clouds(store, poses, start, end), voxel_size)


def voxel_keys(grid: VoxelGrid, voxel_size: float = VOXEL_SIZE_METERS) -> np.ndarray:
    """Integer voxel coordinates of an accumulated map, (N, 3) int64."""
    points = grid.get_global_pointcloud2().points_f32()
    return np.floor(points.astype(np.float64) / voxel_size).astype(np.int64)


@dataclass(frozen=True)
class OverlapStats:
    depth_voxels: int
    lidar_voxels: int
    intersection: int
    union: int
    iou: float
    depth_precision: float
    lidar_recall: float
    depth_precision_tolerant: float
    lidar_recall_tolerant: float

    def summary(self) -> str:
        return (
            f"depth {self.depth_voxels} | lidar {self.lidar_voxels} | "
            f"exact overlap {self.intersection} | IoU {self.iou:.4f} | "
            f"precision {self.depth_precision:.4f} recall {self.lidar_recall:.4f} | "
            f"±{NEIGHBOR_TOLERANCE_VOXELS}vox precision {self.depth_precision_tolerant:.4f} "
            f"recall {self.lidar_recall_tolerant:.4f}"
        )

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _pack(keys: np.ndarray) -> np.ndarray:
    shifted = keys + (1 << 20)
    return (shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2]


def _dilate(keys: np.ndarray, radius: int) -> np.ndarray:
    offsets = np.stack(
        np.meshgrid(*[np.arange(-radius, radius + 1)] * 3, indexing="ij"), axis=-1
    ).reshape(-1, 3)
    return np.unique(_pack((keys[:, None, :] + offsets[None, :, :]).reshape(-1, 3)))


def compare(
    depth_keys: np.ndarray,
    lidar_keys: np.ndarray,
    tolerance: int = NEIGHBOR_TOLERANCE_VOXELS,
) -> OverlapStats:
    depth_packed = np.unique(_pack(depth_keys))
    lidar_packed = np.unique(_pack(lidar_keys))
    intersection = int(np.intersect1d(depth_packed, lidar_packed, assume_unique=True).size)
    union = len(depth_packed) + len(lidar_packed) - intersection
    depth_near_lidar = int(np.isin(depth_packed, _dilate(lidar_keys, tolerance)).sum())
    lidar_near_depth = int(np.isin(lidar_packed, _dilate(depth_keys, tolerance)).sum())
    return OverlapStats(
        depth_voxels=len(depth_packed),
        lidar_voxels=len(lidar_packed),
        intersection=intersection,
        union=union,
        iou=intersection / union if union else 0.0,
        depth_precision=intersection / len(depth_packed) if len(depth_packed) else 0.0,
        lidar_recall=intersection / len(lidar_packed) if len(lidar_packed) else 0.0,
        depth_precision_tolerant=depth_near_lidar / len(depth_packed) if len(depth_packed) else 0.0,
        lidar_recall_tolerant=lidar_near_depth / len(lidar_packed) if len(lidar_packed) else 0.0,
    )
