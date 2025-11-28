# Copyright 2025 Dimensional Inc.
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

import time
from typing import TypedDict

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.types.timestamped import to_human_readable


class RawLidarPoints(TypedDict):
    points: np.ndarray  # type: ignore[type-arg]  # Shape (N, 3) array of 3D points [x, y, z]


class RawLidarData(TypedDict):
    """Data portion of the LIDAR message"""

    frame_id: str
    origin: list[float]
    resolution: float
    src_size: int
    stamp: float
    width: list[int]
    data: RawLidarPoints


class RawLidarMsg(TypedDict):
    """Static type definition for raw LIDAR message"""

    type: str
    topic: str
    data: RawLidarData


class LidarMessage(PointCloud2):
    resolution: float  # we lose resolution when encoding PointCloud2
    origin: Vector3
    raw_msg: RawLidarMsg | None
    # _costmap: Optional[Costmap] = None  # TODO: Fix after costmap migration

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(
            pointcloud=kwargs.get("pointcloud"),
            ts=kwargs.get("ts"),
            frame_id="world",
        )

        self.origin = kwargs.get("origin")  # type: ignore[assignment]
        self.resolution = kwargs.get("resolution", 0.05)

    @classmethod
    def from_msg(cls: type["LidarMessage"], raw_message: RawLidarMsg, **kwargs) -> "LidarMessage":  # type: ignore[no-untyped-def]
        data = raw_message["data"]
        points = data["data"]["points"]
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(points)

        origin = Vector3(data["origin"])
        # webrtc decoding via native decompression doesn't require us
        # to shift the pointcloud by it's origin
        #
        # pointcloud.translate((origin / 2).to_tuple())
        cls_data = {
            "origin": origin,
            "resolution": data["resolution"],
            "pointcloud": pointcloud,
            # - this is broken in unitree webrtc api "stamp":1.758148e+09
            "ts": time.time(),  # data["stamp"],
            "raw_msg": raw_message,
            **kwargs,
        }
        return cls(**cls_data)

    def __repr__(self) -> str:
        return f"LidarMessage(ts={to_human_readable(self.ts)}, origin={self.origin}, resolution={self.resolution}, {self.pointcloud})"

    def __iadd__(self, other: "LidarMessage") -> "LidarMessage":  # type: ignore[override]
        self.pointcloud += other.pointcloud
        return self

    def __add__(self, other: "LidarMessage") -> "LidarMessage":  # type: ignore[override]
        # Determine which message is more recent
        if self.ts >= other.ts:
            ts = self.ts
            origin = self.origin
            resolution = self.resolution
        else:
            ts = other.ts
            origin = other.origin
            resolution = other.resolution

        # Return a new LidarMessage with combined data
        return LidarMessage(  # type: ignore[attr-defined, no-any-return]
            ts=ts,
            origin=origin,
            resolution=resolution,
            pointcloud=self.pointcloud + other.pointcloud,
        ).estimate_normals()

    @property
    def o3d_geometry(self):  # type: ignore[no-untyped-def]
        return self.pointcloud

    # TODO: Fix after costmap migration
    # def costmap(self, voxel_size: float = 0.2) -> Costmap:
    #     if not self._costmap:
    #         down_sampled_pointcloud = self.pointcloud.voxel_down_sample(voxel_size=voxel_size)
    #         inflate_radius_m = 1.0 * voxel_size if voxel_size > self.resolution else 0.0
    #         grid, origin_xy = pointcloud_to_costmap(
    #             down_sampled_pointcloud,
    #             resolution=self.resolution,
    #             inflate_radius_m=inflate_radius_m,
    #         )
    #         self._costmap = Costmap(grid=grid, origin=[*origin_xy, 0.0], resolution=self.resolution)
    #
    #     return self._costmap
