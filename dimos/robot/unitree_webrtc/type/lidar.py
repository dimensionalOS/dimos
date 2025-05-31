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

from dimos.robot.unitree_webrtc.testing.helpers import color
from datetime import datetime
from dimos.robot.unitree_webrtc.type.timeseries import Timestamped, to_datetime, to_human_readable
from dimos.types.vector import Vector
from dataclasses import dataclass, field
from typing import List, TypedDict
import numpy as np
import open3d as o3d
from copy import copy


class RawLidarPoints(TypedDict):
    points: np.ndarray  # Shape (N, 3) array of 3D points [x, y, z]


class RawLidarData(TypedDict):
    """Data portion of the LIDAR message"""

    frame_id: str
    origin: List[float]
    resolution: float
    src_size: int
    stamp: float
    width: List[int]
    data: RawLidarPoints


class RawLidarMsg(TypedDict):
    """Static type definition for raw LIDAR message"""

    type: str
    topic: str
    data: RawLidarData


@dataclass
class LidarMessage(Timestamped):
    ts: datetime
    origin: Vector
    resolution: float
    pointcloud: o3d.geometry.PointCloud
    raw_msg: RawLidarMsg = field(repr=False, default=None)

    @classmethod
    def from_msg(cls, raw_message: RawLidarMsg) -> "LidarMessage":
        data = raw_message["data"]
        points = data["data"]["points"]
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        return cls(
            ts=to_datetime(data["stamp"]),
            origin=Vector(data["origin"]),
            resolution=data["resolution"],
            pointcloud=point_cloud,
            raw_msg=raw_message,
        )

    def __repr__(self):
        return f"LidarMessage(ts={to_human_readable(self.ts)}, origin={self.origin}, resolution={self.resolution}, {self.pointcloud})"

    def __iadd__(self, other: "LidarMessage") -> "LidarMessage":
        self.pointcloud += other.pointcloud
        return self

    def __add__(self, other: "LidarMessage") -> "LidarMessage":
        # Create a new point cloud combining both

        # Determine which message is more recent
        if self.timestamp >= other.timestamp:
            timestamp = self.timestamp
            origin = self.origin
            resolution = self.resolution
        else:
            timestamp = other.timestamp
            origin = other.origin
            resolution = other.resolution

        # Return a new LidarMessage with combined data
        return LidarMessage(
            timestamp=timestamp,
            origin=origin,
            resolution=resolution,
            pointcloud=self.pointcloud + other.pointcloud,
        ).estimate_normals()

    @property
    def o3d_geometry(self):
        return self.pointcloud

    def icp(self, other: "LidarMessage") -> o3d.pipelines.registration.RegistrationResult:
        self.estimate_normals()
        other.estimate_normals()

        reg_p2l = o3d.pipelines.registration.registration_icp(
            self.pointcloud,
            other.pointcloud,
            0.1,
            np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
        )

        return reg_p2l

    def transform(self, transform) -> "LidarMessage":
        self.pointcloud.transform(transform)
        return self

    def clone(self) -> "LidarMessage":
        return self.copy()

    def copy(self) -> "LidarMessage":
        return LidarMessage(
            ts=self.ts,
            origin=copy(self.origin),
            resolution=self.resolution,
            # TODO: seems to work, but will it cause issues because of the shallow copy?
            pointcloud=copy(self.pointcloud),
        )

    def icptransform(self, other):
        return self.transform(self.icp(other).transformation)

    def estimate_normals(self) -> "LidarMessage":
        # Check if normals already exist by testing if the normals attribute has data
        if not self.pointcloud.has_normals() or len(self.pointcloud.normals) == 0:
            self.pointcloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
        return self

    def color(self, color_choice) -> "LidarMessage":
        def get_color(color_choice):
            if isinstance(color_choice, int):
                return color[color_choice]
            return color_choice

        self.pointcloud.paint_uniform_color(get_color(color_choice))
        # Looks like we'll be displaying so might as well?
        self.estimate_normals()
        return self
