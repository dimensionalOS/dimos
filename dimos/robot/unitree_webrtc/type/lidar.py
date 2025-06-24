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
import struct
import json
from typing import List, TypedDict, Union, Any
from dimos.robot.unitree_webrtc.type.timeseries import Timestamped, to_datetime, to_human_readable
from dimos.types.costmap import Costmap, pointcloud_to_costmap
from dimos.types.vector import Vector
from dataclasses import dataclass, field
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
    _costmap: Costmap = field(init=False, repr=False, default=None)

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

    def costmap(self, voxel_size: float = 0.2) -> Costmap:
        if not self._costmap:
            down_sampled_pointcloud = self.pointcloud.voxel_down_sample(voxel_size=voxel_size)
            inflate_radius_m = 1.0 * voxel_size if voxel_size > self.resolution else 0.0
            grid, origin_xy = pointcloud_to_costmap(
                down_sampled_pointcloud,
                resolution=self.resolution,
                inflate_radius_m=inflate_radius_m,
            )
            self._costmap = Costmap(grid=grid, origin=[*origin_xy, 0.0], resolution=self.resolution)

        return self._costmap

    def to_zenoh_binary(self) -> bytes:
        """High-performance binary serialization for Zenoh."""
        # Ensure pointcloud data is on CPU
        points = np.asarray(self.pointcloud.points, dtype=np.float32)
        has_normals = self.pointcloud.has_normals()
        has_colors = self.pointcloud.has_colors()

        normals = (
            np.asarray(self.pointcloud.normals, dtype=np.float32) if has_normals else np.array([])
        )
        colors = (
            np.asarray(self.pointcloud.colors, dtype=np.float32) if has_colors else np.array([])
        )

        # Pack header: timestamp(8), origin_x(8), origin_y(8), origin_z(8), resolution(4),
        # num_points(4), has_normals(1), has_colors(1), reserved(2)
        timestamp_us = int(self.ts.timestamp() * 1_000_000)
        header = struct.pack(
            "!QdddfIBB2x",
            timestamp_us,
            float(self.origin.x),
            float(self.origin.y),
            float(self.origin.z),
            self.resolution,
            len(points),
            int(has_normals),
            int(has_colors),
        )

        # Serialize point data
        points_bytes = np.ascontiguousarray(points).tobytes()
        normals_bytes = np.ascontiguousarray(normals).tobytes() if has_normals else b""
        colors_bytes = np.ascontiguousarray(colors).tobytes() if has_colors else b""

        return header + points_bytes + normals_bytes + colors_bytes

    @classmethod
    def from_zenoh_binary(cls, data: Union[bytes, Any]) -> "LidarMessage":
        """Reconstruct LidarMessage from binary Zenoh data.

        Args:
            data: Binary data from Zenoh (can be bytes or ZBytes)

        Returns:
            LidarMessage instance reconstructed from binary data
        """
        # Handle ZBytes from Zenoh automatically
        if hasattr(data, "to_bytes"):
            # Zenoh ZBytes object
            data_bytes = data.to_bytes()
        elif hasattr(data, "__bytes__"):
            # Object that can be converted to bytes
            data_bytes = bytes(data)
        else:
            # Assume it's already bytes
            data_bytes = data

        # Unpack header (52 bytes total: Q(8) + ddd(24) + f(4) + I(4) + BB(2) + 2x(2) = 44 bytes)
        header_size = 44
        if len(data_bytes) < header_size:
            raise ValueError("Invalid binary data: too short for header")

        (
            timestamp_us,
            origin_x,
            origin_y,
            origin_z,
            resolution,
            num_points,
            has_normals,
            has_colors,
        ) = struct.unpack("!QdddfIBB2x", data_bytes[:header_size])

        # Reconstruct timestamp and origin
        timestamp = datetime.fromtimestamp(timestamp_us / 1_000_000)
        origin = Vector(origin_x, origin_y, origin_z)

        # Calculate data sizes
        points_size = num_points * 3 * 4  # 3 floats per point * 4 bytes per float
        normals_size = num_points * 3 * 4 if has_normals else 0
        colors_size = num_points * 3 * 4 if has_colors else 0

        # Extract data sections
        data_start = header_size
        points_data = data_bytes[data_start : data_start + points_size]

        normals_data = b""
        if has_normals:
            normals_start = data_start + points_size
            normals_data = data_bytes[normals_start : normals_start + normals_size]

        colors_data = b""
        if has_colors:
            colors_start = data_start + points_size + normals_size
            colors_data = data_bytes[colors_start : colors_start + colors_size]

        # Reconstruct pointcloud
        pointcloud = o3d.geometry.PointCloud()

        # Reconstruct points
        points = np.frombuffer(points_data, dtype=np.float32).reshape(-1, 3)
        pointcloud.points = o3d.utility.Vector3dVector(points)

        # Reconstruct normals if present
        if has_normals and len(normals_data) > 0:
            normals = np.frombuffer(normals_data, dtype=np.float32).reshape(-1, 3)
            pointcloud.normals = o3d.utility.Vector3dVector(normals)

        # Reconstruct colors if present
        if has_colors and len(colors_data) > 0:
            colors = np.frombuffer(colors_data, dtype=np.float32).reshape(-1, 3)
            pointcloud.colors = o3d.utility.Vector3dVector(colors)

        return cls(ts=timestamp, origin=origin, resolution=resolution, pointcloud=pointcloud)
