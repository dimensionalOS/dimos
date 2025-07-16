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

from __future__ import annotations

import struct
import time
from typing import Optional

import numpy as np
import open3d as o3d

# Import LCM types
from dimos_lcm.sensor_msgs.PointCloud2 import (
    PointCloud2 as LCMPointCloud2,
)
from dimos_lcm.sensor_msgs.PointField import PointField
from dimos_lcm.std_msgs.Header import Header

from dimos.types.timestamped import Timestamped


# TODO: encode/decode need to be updated to work with full spectrum of pointcloud2 fields
class PointCloud2(Timestamped):
    msg_name = "sensor_msgs.PointCloud2"

    def __init__(
        self,
        pointcloud: o3d.geometry.PointCloud = None,
        frame_id: str = "",
        ts: Optional[float] = None,
    ):
        self.ts = ts if ts is not None else time.time()
        self.pointcloud = pointcloud if pointcloud is not None else o3d.geometry.PointCloud()
        self.frame_id = frame_id

    # TODO what's the usual storage here? is it already numpy?
    def as_numpy(self) -> np.ndarray:
        """Get points as numpy array."""
        return np.asarray(self.pointcloud.points)

    def lcm_encode(self, frame_id: Optional[str] = None) -> bytes:
        """Convert to LCM PointCloud2 message."""
        msg = LCMPointCloud2()

        # Header
        msg.header = Header()
        msg.header.seq = 0  # Initialize sequence number
        msg.header.frame_id = frame_id or self.frame_id

        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)

        points = self.as_numpy()
        if len(points) == 0:
            # Empty point cloud
            msg.height = 0
            msg.width = 0
            msg.point_step = 16  # 4 floats * 4 bytes (x, y, z, intensity)
            msg.row_step = 0
            msg.data_length = 0
            msg.data = b""
            msg.is_dense = True
            msg.is_bigendian = False
            msg.fields_length = 4  # x, y, z, intensity
            msg.fields = self._create_xyz_field()
            return msg.encode()

        # Point cloud dimensions
        msg.height = 1  # Unorganized point cloud
        msg.width = len(points)

        # Define fields (X, Y, Z, intensity as float32)
        msg.fields_length = 4  # x, y, z, intensity
        msg.fields = self._create_xyz_field()

        # Point step and row step
        msg.point_step = 16  # 4 floats * 4 bytes each (x, y, z, intensity)
        msg.row_step = msg.point_step * msg.width

        # Convert points to bytes with intensity padding (little endian float32)
        # Add intensity column (zeros) to make it 4 columns: x, y, z, intensity
        points_with_intensity = np.column_stack(
            [
                points,  # x, y, z columns
                np.zeros(len(points), dtype=np.float32),  # intensity column (padding)
            ]
        )
        data_bytes = points_with_intensity.astype(np.float32).tobytes()
        msg.data_length = len(data_bytes)
        msg.data = data_bytes

        # Properties
        msg.is_dense = True  # No invalid points
        msg.is_bigendian = False  # Little endian

        return msg.encode()

    @classmethod
    def lcm_decode(cls, data: bytes) -> "PointCloud2":
        msg = LCMPointCloud2.decode(data)

        if msg.width == 0 or msg.height == 0:
            # Empty point cloud
            pc = o3d.geometry.PointCloud()
            return cls(
                pointcloud=pc,
                frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
                ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header") and msg.header.stamp.sec > 0
                else None,
            )

        # Parse field information to find X, Y, Z offsets
        x_offset = y_offset = z_offset = None
        for msgfield in msg.fields:
            if msgfield.name == "x":
                x_offset = msgfield.offset
            elif msgfield.name == "y":
                y_offset = msgfield.offset
            elif msgfield.name == "z":
                z_offset = msgfield.offset

        if any(offset is None for offset in [x_offset, y_offset, z_offset]):
            raise ValueError("PointCloud2 message missing X, Y, or Z msgfields")

        # Extract points from binary data
        num_points = msg.width * msg.height
        points = np.zeros((num_points, 3), dtype=np.float32)

        data = msg.data
        point_step = msg.point_step

        for i in range(num_points):
            base_offset = i * point_step

            # Extract X, Y, Z (assuming float32, little endian)
            x_bytes = data[base_offset + x_offset : base_offset + x_offset + 4]
            y_bytes = data[base_offset + y_offset : base_offset + y_offset + 4]
            z_bytes = data[base_offset + z_offset : base_offset + z_offset + 4]

            points[i, 0] = struct.unpack("<f", x_bytes)[0]
            points[i, 1] = struct.unpack("<f", y_bytes)[0]
            points[i, 2] = struct.unpack("<f", z_bytes)[0]

        # Create Open3D point cloud
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        return cls(
            pointcloud=pc,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else None,
        )

    def _create_xyz_field(self) -> list:
        """Create standard X, Y, Z field definitions for LCM PointCloud2."""
        fields = []

        # X field
        x_field = PointField()
        x_field.name = "x"
        x_field.offset = 0
        x_field.datatype = 7  # FLOAT32
        x_field.count = 1
        fields.append(x_field)

        # Y field
        y_field = PointField()
        y_field.name = "y"
        y_field.offset = 4
        y_field.datatype = 7  # FLOAT32
        y_field.count = 1
        fields.append(y_field)

        # Z field
        z_field = PointField()
        z_field.name = "z"
        z_field.offset = 8
        z_field.datatype = 7  # FLOAT32
        z_field.count = 1
        fields.append(z_field)

        # I field
        i_field = PointField()
        i_field.name = "intensity"
        i_field.offset = 12
        i_field.datatype = 7  # FLOAT32
        i_field.count = 1
        fields.append(i_field)

        return fields

    def __len__(self) -> int:
        """Return number of points."""
        return len(self.pointcloud.points)

    def __repr__(self) -> str:
        """String representation."""
        return f"PointCloud(points={len(self)}, frame_id='{self.frame_id}', ts={self.ts})"
