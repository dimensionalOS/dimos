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

import functools
import struct

# Import LCM types
from dimos_lcm.sensor_msgs.PointCloud2 import (  # type: ignore[import-untyped]
    PointCloud2 as LCMPointCloud2,
)
from dimos_lcm.sensor_msgs.PointField import PointField  # type: ignore[import-untyped]
from dimos_lcm.std_msgs.Header import Header  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Vector3

# Import ROS types
try:
    from sensor_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        PointCloud2 as ROSPointCloud2,
        PointField as ROSPointField,
    )
    from std_msgs.msg import Header as ROSHeader  # type: ignore[attr-defined, import-untyped]

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from dimos.types.timestamped import Timestamped


# TODO: encode/decode need to be updated to work with full spectrum of pointcloud2 fields
class PointCloud2(Timestamped):
    msg_name = "sensor_msgs.PointCloud2"

    def __init__(
        self,
        pointcloud: o3d.geometry.PointCloud | o3d.t.geometry.PointCloud | None = None,
        frame_id: str = "world",
        ts: float | None = None,
    ) -> None:
        self.ts = ts  # type: ignore[assignment]
        self.frame_id = frame_id

        # Store internally as tensor pointcloud for speed
        if pointcloud is None:
            self._pcd_tensor: o3d.t.geometry.PointCloud = o3d.t.geometry.PointCloud()
        elif isinstance(pointcloud, o3d.t.geometry.PointCloud):
            self._pcd_tensor = pointcloud
        else:
            # Convert legacy to tensor
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(pointcloud)
        self._pcd_legacy_cache: o3d.geometry.PointCloud | None = None

    def _ensure_tensor_initialized(self) -> None:
        """Ensure _pcd_tensor and _pcd_legacy_cache exist (handles unpickled old objects)."""
        # Always ensure _pcd_legacy_cache exists
        if not hasattr(self, "_pcd_legacy_cache"):
            self._pcd_legacy_cache = None

        # Check for old pickled format: 'pointcloud' directly in __dict__
        # This takes priority even if _pcd_tensor exists (it might be empty)
        old_pcd = self.__dict__.get("pointcloud")
        if old_pcd is not None and isinstance(old_pcd, o3d.geometry.PointCloud):
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(old_pcd)
            self._pcd_legacy_cache = old_pcd  # reuse it
            del self.__dict__["pointcloud"]
            return

        if not hasattr(self, "_pcd_tensor"):
            self._pcd_tensor = o3d.t.geometry.PointCloud()

    def __getstate__(self) -> dict[str, object]:
        """Serialize to numpy for pickling (tensors don't pickle well)."""
        self._ensure_tensor_initialized()
        state = self.__dict__.copy()
        # Convert tensor to numpy for serialization
        if "positions" in self._pcd_tensor.point:
            state["_pcd_numpy"] = self._pcd_tensor.point["positions"].numpy()
        else:
            state["_pcd_numpy"] = np.zeros((0, 3), dtype=np.float32)
        # Remove non-picklable objects
        del state["_pcd_tensor"]
        state["_pcd_legacy_cache"] = None
        return state

    def __setstate__(self, state: dict[str, object]) -> None:
        """Restore from pickled state."""
        points_obj = state.pop("_pcd_numpy", None)
        points: np.ndarray[tuple[int, int], np.dtype[np.float32]] = (
            points_obj if isinstance(points_obj, np.ndarray) else np.zeros((0, 3), dtype=np.float32)
        )  # type: ignore[assignment]
        self.__dict__.update(state)  # type: ignore[arg-type]
        # Recreate tensor from numpy
        self._pcd_tensor = o3d.t.geometry.PointCloud()
        if len(points) > 0:
            self._pcd_tensor.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

    @property
    def pointcloud(self) -> o3d.geometry.PointCloud:
        """Legacy pointcloud property for backwards compatibility. Cached."""
        self._ensure_tensor_initialized()
        if self._pcd_legacy_cache is None:
            self._pcd_legacy_cache = self._pcd_tensor.to_legacy()
        return self._pcd_legacy_cache

    @pointcloud.setter
    def pointcloud(self, value: o3d.geometry.PointCloud | o3d.t.geometry.PointCloud) -> None:
        if isinstance(value, o3d.t.geometry.PointCloud):
            self._pcd_tensor = value
        else:
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(value)
        self._pcd_legacy_cache = None

    @property
    def pointcloud_tensor(self) -> o3d.t.geometry.PointCloud:
        """Direct access to tensor pointcloud (faster, no conversion)."""
        self._ensure_tensor_initialized()
        return self._pcd_tensor

    @classmethod
    def from_numpy(
        cls,
        points: np.ndarray,  # type: ignore[type-arg]
        frame_id: str = "world",
        timestamp: float | None = None,
    ) -> PointCloud2:
        """Create PointCloud2 from numpy array of shape (N, 3).

        Args:
            points: Nx3 numpy array of 3D points
            frame_id: Frame ID for the point cloud
            timestamp: Timestamp for the point cloud (defaults to current time)

        Returns:
            PointCloud2 instance
        """
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32)
        return cls(pointcloud=pcd_t, ts=timestamp, frame_id=frame_id)

    def __str__(self) -> str:
        return f"PointCloud2(frame_id='{self.frame_id}', num_points={len(self)})"

    @functools.cached_property
    def center(self) -> Vector3:
        """Calculate the center of the pointcloud in world frame."""
        center = np.asarray(self.pointcloud.points).mean(axis=0)
        return Vector3(*center)

    def points(self):  # type: ignore[no-untyped-def]
        """Get points (returns tensor positions, use as_numpy() for numpy array)."""
        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return o3c.Tensor(np.zeros((0, 3), dtype=np.float32))
        return self._pcd_tensor.point["positions"]

    def __add__(self, other: PointCloud2) -> PointCloud2:
        """Combine two PointCloud2 instances into one.

        The resulting point cloud contains points from both inputs.
        The frame_id and timestamp are taken from the first point cloud.

        Args:
            other: Another PointCloud2 instance to combine with

        Returns:
            New PointCloud2 instance containing combined points
        """
        if not isinstance(other, PointCloud2):
            raise ValueError("Can only add PointCloud2 to another PointCloud2")

        return PointCloud2(
            pointcloud=self.pointcloud + other.pointcloud,
            frame_id=self.frame_id,
            ts=max(self.ts, other.ts),
        )

    def as_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get points as numpy array (fast, no legacy conversion)."""
        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return np.zeros((0, 3), dtype=np.float32)
        result: np.ndarray = self._pcd_tensor.point["positions"].numpy()  # type: ignore[type-arg]
        return result

    @functools.cache
    def get_axis_aligned_bounding_box(self) -> o3d.geometry.AxisAlignedBoundingBox:
        """Get axis-aligned bounding box of the point cloud."""
        return self.pointcloud.get_axis_aligned_bounding_box()

    @functools.cache
    def get_oriented_bounding_box(self) -> o3d.geometry.OrientedBoundingBox:
        """Get oriented bounding box of the point cloud."""
        return self.pointcloud.get_oriented_bounding_box()

    @functools.cache
    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of axis-aligned bounding box."""
        bbox = self.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        return tuple(extent)

    def bounding_box_intersects(self, other: PointCloud2) -> bool:
        # Get axis-aligned bounding boxes
        bbox1 = self.get_axis_aligned_bounding_box()
        bbox2 = other.get_axis_aligned_bounding_box()

        # Get min and max bounds
        min1 = bbox1.get_min_bound()
        max1 = bbox1.get_max_bound()
        min2 = bbox2.get_min_bound()
        max2 = bbox2.get_max_bound()

        # Check overlap in all three dimensions
        # Boxes intersect if they overlap in ALL dimensions
        return (  # type: ignore[no-any-return]
            min1[0] <= max2[0]
            and max1[0] >= min2[0]
            and min1[1] <= max2[1]
            and max1[1] >= min2[1]
            and min1[2] <= max2[2]
            and max1[2] >= min2[2]
        )

    def lcm_encode(self, frame_id: str | None = None) -> bytes:
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
            return msg.lcm_encode()  # type: ignore[no-any-return]

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

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> PointCloud2:
        msg = LCMPointCloud2.lcm_decode(data)

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

        # Extract points from binary data using numpy for bulk conversion
        num_points = msg.width * msg.height
        data = msg.data
        point_step = msg.point_step

        # Check if we can use fast numpy path (common case: sequential float32 x,y,z)
        if x_offset == 0 and y_offset == 4 and z_offset == 8 and point_step >= 12:
            # Fast path: direct numpy conversion for tightly packed float32 x,y,z
            if point_step == 12:
                # Perfectly packed x,y,z with no padding
                points = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
            else:
                # Has additional fields after x,y,z (e.g., intensity), extract with stride
                dt = np.dtype(
                    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{point_step - 12}")]
                )
                structured = np.frombuffer(data, dtype=dt, count=num_points)
                points = np.column_stack((structured["x"], structured["y"], structured["z"]))
        else:
            # Slow fallback for non-standard field layouts
            points = np.zeros((num_points, 3), dtype=np.float32)
            for i in range(num_points):
                base_offset = i * point_step
                x_bytes = data[base_offset + x_offset : base_offset + x_offset + 4]
                y_bytes = data[base_offset + y_offset : base_offset + y_offset + 4]
                z_bytes = data[base_offset + z_offset : base_offset + z_offset + 4]
                points[i, 0] = struct.unpack("<f", x_bytes)[0]
                points[i, 1] = struct.unpack("<f", y_bytes)[0]
                points[i, 2] = struct.unpack("<f", z_bytes)[0]

        # Create Open3D tensor point cloud (fast, no Vector3dVector)
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

        return cls(
            pointcloud=pcd_t,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else None,
        )

    def _create_xyz_field(self) -> list:  # type: ignore[type-arg]
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
        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return 0
        return int(self._pcd_tensor.point["positions"].shape[0])

    def filter_by_height(
        self,
        min_height: float | None = None,
        max_height: float | None = None,
    ) -> PointCloud2:
        """Filter points based on their height (z-coordinate).

        This method creates a new PointCloud2 containing only points within the specified
        height range. All metadata (frame_id, timestamp) is preserved.

        Args:
            min_height: Optional minimum height threshold. Points with z < min_height are filtered out.
                       If None, no lower limit is applied.
            max_height: Optional maximum height threshold. Points with z > max_height are filtered out.
                       If None, no upper limit is applied.

        Returns:
            New PointCloud2 instance containing only the filtered points.

        Raises:
            ValueError: If both min_height and max_height are None (no filtering would occur).

        Example:
            # Remove ground points below 0.1m height
            filtered_pc = pointcloud.filter_by_height(min_height=0.1)

            # Keep only points between ground level and 2m height
            filtered_pc = pointcloud.filter_by_height(min_height=0.0, max_height=2.0)

            # Remove points above 1.5m (e.g., ceiling)
            filtered_pc = pointcloud.filter_by_height(max_height=1.5)
        """
        # Validate that at least one threshold is provided
        if min_height is None and max_height is None:
            raise ValueError("At least one of min_height or max_height must be specified")

        # Get points as numpy array
        points = self.as_numpy()

        if len(points) == 0:
            # Empty pointcloud - return a copy
            return PointCloud2(
                pointcloud=o3d.geometry.PointCloud(),
                frame_id=self.frame_id,
                ts=self.ts,
            )

        # Extract z-coordinates (height values) - column index 2
        heights = points[:, 2]

        # Create boolean mask for filtering based on height thresholds
        # Start with all True values
        mask = np.ones(len(points), dtype=bool)

        # Apply minimum height filter if specified
        if min_height is not None:
            mask &= heights >= min_height

        # Apply maximum height filter if specified
        if max_height is not None:
            mask &= heights <= max_height

        # Apply mask to filter points
        filtered_points = points[mask]

        # Create new PointCloud2 with filtered points
        return PointCloud2.from_numpy(
            points=filtered_points,
            frame_id=self.frame_id,
            timestamp=self.ts,
        )

    def __repr__(self) -> str:
        """String representation."""
        return f"PointCloud(points={len(self)}, frame_id='{self.frame_id}', ts={self.ts})"

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSPointCloud2) -> PointCloud2:
        """Convert from ROS sensor_msgs/PointCloud2 message.

        Args:
            ros_msg: ROS PointCloud2 message

        Returns:
            PointCloud2 instance
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS packages not available. Cannot convert from ROS message.")

        # Handle empty point cloud
        if ros_msg.width == 0 or ros_msg.height == 0:
            pc = o3d.geometry.PointCloud()
            return cls(
                pointcloud=pc,
                frame_id=ros_msg.header.frame_id,
                ts=ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec / 1e9,
            )

        # Parse field information to find X, Y, Z offsets
        x_offset = y_offset = z_offset = None
        for field in ros_msg.fields:
            if field.name == "x":
                x_offset = field.offset
            elif field.name == "y":
                y_offset = field.offset
            elif field.name == "z":
                z_offset = field.offset

        if any(offset is None for offset in [x_offset, y_offset, z_offset]):
            raise ValueError("PointCloud2 message missing X, Y, or Z fields")

        # Extract points from binary data using numpy for bulk conversion
        num_points = ros_msg.width * ros_msg.height
        data = ros_msg.data
        point_step = ros_msg.point_step

        # Determine byte order
        byte_order = ">" if ros_msg.is_bigendian else "<"

        # Check if we can use fast numpy path (common case: sequential float32 x,y,z)
        if (
            x_offset == 0
            and y_offset == 4
            and z_offset == 8
            and point_step >= 12
            and not ros_msg.is_bigendian
        ):
            # Fast path: direct numpy reshape for tightly packed float32 x,y,z
            # This is the most common case for point clouds
            if point_step == 12:
                # Perfectly packed x,y,z with no padding
                points = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
            else:
                # Has additional fields after x,y,z, need to extract with stride
                dt = np.dtype(
                    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{point_step - 12}")]
                )
                structured = np.frombuffer(data, dtype=dt, count=num_points)
                points = np.column_stack((structured["x"], structured["y"], structured["z"]))
        else:
            # General case: handle arbitrary field offsets and byte order
            # Create structured dtype for the entire point
            dt_fields = []

            # Add padding before x if needed
            if x_offset > 0:  # type: ignore[operator]
                dt_fields.append(("_pad_x", f"V{x_offset}"))
            dt_fields.append(("x", f"{byte_order}f4"))

            # Add padding between x and y if needed
            gap_xy = y_offset - x_offset - 4  # type: ignore[operator]
            if gap_xy > 0:
                dt_fields.append(("_pad_xy", f"V{gap_xy}"))
            dt_fields.append(("y", f"{byte_order}f4"))

            # Add padding between y and z if needed
            gap_yz = z_offset - y_offset - 4  # type: ignore[operator]
            if gap_yz > 0:
                dt_fields.append(("_pad_yz", f"V{gap_yz}"))
            dt_fields.append(("z", f"{byte_order}f4"))

            # Add padding at the end to match point_step
            remaining = point_step - z_offset - 4
            if remaining > 0:
                dt_fields.append(("_pad_end", f"V{remaining}"))

            dt = np.dtype(dt_fields)
            structured = np.frombuffer(data, dtype=dt, count=num_points)
            points = np.column_stack((structured["x"], structured["y"], structured["z"]))

        # Filter out NaN and Inf values if not dense
        if not ros_msg.is_dense:
            mask = np.isfinite(points).all(axis=1)
            points = points[mask]

        # Create Open3D point cloud
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        # Extract timestamp
        ts = ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec / 1e9

        return cls(
            pointcloud=pc,
            frame_id=ros_msg.header.frame_id,
            ts=ts,
        )

    def to_ros_msg(self) -> ROSPointCloud2:
        """Convert to ROS sensor_msgs/PointCloud2 message.

        Returns:
            ROS PointCloud2 message
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS packages not available. Cannot convert to ROS message.")

        ros_msg = ROSPointCloud2()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header = ROSHeader()  # type: ignore[no-untyped-call]
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1e9)

        points = self.as_numpy()

        if len(points) == 0:
            # Empty point cloud
            ros_msg.height = 0
            ros_msg.width = 0
            ros_msg.fields = []
            ros_msg.is_bigendian = False
            ros_msg.point_step = 0
            ros_msg.row_step = 0
            ros_msg.data = b""
            ros_msg.is_dense = True
            return ros_msg

        # Set dimensions
        ros_msg.height = 1  # Unorganized point cloud
        ros_msg.width = len(points)

        # Define fields (X, Y, Z as float32)
        ros_msg.fields = [
            ROSPointField(name="x", offset=0, datatype=ROSPointField.FLOAT32, count=1),  # type: ignore[no-untyped-call]
            ROSPointField(name="y", offset=4, datatype=ROSPointField.FLOAT32, count=1),  # type: ignore[no-untyped-call]
            ROSPointField(name="z", offset=8, datatype=ROSPointField.FLOAT32, count=1),  # type: ignore[no-untyped-call]
        ]

        # Set point step and row step
        ros_msg.point_step = 12  # 3 floats * 4 bytes each
        ros_msg.row_step = ros_msg.point_step * ros_msg.width

        # Convert points to bytes (little endian float32)
        ros_msg.data = points.astype(np.float32).tobytes()

        # Set properties
        ros_msg.is_bigendian = False  # Little endian
        ros_msg.is_dense = True  # No invalid points

        return ros_msg
