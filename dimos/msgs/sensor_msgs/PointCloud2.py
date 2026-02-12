# Copyright 2025-2026 Dimensional Inc.
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
from typing import TYPE_CHECKING, Any

# Import LCM types
from dimos_lcm.sensor_msgs.PointCloud2 import (
    PointCloud2 as LCMPointCloud2,
)
from dimos_lcm.sensor_msgs.PointField import PointField  # type: ignore[import-untyped]
from dimos_lcm.std_msgs.Header import Header  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Transform, Vector3
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image


@functools.lru_cache(maxsize=16)
def _get_matplotlib_cmap(name: str):  # type: ignore[no-untyped-def]
    """Get a matplotlib colormap by name (cached for performance)."""
    import matplotlib.pyplot as plt

    return plt.get_cmap(name)


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
        )
        self.__dict__.update(state)
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

    @classmethod
    def from_rgbd(
        cls,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        depth_scale: float = 1.0,
        depth_trunc: float = 5.0,
    ) -> PointCloud2:
        """Create PointCloud2 from RGB and depth Image messages.

        Uses frame_id and timestamp from the depth image.

        Args:
            color_image: RGB/BGR color Image message
            depth_image: Depth Image message (float32 meters or uint16 mm)
            camera_info: CameraInfo message with intrinsics
            depth_scale: Scale factor to convert depth to meters (default 1.0 for float32)
            depth_trunc: Maximum depth in meters to include

        Returns:
            PointCloud2 instance with colored points
        """
        # Get color as RGB numpy array
        color_data = color_image.to_rgb().data
        if hasattr(color_data, "get"):  # CuPy array
            color_data = color_data.get()
        color_data = np.ascontiguousarray(color_data)

        # Get depth numpy array
        depth_data = depth_image.data
        if hasattr(depth_data, "get"):  # CuPy array
            depth_data = depth_data.get()

        # Convert depth to float32 meters if needed
        if depth_data.dtype == np.uint16:
            depth_data = depth_data.astype(np.float32) * depth_scale
        elif depth_data.dtype != np.float32:
            depth_data = depth_data.astype(np.float32)
        depth_data = np.ascontiguousarray(depth_data)

        # Verify dimensions match
        color_h, color_w = color_data.shape[:2]
        depth_h, depth_w = depth_data.shape[:2]
        if (color_h, color_w) != (depth_h, depth_w):
            raise ValueError(
                f"Color {color_w}x{color_h} and depth {depth_w}x{depth_h} dimensions don't match"
            )

        # Get intrinsics from camera_info
        intrinsic = camera_info.get_K_matrix()
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]

        # Verify intrinsics match image dimensions
        if camera_info.width != color_w or camera_info.height != color_h:
            # Scale intrinsics if resolution differs
            scale_x = color_w / camera_info.width
            scale_y = color_h / camera_info.height
            fx *= scale_x
            fy *= scale_y
            cx *= scale_x
            cy *= scale_y

        # Create Open3D images
        color_o3d = o3d.geometry.Image(color_data.astype(np.uint8))

        # Filter invalid depth values
        depth_filtered = depth_data.copy()
        valid_mask = np.isfinite(depth_filtered) & (depth_filtered > 0)
        depth_filtered[~valid_mask] = 0.0
        depth_o3d = o3d.geometry.Image(depth_filtered.astype(np.float32))

        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=color_w,
            height=color_h,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )

        # Create RGBD image and point cloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1.0,  # Already scaled
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)

        return cls(
            pointcloud=pcd,
            frame_id=depth_image.frame_id,
            ts=depth_image.ts,
        )

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

    def transform(self, tf: Transform) -> PointCloud2:
        """Transform the pointcloud using a Transform object.

        Applies the rotation and translation from the transform to all points,
        converting them into the transform's frame_id.

        Args:
            tf: Transform object containing rotation and translation

        Returns:
            New PointCloud2 instance with transformed points in the new frame
        """
        points, _ = self.as_numpy()

        if len(points) == 0:
            return PointCloud2(
                pointcloud=o3d.geometry.PointCloud(),
                frame_id=tf.frame_id,
                ts=self.ts,
            )

        # Build 4x4 transformation matrix from Transform
        transform_matrix = tf.to_matrix()

        # Convert points to homogeneous coordinates (N, 4)
        ones = np.ones((len(points), 1))
        points_homogeneous = np.hstack([points, ones])

        # Apply transformation: (4, 4) @ (4, N) -> (4, N) -> transpose to (N, 4)
        transformed_points = (transform_matrix @ points_homogeneous.T).T

        # Extract xyz coordinates (drop homogeneous coordinate)
        transformed_xyz = transformed_points[:, :3].astype(np.float64)

        # Create new Open3D point cloud
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(transformed_xyz)

        # Copy colors if available
        if self.pointcloud.has_colors():
            new_pcd.colors = self.pointcloud.colors

        return PointCloud2(
            pointcloud=new_pcd,
            frame_id=tf.frame_id,
            ts=self.ts,
        )

    def voxel_downsample(self, voxel_size: float = 0.025) -> PointCloud2:
        """Downsample the pointcloud with a voxel grid."""
        if voxel_size <= 0:
            return self
        if len(self.pointcloud.points) < 20:
            return self
        downsampled = self._pcd_tensor.voxel_down_sample(voxel_size)
        return PointCloud2(pointcloud=downsampled, frame_id=self.frame_id, ts=self.ts)

    def as_numpy(
        self,
    ) -> tuple[np.ndarray[Any, Any], np.ndarray[Any, Any] | None]:
        """Get points and colors as numpy arrays.

        Returns:
            Tuple of (points, colors) where:
            - points: Nx3 numpy array of 3D points
            - colors: Nx3 array in [0, 1] range, or None if no colors
        """
        points = np.asarray(self.pointcloud.points)
        colors = np.asarray(self.pointcloud.colors) if self.pointcloud.has_colors() else None
        return points, colors

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
        """Convert to LCM PointCloud2 message with optional RGB colors."""
        msg = LCMPointCloud2()

        # Header
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id

        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)

        points, _ = self.as_numpy()

        # Check if pointcloud has colors
        self._ensure_tensor_initialized()
        has_colors = "colors" in self._pcd_tensor.point

        if len(points) == 0:
            msg.height = 0
            msg.width = 0
            msg.point_step = 16
            msg.row_step = 0
            msg.data_length = 0
            msg.data = b""
            msg.is_dense = True
            msg.is_bigendian = False
            msg.fields_length = 4
            msg.fields = self._create_xyzrgb_fields() if has_colors else self._create_xyz_fields()
            return msg.lcm_encode()  # type: ignore[no-any-return]

        msg.height = 1
        msg.width = len(points)

        if has_colors:
            # Get colors (0-1 range) and convert to uint8
            colors = self._pcd_tensor.point["colors"].numpy()
            if colors.max() <= 1.0:
                colors = (colors * 255).astype(np.uint8)
            else:
                colors = colors.astype(np.uint8)

            # Pack RGB into float32 (ROS convention: bytes are [padding, r, g, b])
            rgb_packed = np.zeros(len(points), dtype=np.float32)
            rgb_uint32 = (
                (colors[:, 0].astype(np.uint32) << 16)
                | (colors[:, 1].astype(np.uint32) << 8)
                | colors[:, 2].astype(np.uint32)
            )
            rgb_packed = rgb_uint32.view(np.float32)

            msg.fields = self._create_xyzrgb_fields()
            msg.fields_length = 4
            msg.point_step = 16  # x, y, z, rgb (4 floats)

            point_data = np.column_stack([points, rgb_packed]).astype(np.float32)
        else:
            msg.fields = self._create_xyz_fields()
            msg.fields_length = 4
            msg.point_step = 16  # x, y, z, intensity

            point_data = np.column_stack(
                [
                    points,
                    np.zeros(len(points), dtype=np.float32),
                ]
            ).astype(np.float32)

        msg.row_step = msg.point_step * msg.width
        data_bytes = point_data.tobytes()
        msg.data_length = len(data_bytes)
        msg.data = data_bytes

        msg.is_dense = True
        msg.is_bigendian = False

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> PointCloud2:
        msg = LCMPointCloud2.lcm_decode(data)

        if msg.width == 0 or msg.height == 0:
            pc = o3d.geometry.PointCloud()
            return cls(
                pointcloud=pc,
                frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
                ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header") and msg.header.stamp.sec > 0
                else None,
            )

        # Parse field offsets
        x_offset = y_offset = z_offset = rgb_offset = None
        for msgfield in msg.fields:
            if msgfield.name == "x":
                x_offset = msgfield.offset
            elif msgfield.name == "y":
                y_offset = msgfield.offset
            elif msgfield.name == "z":
                z_offset = msgfield.offset
            elif msgfield.name == "rgb":
                rgb_offset = msgfield.offset

        if any(offset is None for offset in [x_offset, y_offset, z_offset]):
            raise ValueError("PointCloud2 message missing X, Y, or Z msgfields")

        num_points = msg.width * msg.height
        raw_data = msg.data
        point_step = msg.point_step

        # Fast path for standard layout
        if x_offset == 0 and y_offset == 4 and z_offset == 8 and point_step >= 12:
            if point_step == 12:
                points = np.frombuffer(raw_data, dtype=np.float32).reshape(-1, 3)
            else:
                dt = np.dtype(
                    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{point_step - 12}")]
                )
                structured = np.frombuffer(raw_data, dtype=dt, count=num_points)
                points = np.column_stack((structured["x"], structured["y"], structured["z"]))
        else:
            points = np.zeros((num_points, 3), dtype=np.float32)
            for i in range(num_points):
                base_offset = i * point_step
                points[i, 0] = struct.unpack(
                    "<f", raw_data[base_offset + x_offset : base_offset + x_offset + 4]
                )[0]
                points[i, 1] = struct.unpack(
                    "<f", raw_data[base_offset + y_offset : base_offset + y_offset + 4]
                )[0]
                points[i, 2] = struct.unpack(
                    "<f", raw_data[base_offset + z_offset : base_offset + z_offset + 4]
                )[0]

        # Create tensor pointcloud
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

        # Extract RGB colors if present
        if rgb_offset is not None:
            dt = np.dtype(
                [
                    ("_pre", f"V{rgb_offset}"),
                    ("rgb", "<f4"),
                    ("_post", f"V{point_step - rgb_offset - 4}"),
                ]
            )
            structured = np.frombuffer(raw_data, dtype=dt, count=num_points)
            rgb_packed = structured["rgb"].view(np.uint32)
            r = ((rgb_packed >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((rgb_packed >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (rgb_packed & 0xFF).astype(np.float32) / 255.0
            colors = np.column_stack([r, g, b])
            pcd_t.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)

        return cls(
            pointcloud=pcd_t,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else None,
        )

    def _create_xyz_fields(self) -> list:  # type: ignore[type-arg]
        """Create X, Y, Z, intensity field definitions."""
        fields = []
        for i, name in enumerate(["x", "y", "z", "intensity"]):
            field = PointField()
            field.name = name
            field.offset = i * 4
            field.datatype = 7  # FLOAT32
            field.count = 1
            fields.append(field)
        return fields

    def _create_xyzrgb_fields(self) -> list:  # type: ignore[type-arg]
        """Create X, Y, Z, RGB field definitions for colored pointclouds."""
        fields = []
        for i, name in enumerate(["x", "y", "z"]):
            field = PointField()
            field.name = name
            field.offset = i * 4
            field.datatype = 7  # FLOAT32
            field.count = 1
            fields.append(field)

        # RGB field (packed as float32, ROS convention)
        rgb_field = PointField()
        rgb_field.name = "rgb"
        rgb_field.offset = 12
        rgb_field.datatype = 7  # FLOAT32 (contains packed RGB)
        rgb_field.count = 1
        fields.append(rgb_field)

        return fields

    def __len__(self) -> int:
        """Return number of points."""
        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return 0
        return int(self._pcd_tensor.point["positions"].shape[0])

    def to_rerun(
        self,
        voxel_size: float = 0.05,
        colormap: str | None = "turbo",
        colors: list[int] | None = None,
        mode: str = "boxes",
        size: float | None = None,
        fill_mode: str = "solid",
        **kwargs: object,
    ) -> Archetype:
        import rerun as rr

        """Convert to Rerun Points3D or Boxes3D archetype.

        Args:
            voxel_size: size for visualization
            colormap: Optional colormap name (e.g., "turbo", "viridis") to color by height
            colors: Optional RGB color [r, g, b] for all points (0-255)
            mode: Visualization mode - "points" for spheres, "boxes" for cubes (default)
            size: Box size for mode="boxes" (e.g., voxel_size). Defaults to radii*2.
            fill_mode: Fill mode for boxes - "solid", "majorwireframe", or "densewireframe"
            **kwargs: Additional args (ignored for compatibility)

        Returns:
            rr.Points3D or rr.Boxes3D archetype for logging to Rerun
        """
        points, _ = self.as_numpy()
        if len(points) == 0:
            return rr.Points3D([]) if mode == "points" else rr.Boxes3D(centers=[])

        # Determine colors
        point_colors = None
        if colormap is not None:
            # Color by height (z-coordinate)
            z = points[:, 2]
            z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
            cmap = _get_matplotlib_cmap(colormap)
            point_colors = (cmap(z_norm)[:, :3] * 255).astype(np.uint8)
        elif colors is not None:
            point_colors = colors

        if mode == "boxes":
            # Use boxes for voxel visualization
            box_size = size if size is not None else voxel_size
            half = box_size / 2
            return rr.Boxes3D(
                centers=points,
                half_sizes=[half, half, half],
                colors=point_colors,
                fill_mode=fill_mode,  # type: ignore[arg-type]
            )
        else:
            return rr.Points3D(
                positions=points,
                radii=voxel_size / 2,
                colors=point_colors,
            )

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
        points, _ = self.as_numpy()

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
