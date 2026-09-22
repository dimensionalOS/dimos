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

from dataclasses import dataclass, field
import functools
from typing import TYPE_CHECKING, Any

import cv2
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
import open3d as o3d

from dimos.msgs.geometry import inverse_transform, transform_matrix
from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz, select_points
from dimos.perception.detection.type.detection3d.base import Detection3D
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    PointCloudFilter,
    radius_outlier,
    raycast,
    statistical,
)

if TYPE_CHECKING:
    from dimos_generated.sensor_msgs.msg import CameraInfo, Image

    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2 = field(default_factory=PointCloud2)

    @functools.cached_property
    def center(self) -> Vector3:
        center = pointcloud_xyz(self.pointcloud).mean(axis=0)
        return Vector3(x=float(center[0]), y=float(center[1]), z=float(center[2]))

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using pointcloud center.

        Returns pose in world frame with identity rotation.
        The pointcloud is already in world frame.
        """
        return PoseStamped(
            header=self.pointcloud.header,
            pose=Pose(
                position=Point(x=self.center.x, y=self.center.y, z=self.center.z),
                orientation=Quaternion(w=1),
            ),
        )

    def _open3d(self) -> o3d.geometry.PointCloud:
        return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pointcloud_xyz(self.pointcloud)))

    def get_bounding_box(self) -> o3d.geometry.AxisAlignedBoundingBox:
        """Get axis-aligned bounding box of the detection's pointcloud."""
        return self._open3d().get_axis_aligned_bounding_box()

    def get_oriented_bounding_box(self) -> o3d.geometry.OrientedBoundingBox:
        """Get oriented bounding box of the detection's pointcloud."""
        return self._open3d().get_oriented_bounding_box()

    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of the detection's bounding box."""
        extent = self.get_bounding_box().get_extent()
        return float(extent[0]), float(extent[1]), float(extent[2])

    def bounding_box_intersects(self, other: Detection3DPC) -> bool:
        """Check if this detection's bounding box intersects with another's."""
        first, second = self.get_bounding_box(), other.get_bounding_box()
        return bool(
            np.all(first.get_min_bound() <= second.get_max_bound())
            and np.all(second.get_min_bound() <= first.get_max_bound())
        )

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        # The pointcloud is in world frame, and transform gives camera position in world
        center_world = self.center
        # Camera position in world frame is the translation part of the transform
        camera_pos = inverse_transform(self.transform).transform.translation
        # Use Vector3 subtraction and magnitude
        distance = np.linalg.norm(
            [
                center_world.x - camera_pos.x,
                center_world.y - camera_pos.y,
                center_world.z - camera_pos.z,
            ]
        )

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "points": str(self.pointcloud.width * self.pointcloud.height),
        }

    @classmethod
    def from_depth(
        cls,
        det: Detection2DBBox,
        depth: Image,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
        filters: list[PointCloudFilter] | None = None,
        max_depth: float = 10.0,
        depth_gap: float = 0.1,
        mask_scale: float = 0.9,
    ) -> Detection3DPC | None:
        """Create a Detection3D by unprojecting the detection's depth pixels.

        ``depth`` must be aligned to the detection's image (same intrinsics and
        size). uint16 depth is taken as millimeters, float as meters. Only the
        depth cluster containing the median survives (``depth_gap`` split) —
        mask/bbox edges bleed into the background across a depth jump. The
        segmentation mask is eroded to ``mask_scale`` of its size first, since
        the bleed lives on the mask boundary.
        """
        # no radius_outlier: dense depth clouds make radius search expensive,
        # and the depth-gap cluster above already drops disconnected points
        if filters is None:
            filters = [statistical()]

        pixels = image_view(depth)
        depth_m = pixels.astype(np.float32)
        if pixels.dtype.kind == "u" and pixels.dtype.itemsize == 2:
            depth_m *= 0.001

        height, width = depth_m.shape[:2]
        seg_mask = getattr(det, "mask", None)
        if seg_mask is not None:
            pixel_mask = seg_mask > 0
            if mask_scale < 1.0:
                radius = float(np.sqrt(pixel_mask.sum() / np.pi))
                erode_px = round((1.0 - mask_scale) * radius)
                if erode_px > 0:
                    kernel = np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
                    eroded = cv2.erode(pixel_mask.astype(np.uint8), kernel).astype(bool)
                    if eroded.any():
                        pixel_mask = eroded
        else:
            x_min, y_min, x_max, y_max = det.bbox
            pixel_mask = np.zeros((height, width), dtype=bool)
            pixel_mask[
                max(int(y_min), 0) : min(int(y_max) + 1, height),
                max(int(x_min), 0) : min(int(x_max) + 1, width),
            ] = True

        rows, cols = np.nonzero(pixel_mask)
        z = depth_m[rows, cols]
        valid = (z > 0) & (z < max_depth)
        if not valid.any():
            return None
        rows, cols, z = rows[valid], cols[valid], z[valid]

        # keep the depth cluster containing the median
        order = np.argsort(z)
        z_sorted = z[order]
        gaps = np.nonzero(np.diff(z_sorted) > depth_gap)[0]
        starts = np.concatenate(([0], gaps + 1))
        ends = np.concatenate((gaps + 1, [len(z_sorted)]))
        median_idx = np.searchsorted(z_sorted, np.median(z_sorted))
        for start, end in zip(starts, ends, strict=False):
            if start <= median_idx < end:
                keep = order[start:end]
                rows, cols, z = rows[keep], cols[keep], z[keep]
                break

        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]
        points_optical = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))

        optical_to_world = inverse_transform(world_to_optical_transform)
        matrix = transform_matrix(optical_to_world.transform)
        points_world = points_optical @ matrix[:3, :3].T + matrix[:3, 3]
        detection_pc = pointcloud_from_xyz(
            points_world,
            header=Header(stamp=depth.header.stamp, frame_id=optical_to_world.header.frame_id),
        )

        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        if detection_pc.width * detection_pc.height == 0:
            return None

        return cls(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            pointcloud=detection_pc,
            transform=world_to_optical_transform,
            frame_id=detection_pc.header.frame_id,
        )

    @classmethod
    def from_2d(  # type: ignore[override]
        cls,
        det: Detection2DBBox,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
        # filters are to be adjusted based on the sensor noise characteristics if feeding
        # sensor data directly
        filters: list[PointCloudFilter] | None = None,
    ) -> Detection3DPC | None:
        """Create a Detection3D from a 2D detection by projecting world pointcloud.

        This method handles:
        1. Projecting world pointcloud to camera frame
        2. Filtering points within the 2D detection bounding box
        3. Cleaning up the pointcloud (height filter, outlier removal)
        4. Hidden point removal from camera perspective

        Args:
            det: The 2D detection
            world_pointcloud: Full pointcloud in world frame
            camera_info: Camera calibration info
            world_to_camerlka_transform: Transform from world to camera frame
            filters: List of functions to apply to the pointcloud for filtering
        Returns:
            Detection3D with filtered pointcloud, or None if no valid points
        """
        # Set default filters if none provided
        if filters is None:
            filters = [
                # height_filter(0.1),
                raycast(),
                radius_outlier(),
                statistical(),
            ]

        # Extract camera parameters
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]
        image_width = camera_info.width
        image_height = camera_info.height

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # Convert pointcloud to numpy array
        world_points = pointcloud_xyz(world_pointcloud)
        indices: NDArray[np.int64] = np.arange(len(world_points), dtype=np.int64)

        # Project points to camera frame
        points_homogeneous = np.hstack([world_points, np.ones((world_points.shape[0], 1))])
        extrinsics_matrix = transform_matrix(world_to_optical_transform.transform)
        points_camera = (extrinsics_matrix @ points_homogeneous.T).T

        # Filter out points behind the camera
        valid_mask = points_camera[:, 2] > 0
        points_camera = points_camera[valid_mask]
        world_points = world_points[valid_mask]
        indices = indices[valid_mask]

        if len(world_points) == 0:
            return None

        # Project to 2D
        points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

        # Filter points within image bounds
        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < image_width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < image_height)
        )
        points_2d = points_2d[in_image_mask]
        world_points = world_points[in_image_mask]
        indices = indices[in_image_mask]

        if len(world_points) == 0:
            return None

        # Find points within this detection — segmentation mask if present
        # (Detection2DSeg), else bbox with a small margin
        seg_mask = getattr(det, "mask", None)
        if seg_mask is not None:
            cols = np.minimum(points_2d[:, 0].astype(int), seg_mask.shape[1] - 1)
            rows = np.minimum(points_2d[:, 1].astype(int), seg_mask.shape[0] - 1)
            in_det_mask = seg_mask[rows, cols] > 0
        else:
            x_min, y_min, x_max, y_max = det.bbox
            margin = 5  # pixels
            in_det_mask = (
                (points_2d[:, 0] >= x_min - margin)
                & (points_2d[:, 0] <= x_max + margin)
                & (points_2d[:, 1] >= y_min - margin)
                & (points_2d[:, 1] <= y_max + margin)
            )

        detection_points = world_points[in_det_mask]

        if detection_points.shape[0] == 0:
            # print(f"No points found in detection bbox after projection. {det.name}")
            return None

        # Create initial pointcloud for this detection
        keep = np.zeros(world_pointcloud.width * world_pointcloud.height, dtype=bool)
        keep[indices[in_det_mask]] = True
        initial_pc = select_points(world_pointcloud, keep)

        # Apply filters - each filter gets all arguments
        detection_pc = initial_pc
        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        # Final check for empty pointcloud
        if detection_pc.width * detection_pc.height == 0:
            return None

        # Create Detection3D with filtered pointcloud
        return cls(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            pointcloud=detection_pc,
            transform=world_to_optical_transform,
            frame_id=world_pointcloud.header.frame_id,
        )
