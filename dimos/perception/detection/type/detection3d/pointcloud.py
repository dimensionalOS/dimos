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

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection3d.base import Detection3D
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    PointCloudFilter,
    radius_outlier,
    raycast,
    statistical,
)

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


@dataclass(frozen=True)
class ProjectedPointCloud:
    """Reusable per-frame projection with aligned source, camera, and image points.

    Keeping the projected arrays together lets multiple detections share one
    point-cloud transform and projection.
    """

    source_points: np.ndarray
    camera_points: np.ndarray
    image_points: np.ndarray
    image_width: int
    image_height: int

    @classmethod
    def from_pointcloud(
        cls,
        pointcloud: PointCloud2,
        camera_info: CameraInfo,
        pointcloud_to_camera: Transform,
    ) -> ProjectedPointCloud:
        """Transform and project valid points into image coordinates."""
        intrinsics = np.asarray(camera_info.K, dtype=np.float64)
        if intrinsics.size != 9:
            raise ValueError("camera intrinsics must contain nine values")
        intrinsics = intrinsics.reshape(3, 3)
        if not np.all(np.isfinite(intrinsics)) or intrinsics[0, 0] <= 0 or intrinsics[1, 1] <= 0:
            raise ValueError("camera intrinsics must be finite with positive focal lengths")
        if camera_info.width <= 0 or camera_info.height <= 0:
            raise ValueError("camera calibration dimensions must be positive")

        raw_points, _ = pointcloud.as_numpy()
        source_points = np.asarray(raw_points, dtype=np.float64)
        if source_points.ndim != 2 or source_points.shape[1] != 3:
            raise ValueError("point cloud positions must have shape (N, 3)")
        source_points = source_points[np.all(np.isfinite(source_points), axis=1)]

        transform = np.asarray(pointcloud_to_camera.to_matrix(), dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("pointcloud_to_camera must be a finite 4x4 transform")
        homogeneous = np.column_stack(
            (source_points, np.ones(len(source_points), dtype=np.float64))
        )
        camera_points = (transform @ homogeneous.T).T[:, :3]
        in_front = np.all(np.isfinite(camera_points), axis=1) & (camera_points[:, 2] > 0)
        source_points = source_points[in_front]
        camera_points = camera_points[in_front]

        projected = (intrinsics @ camera_points.T).T
        image_points = projected[:, :2] / projected[:, 2, np.newaxis]
        in_image = (
            np.all(np.isfinite(image_points), axis=1)
            & (image_points[:, 0] >= 0)
            & (image_points[:, 0] < camera_info.width)
            & (image_points[:, 1] >= 0)
            & (image_points[:, 1] < camera_info.height)
        )
        return cls(
            source_points=source_points[in_image],
            camera_points=camera_points[in_image],
            image_points=image_points[in_image],
            image_width=camera_info.width,
            image_height=camera_info.height,
        )

    def points_in_detection(
        self,
        detection: Detection2DBBox,
        *,
        nearest_per_pixel: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return source- and camera-frame points inside one mask or box."""
        mask = getattr(detection, "mask", None)
        if mask is not None:
            mask = np.asarray(mask)
            if mask.ndim != 2 or mask.shape != (self.image_height, self.image_width):
                raise ValueError("segmentation mask dimensions must match the calibrated image")
            pixels = self.image_points.astype(np.intp)
            selected = mask[pixels[:, 1], pixels[:, 0]] > 0
        else:
            x_min, y_min, x_max, y_max = detection.bbox
            margin = 5
            selected = (
                (self.image_points[:, 0] >= x_min - margin)
                & (self.image_points[:, 0] <= x_max + margin)
                & (self.image_points[:, 1] >= y_min - margin)
                & (self.image_points[:, 1] <= y_max + margin)
            )

        source_points = self.source_points[selected]
        camera_points = self.camera_points[selected]
        image_points = self.image_points[selected]
        if nearest_per_pixel and len(image_points):
            pixels = image_points.astype(np.intp)
            linear_pixels = pixels[:, 1] * self.image_width + pixels[:, 0]
            order = np.lexsort((camera_points[:, 2], linear_pixels))
            sorted_linear = linear_pixels[order]
            first_per_pixel = np.empty(len(order), dtype=bool)
            first_per_pixel[0] = True
            first_per_pixel[1:] = sorted_linear[1:] != sorted_linear[:-1]
            keep = order[first_per_pixel]
            source_points = source_points[keep]
            camera_points = camera_points[keep]
        return source_points, camera_points


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2 = field(default_factory=PointCloud2)

    @functools.cached_property
    def center(self) -> Vector3:
        return Vector3(*self.pointcloud.center)

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using pointcloud center.

        Returns pose in world frame with identity rotation.
        The pointcloud is already in world frame.
        """
        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=self.center,
            orientation=(0.0, 0.0, 0.0, 1.0),  # Identity quaternion
        )

    def get_bounding_box(self):  # type: ignore[no-untyped-def]
        """Get axis-aligned bounding box of the detection's pointcloud."""
        return self.pointcloud.axis_aligned_bounding_box

    def get_oriented_bounding_box(self):  # type: ignore[no-untyped-def]
        """Get oriented bounding box of the detection's pointcloud."""
        return self.pointcloud.oriented_bounding_box

    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of the detection's bounding box."""
        return self.pointcloud.bounding_box_dimensions

    def bounding_box_intersects(self, other: Detection3DPC) -> bool:
        """Check if this detection's bounding box intersects with another's."""
        return self.pointcloud.bounding_box_intersects(other.pointcloud)

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        # The pointcloud is in world frame, and transform gives camera position in world
        center_world = self.center
        # Camera position in world frame is the translation part of the transform
        camera_pos = self.transform.translation
        # Use Vector3 subtraction and magnitude
        distance = (center_world - camera_pos).magnitude()

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "points": str(len(self.pointcloud)),
        }

    @classmethod
    def from_depth(
        cls,
        det: Detection2DBBox,
        depth: Image,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
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

        depth_m = np.asarray(depth.data, dtype=np.float32)
        if depth.data.dtype == np.uint16:
            depth_m *= 0.001

        height, width = depth_m.shape[:2]
        seg_mask = getattr(det, "mask", None)
        if seg_mask is not None:
            pixel_mask = seg_mask > 0
            if mask_scale < 1.0:
                import cv2

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

        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        points_optical = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))

        detection_pc = PointCloud2.from_numpy(points_optical, timestamp=det.ts).transform(
            -world_to_optical_transform
        )

        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        if len(detection_pc.pointcloud.points) == 0:
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
            frame_id=detection_pc.frame_id,
        )

    @classmethod
    def from_2d(  # type: ignore[override]
        cls,
        det: Detection2DBBox,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
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

        projected = ProjectedPointCloud.from_pointcloud(
            world_pointcloud, camera_info, world_to_optical_transform
        )
        detection_points, _ = projected.points_in_detection(det)

        if detection_points.shape[0] == 0:
            # print(f"No points found in detection bbox after projection. {det.name}")
            return None

        # Create initial pointcloud for this detection
        initial_pc = PointCloud2.from_numpy(
            detection_points,
            frame_id=world_pointcloud.frame_id,
            timestamp=world_pointcloud.ts,
        )

        # Apply filters - each filter gets all arguments
        detection_pc = initial_pc
        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        # Final check for empty pointcloud
        if len(detection_pc.pointcloud.points) == 0:
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
            frame_id=world_pointcloud.frame_id,
        )
