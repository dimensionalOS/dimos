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


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2 = field(default_factory=PointCloud2)

    @functools.cached_property
    def center(self) -> Vector3:
        return Vector3(*self.pointcloud.center)

    def __add__(self, other: Detection3DPC) -> Detection3DPC:
        """Union of two sightings of one object.

        The cloud is the union of both; identity metadata follows the
        higher-confidence sighting and time context follows the later one,
        matching latest-pose semantics.
        """
        later = self if self.ts >= other.ts else other
        stronger = self if self.confidence >= other.confidence else other
        return Detection3DPC(
            image=later.image,
            bbox=later.bbox,
            track_id=self.track_id,
            class_id=stronger.class_id,
            confidence=stronger.confidence,
            name=stronger.name,
            ts=later.ts,
            pointcloud=self.pointcloud + other.pointcloud,
            transform=later.transform,
            frame_id=self.frame_id,
        )

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

    @staticmethod
    def project_cloud(
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Project a world cloud through the camera once.

        Returns the world points that land inside the image and their pixel
        coordinates - the detection-independent half of ``from_2d``, shared
        by every detection of one frame via ``from_projection``.
        """
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        world_points, _ = world_pointcloud.as_numpy()

        # Project points to camera frame
        points_homogeneous = np.hstack([world_points, np.ones((world_points.shape[0], 1))])
        extrinsics_matrix = world_to_optical_transform.to_matrix()
        points_camera = (extrinsics_matrix @ points_homogeneous.T).T

        # Filter out points behind the camera
        valid_mask = points_camera[:, 2] > 0
        points_camera = points_camera[valid_mask]
        world_points = world_points[valid_mask]

        if len(world_points) == 0:
            return world_points, np.empty((0, 2))

        # Project to 2D
        points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

        # Filter points within image bounds
        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < camera_info.width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < camera_info.height)
        )
        return world_points[in_image_mask], points_2d[in_image_mask]

    @classmethod
    def from_projection(
        cls,
        det: Detection2DBBox,
        world_points: np.ndarray,
        points_2d: np.ndarray,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        frame_id: str,
        timestamp: float,
        filters: list[PointCloudFilter] | None = None,
    ) -> Detection3DPC | None:
        """Create a Detection3D by selecting from a shared frame projection.

        ``world_points`` and ``points_2d`` come from ``project_cloud`` for
        this detection's frame; only the mask selection and the per-detection
        filters run here.
        """
        # Set default filters if none provided
        if filters is None:
            filters = [
                # height_filter(0.1),
                raycast(),
                radius_outlier(),
                statistical(),
            ]

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
            return None

        # Create initial pointcloud for this detection
        initial_pc = PointCloud2.from_numpy(
            detection_points,
            frame_id=frame_id,
            timestamp=timestamp,
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
            frame_id=frame_id,
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

        One-detection convenience over ``project_cloud`` + ``from_projection``;
        callers lifting several detections of one frame should project once
        and call ``from_projection`` per detection instead.
        """
        world_points, points_2d = cls.project_cloud(
            world_pointcloud, camera_info, world_to_optical_transform
        )
        return cls.from_projection(
            det,
            world_points,
            points_2d,
            camera_info,
            world_to_optical_transform,
            world_pointcloud.frame_id,
            world_pointcloud.ts,
            filters,
        )
