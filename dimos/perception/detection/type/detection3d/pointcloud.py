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

from dimos_generated.builtin_interfaces.msg import Time
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

from dimos.msgs.geometry import inverse_transform, transform_matrix
from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import (
    concatenate_clouds,
    pointcloud_from_xyz,
    pointcloud_xyz,
    select_points,
)
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.perception.detection.type.detection3d.base import Detection3D
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    PointCloudFilter,
    radius_outlier,
    raycast,
    statistical,
)

if TYPE_CHECKING:
    from dimos_generated.sensor_msgs.msg import CameraInfo, Image
    import open3d as o3d

    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


def lattice_quantum(points: np.ndarray) -> float | None:
    """The grid pitch when coordinates lie on a lattice; None for continuous scans.

    Grid-quantized sources carry their pitch in the data itself; it sizes
    merge cells and the projection splat. Quantization does not classify the
    source - a mm-integer wire format grids a scan without making it a map.
    """
    sample = points[:2048]
    x = np.unique(sample[:, 0])
    if len(x) < 8:
        return None
    diffs = np.diff(x)
    diffs = diffs[diffs > 1e-9]
    if len(diffs) == 0:
        return None
    quantum = float(diffs.min())
    if quantum < 1e-4:
        return None
    scaled = (sample - sample[0]) / quantum
    if float(np.abs(scaled - np.round(scaled)).max()) > 0.01:
        return None
    return quantum


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2 = field(
        default_factory=lambda: PointCloud2(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            height=0,
            width=0,
            fields=[],
            is_bigendian=False,
            point_step=0,
            row_step=0,
            data=np.array([], dtype=np.uint8),
            is_dense=False,
        )
    )

    @functools.cached_property
    def center(self) -> Vector3:
        center = pointcloud_xyz(self.pointcloud).mean(axis=0)
        return Vector3(x=float(center[0]), y=float(center[1]), z=float(center[2]))

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
            pointcloud=concatenate_clouds(self.pointcloud, other.pointcloud),
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
            header=self.pointcloud.header,
            pose=Pose(
                position=Point(x=self.center.x, y=self.center.y, z=self.center.z),
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
            ),
        )

    def _open3d(self) -> o3d.geometry.PointCloud:
        import open3d as o3d

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
        import cv2

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

    @staticmethod
    def project_pixels(points_camera: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
        """Pixel coordinates of camera-frame points under the camera's own model.

        Applies the recorded distortion (equidistant fisheye or the radtan
        family) so cloud pixels land where the image's pixels actually are;
        an undistorted calibration falls through to the pinhole projection.
        Points beyond the model's monotonic radius get out-of-image pixels.
        """
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]
        coefficients = np.asarray(
            camera_info.d if camera_info.d is not None else (), dtype=np.float64
        )
        xy = points_camera[:, :2] / points_camera[:, 2:3]
        if coefficients.size == 0 or not np.any(coefficients):
            return np.column_stack((xy[:, 0] * fx + cx, xy[:, 1] * fy + cy))

        import cv2

        fisheye = camera_info.distortion_model == "equidistant"
        # The polynomial is calibrated only out to the image corner; beyond
        # it the projection extrapolates or folds back into the image, so
        # points past the corner's angle (or past the first fold) are
        # unmappable.
        corners = np.array(
            [
                [0, 0],
                [camera_info.width, 0],
                [0, camera_info.height],
                [camera_info.width, camera_info.height],
            ],
            dtype=np.float64,
        )
        corner_limit = float(np.hypot((corners[:, 0] - cx) / fx, (corners[:, 1] - cy) / fy).max())
        theta = np.linspace(0.0, np.pi / 2 * 0.99, 2048)
        if fisheye:
            k = np.zeros(4)
            k[: min(4, coefficients.size)] = coefficients[:4]
            distorted = theta * (
                1 + k[0] * theta**2 + k[1] * theta**4 + k[2] * theta**6 + k[3] * theta**8
            )
            radius = np.tan(theta)
        else:
            k = np.zeros(3)
            radial = coefficients[[0, 1]].tolist() + (
                [coefficients[4]] if coefficients.size > 4 else []
            )
            k[: len(radial)] = radial
            radius = np.tan(theta)
            distorted = radius * (1 + k[0] * radius**2 + k[1] * radius**4 + k[2] * radius**6)
        beyond = np.nonzero((np.diff(distorted) <= 0) | (distorted[1:] > corner_limit))[0]
        r_max = radius[beyond[0]] if len(beyond) else radius[-1]

        pixels = np.full((len(points_camera), 2), -1.0)
        mappable = (xy**2).sum(axis=1) <= r_max**2
        if mappable.any():
            pts = np.ascontiguousarray(points_camera[mappable, :3], dtype=np.float64)
            camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            zero = np.zeros(3)
            if fisheye:
                projected, _ = cv2.fisheye.projectPoints(
                    pts.reshape(-1, 1, 3), zero, zero, camera_matrix, coefficients[:4]
                )
            else:
                projected, _ = cv2.projectPoints(pts, zero, zero, camera_matrix, coefficients)
            pixels[mappable] = projected.reshape(-1, 2)
        return pixels

    @staticmethod
    def project_cloud(
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Project a frame once for the existing batched detection API."""
        points, pixels, _ = Detection3DPC._project_cloud_indices(
            world_pointcloud, camera_info, world_to_optical_transform
        )
        return points, pixels

    @staticmethod
    def _project_cloud_indices(
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
    ) -> tuple[np.ndarray, np.ndarray, NDArray[np.int64]]:
        """Project a world cloud through the camera once.

        Returns the world points that land inside the image and their pixel
        coordinates - the detection-independent half of ``from_2d``, shared
        by every detection of one frame via ``from_projection``.
        """
        world_points = pointcloud_xyz(world_pointcloud)
        indices = np.arange(len(world_points), dtype=np.int64)

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
            return world_points, np.empty((0, 2)), indices

        points_2d = Detection3DPC.project_pixels(points_camera, camera_info)

        # Filter points within image bounds
        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < camera_info.width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < camera_info.height)
        )
        return world_points[in_image_mask], points_2d[in_image_mask], indices[in_image_mask]

    @classmethod
    def from_projection(
        cls,
        det: Detection2DBBox,
        world_points: np.ndarray,
        points_2d: np.ndarray,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
        frame_id: str,
        timestamp: float,
        filters: list[PointCloudFilter] | None = None,
        splat_m: float | None = None,
        *,
        projected_cloud: PointCloud2 | None = None,
    ) -> Detection3DPC | None:
        """Create a Detection3D by selecting from a shared frame projection.

        ``world_points`` and ``points_2d`` come from ``project_cloud`` for
        this detection's frame; only the mask selection and the per-detection
        filters run here. ``splat_m`` is the half-size of a source cell: a
        point then selects when any of its projected footprint touches the
        mask, not only its center pixel - center-only sampling starves masks
        a few cells wide.
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
            height, width = seg_mask.shape[:2]

            def mask_hit(cols_f: np.ndarray, rows_f: np.ndarray) -> np.ndarray:
                cols = np.clip(cols_f.astype(int), 0, width - 1)
                rows = np.clip(rows_f.astype(int), 0, height - 1)
                hit: np.ndarray = seg_mask[rows, cols] > 0
                return hit

            in_det_mask = mask_hit(points_2d[:, 0], points_2d[:, 1])
            if splat_m is not None:
                camera = transform_matrix(inverse_transform(world_to_optical_transform).transform)[
                    :3, 3
                ]
                ranges = np.linalg.norm(world_points - camera, axis=1)
                radius = splat_m * camera_info.k[0] / np.maximum(ranges, 1e-6)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    in_det_mask |= mask_hit(
                        points_2d[:, 0] + dx * radius, points_2d[:, 1] + dy * radius
                    )
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
        initial_pc = (
            select_points(projected_cloud, in_det_mask)
            if projected_cloud is not None
            else pointcloud_from_xyz(
                detection_points,
                header=Header(frame_id=frame_id, stamp=time_from_seconds(timestamp)),
            )
        )

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
            frame_id=frame_id,
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

        One-detection convenience over ``project_cloud`` + ``from_projection``;
        callers lifting several detections of one frame should project once
        and call ``from_projection`` per detection instead. The mask splat
        radius comes from the cloud's own lattice pitch, so every caller
        samples a grid-quantized source the same way.
        """
        world_points, points_2d, indices = cls._project_cloud_indices(
            world_pointcloud, camera_info, world_to_optical_transform
        )
        keep = np.zeros(world_pointcloud.width * world_pointcloud.height, dtype=bool)
        keep[indices] = True
        projected_cloud = select_points(world_pointcloud, keep)
        quantum = lattice_quantum(world_points)
        return cls.from_projection(
            det,
            world_points,
            points_2d,
            camera_info,
            world_to_optical_transform,
            world_pointcloud.header.frame_id,
            to_seconds(world_pointcloud.header.stamp),
            filters,
            splat_m=quantum / 2 if quantum is not None else None,
            projected_cloud=projected_cloud,
        )
