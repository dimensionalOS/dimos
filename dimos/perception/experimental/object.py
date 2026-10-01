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
import time
from typing import TYPE_CHECKING, Any
import uuid

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D as ROSDetection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import numpy as np

from dimos.msgs.geometry import quaternion_from_matrix, transform_matrix
from dimos.msgs.image import image_to_rgb, image_view
from dimos.msgs.pointcloud import (
    concatenate_clouds,
    pointcloud_from_xyz,
    pointcloud_from_xyz_rgb,
    pointcloud_rgb,
    pointcloud_to_open3d,
    pointcloud_xyz,
    voxel_downsample_cloud,
)
from dimos.msgs.time import time_from_seconds
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.base import Detection3D

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


@dataclass(kw_only=True)
class Object(Detection3D):
    """3D object detection combining bounding box and pointcloud representations.

    Represents a detected object in 3D space with support for accumulating
    multiple detections over time.
    """

    object_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    center: Vector3
    size: Vector3
    pose: PoseStamped
    pointcloud: PointCloud2
    camera_transform: TransformStamped | None = None
    mask: np.ndarray[Any, np.dtype[np.uint8]] | None = None
    detections_count: int = 1
    visual_embedding: np.ndarray[Any, np.dtype[np.float32]] | None = None
    observation_partial: bool = False
    identity_status: str | None = None
    identity_basis: str | None = None
    last_seen_ts: float | None = None

    def __post_init__(self) -> None:
        self.set_center(self.center)

    def set_center(self, center: Vector3) -> None:
        """Update the canonical center and its pose representation together."""
        self.center = Vector3(x=center.x, y=center.y, z=center.z)
        self.pose.pose.position = Point(x=center.x, y=center.y, z=center.z)

    def update_object(self, other: Object, *, accumulate_pointcloud: bool = True) -> None:
        """Update this object with data from another detection.

        Optionally accumulates pointclouds already transformed to the world frame. Updates
        geometry and bookkeeping from the new detection and increments ``detections_count``.

        Args:
            other: Another Object instance with newer detection data.
            accumulate_pointcloud: Whether to retain points from earlier detections.
        """
        # Accumulate pointclouds (both are already in world frame) for visualization,
        # but use the latest single-detection geometry for obstacle sizing.
        # Recomputing size from accumulated clouds inflates obstacles unrealistically.
        if (
            accumulate_pointcloud
            and other.camera_transform is not None
            and self.camera_transform is not None
        ):
            self.pointcloud = concatenate_clouds(self.pointcloud, other.pointcloud)
        else:
            self.pointcloud = other.pointcloud

        # Always use the latest detection's geometry (not accumulated cloud OBB)
        self.size = other.size
        self.pose = other.pose
        self.set_center(other.center)

        self.camera_transform = other.camera_transform
        self.track_id = other.track_id
        self.mask = other.mask
        self.name = other.name
        self.bbox = other.bbox
        self.confidence = other.confidence
        self.class_id = other.class_id
        self.ts = other.ts
        self.frame_id = other.frame_id
        self.image = other.image
        if other.visual_embedding is not None:
            self.visual_embedding = other.visual_embedding
        self.observation_partial = other.observation_partial
        self.detections_count += 1

    def get_oriented_bounding_box(self) -> Any:
        """Get oriented bounding box of the pointcloud."""
        return pointcloud_to_open3d(self.pointcloud).get_oriented_bounding_box()

    def _detection3d_bbox_components(self) -> tuple[Vector3, Quaternion, Vector3]:
        """Return canonical geometry without refitting the point cloud."""
        center = self.center
        size = self.size
        orientation = self.pose.pose.orientation
        return (
            Vector3(x=center.x, y=center.y, z=center.z),
            Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w),
            Vector3(
                x=max(float(size.x), 1e-3),
                y=max(float(size.y), 1e-3),
                z=max(float(size.z), 1e-3),
            ),
        )

    def scene_entity_label(self) -> str:
        """Get label for scene visualization."""
        if self.detections_count > 1:
            return f"{self.name} ({self.detections_count})"
        return f"{self.track_id}/{self.name} ({self.confidence:.0%})"

    def to_detection3d_msg(self) -> ROSDetection3D:
        """Convert to ROS Detection3D message."""
        center, orientation, size = self._detection3d_bbox_components()

        msg = ROSDetection3D()
        msg.header = self.pose.header
        msg.id = self.object_id
        msg.results = [
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=self.name,
                    score=self.confidence,
                )
            )
        ]
        msg.bbox = BoundingBox3D(
            center=Pose(
                position=Point(x=center.x, y=center.y, z=center.z),
                orientation=orientation,
            ),
            size=size,
        )

        return msg

    def agent_encode(self) -> dict[str, Any]:
        """Encode for agent consumption."""
        return {
            "object_id": self.object_id,
            "track_id": self.track_id,
            "name": self.name,
            "detections": self.detections_count,
            "identity_status": self.identity_status,
            "identity_basis": self.identity_basis,
            "last_seen_ts": self.last_seen_ts,
            "last_seen": f"{round(time.time() - (self.last_seen_ts or self.ts))}s ago",
        }

    def to_dict(self) -> dict[str, Any]:
        """Convert object to dictionary with all relevant data."""
        colors = pointcloud_rgb(self.pointcloud)
        return {
            "object_id": self.object_id,
            "track_id": self.track_id,
            "class_id": self.class_id,
            "name": self.name,
            "identity_status": self.identity_status,
            "identity_basis": self.identity_basis,
            "last_seen_ts": self.last_seen_ts,
            "mask": self.mask,
            "pointcloud": (
                pointcloud_xyz(self.pointcloud),
                None if colors is None else colors.astype(np.float64) / 255.0,
            ),
            "image": image_view(self.image) if self.image else None,
        }

    @classmethod
    def from_2d_to_list(
        cls,
        detections_2d: ImageDetections2D[Detection2DSeg],
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        camera_transform: TransformStamped | None = None,
        depth_scale: float = 1.0,
        depth_trunc: float = 10.0,
        statistical_nb_neighbors: int = 10,
        statistical_std_ratio: float = 0.5,
        voxel_downsample: float = 0.005,
        mask_erode_pixels: int = 3,
        max_distance: float = 0.0,
        use_aabb: bool = False,
        max_obstacle_width: float = 0.0,
    ) -> list[Object]:
        """Create 3D Objects from 2D detections and RGBD images.

        Uses Open3D's optimized RGBD projection for efficient processing.

        Args:
            detections_2d: 2D detections with segmentation masks
            color_image: RGB color image
            depth_image: Depth image (in meters if depth_scale=1.0)
            camera_info: Camera intrinsics
            camera_transform: Optional transform from camera frame to world frame.
                If provided, pointclouds will be transformed to world frame.
            depth_scale: Scale factor for depth (1.0 for meters, 1000.0 for mm)
            depth_trunc: Maximum depth value in meters
            statistical_nb_neighbors: Neighbors for statistical outlier removal
            statistical_std_ratio: Std ratio for statistical outlier removal
            voxel_downsample: Voxel size (meters) for downsampling before filtering. Set <= 0 to skip.
            mask_erode_pixels: Number of pixels to erode the mask by to remove
                              noisy depth edge points. Set to 0 to disable.
            max_distance: Maximum distance from origin (meters) for object center.
                Objects beyond this are discarded as background. 0 disables the filter.
            use_aabb: Use axis-aligned bounding box instead of oriented bounding box.
                Produces upright obstacles with identity orientation.
            max_obstacle_width: Clamp X/Y size to this value (meters). Useful for
                manipulation where obstacles must fit within the gripper. 0 disables.

        Returns:
            List of Object instances with pointclouds
        """
        import cv2
        import open3d as o3d  # type: ignore[import-untyped]

        color_cv = image_to_rgb(color_image)
        depth_cv = image_view(depth_image)
        h, w = depth_cv.shape[:2]

        # Build Open3D camera intrinsics
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]
        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        objects: list[Object] = []

        for det in detections_2d.detections:
            if isinstance(det, Detection2DSeg):
                mask = det.mask
                store_mask = det.mask
            else:
                mask = np.zeros((h, w), dtype=np.uint8)
                x1, y1, x2, y2 = map(int, det.bbox)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)
                mask[y1:y2, x1:x2] = 255
                store_mask = mask

            if mask_erode_pixels > 0:
                mask_uint8 = mask.astype(np.uint8)
                if mask_uint8.max() == 1:
                    mask_uint8 = mask_uint8 * 255
                kernel_size = 2 * mask_erode_pixels + 1
                erode_kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
                )
                mask = cv2.erode(mask_uint8, erode_kernel)  # type: ignore[assignment]

            depth_masked = depth_cv.copy()
            depth_masked[mask == 0] = 0

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(color_cv.astype(np.uint8)),
                o3d.geometry.Image(depth_masked.astype(np.float32)),
                depth_scale=depth_scale,
                depth_trunc=depth_trunc,
                convert_rgb_to_intensity=False,
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

            initial = pointcloud_from_xyz_rgb(
                np.asarray(pcd.points),
                (np.asarray(pcd.colors) * 255).astype(np.uint8),
                header=depth_image.header,
            )
            pc0 = voxel_downsample_cloud(initial, voxel_downsample)
            pcd_filtered, _ = pointcloud_to_open3d(pc0).remove_statistical_outlier(
                nb_neighbors=statistical_nb_neighbors,
                std_ratio=statistical_std_ratio,
            )
            if len(pcd_filtered.points) < 10:
                continue
            header = depth_image.header
            if camera_transform is not None:
                if camera_transform.child_frame_id != header.frame_id:
                    raise ValueError("Camera transform source frame does not match depth image")
                pcd_filtered.transform(transform_matrix(camera_transform.transform))
                header = Header(
                    stamp=depth_image.header.stamp, frame_id=camera_transform.header.frame_id
                )
            pc = pointcloud_from_xyz_rgb(
                np.asarray(pcd_filtered.points),
                (np.asarray(pcd_filtered.colors) * 255).astype(np.uint8),
                header=header,
            )
            frame_id = header.frame_id

            # Compute bounding box: AABB for stable upright obstacles, OBB for tighter fit
            if use_aabb:
                aabb = pcd_filtered.get_axis_aligned_bounding_box()
                aabb_center = (aabb.min_bound + aabb.max_bound) / 2.0
                aabb_extent = aabb.max_bound - aabb.min_bound
                center = Vector3(x=aabb_center[0], y=aabb_center[1], z=aabb_center[2])
                sx, sy, sz = float(aabb_extent[0]), float(aabb_extent[1]), float(aabb_extent[2])
                orientation = Quaternion(w=1.0)
            else:
                obb = pcd_filtered.get_oriented_bounding_box()
                center = Vector3(x=obb.center[0], y=obb.center[1], z=obb.center[2])
                sx, sy, sz = float(obb.extent[0]), float(obb.extent[1]), float(obb.extent[2])
                orientation = quaternion_from_matrix(np.asarray(obb.R))

            if max_obstacle_width > 0:
                sx = min(sx, max_obstacle_width)
                sy = min(sy, max_obstacle_width)
            size = Vector3(x=sx, y=sy, z=sz)
            pose = PoseStamped(
                header=Header(stamp=time_from_seconds(det.ts), frame_id=frame_id),
                pose=Pose(
                    position=Point(x=center.x, y=center.y, z=center.z), orientation=orientation
                ),
            )

            # Skip objects too far from origin (background detections)
            if max_distance > 0:
                dist = (center.x**2 + center.y**2 + center.z**2) ** 0.5
                if dist > max_distance:
                    continue

            objects.append(
                cls(
                    bbox=det.bbox,
                    track_id=det.track_id,
                    class_id=det.class_id,
                    confidence=det.confidence,
                    name=det.name,
                    ts=det.ts,
                    image=det.image,
                    frame_id=frame_id,
                    pointcloud=pc,
                    center=center,
                    size=size,
                    pose=pose,
                    camera_transform=camera_transform,
                    mask=store_mask,
                )
            )

        return objects


def aggregate_pointclouds(objects: list[Object]) -> PointCloud2:
    """Aggregate all object pointclouds into a single colored pointcloud.

    Each object's points are colored based on its object_id.

    Args:
        objects: List of Object instances with pointclouds

    Returns:
        Combined PointCloud2 with all points colored by object (empty if no points).
    """
    header = (
        Header()
        if not objects
        else Header(frame_id=objects[0].frame_id, stamp=objects[0].pointcloud.header.stamp)
    )
    all_points = []
    all_colors = []
    for obj in objects:
        if obj.frame_id != header.frame_id:
            raise ValueError("Cannot aggregate object clouds in different frames")
        points = pointcloud_xyz(obj.pointcloud)
        if len(points) == 0:
            continue
        colors = pointcloud_rgb(obj.pointcloud)
        try:
            seed = int(obj.object_id, 16)
        except (ValueError, TypeError):
            seed = abs(hash(obj.object_id))
        rng = np.random.default_rng(abs(seed))
        track_color = rng.integers(50, 255, 3) / 255.0
        blended = (
            np.clip(0.6 * colors.astype(np.float64) / 255.0 + 0.4 * track_color, 0.0, 1.0)
            if colors is not None
            else np.tile(track_color, (len(points), 1))
        )
        all_points.append(points)
        all_colors.append(blended)
    if not all_points:
        return pointcloud_from_xyz(np.empty((0, 3), dtype=np.float32), header=header)
    return pointcloud_from_xyz_rgb(
        np.vstack(all_points), (np.vstack(all_colors) * 255).astype(np.uint8), header=header
    )


def to_detection3d_array(
    objects: list[Object],
    *,
    frame_id: str | None = None,
    ts: float | None = None,
) -> Detection3DArray:
    """Convert a list of Objects to a ROS Detection3DArray message.

    Args:
        objects: List of Object instances
        frame_id: Optional output frame override, including for an empty list.
        ts: Optional output timestamp override, including for an empty list.

    Returns:
        Detection3DArray ROS message
    """
    detections = [obj.to_detection3d_msg() for obj in objects]
    resolved_frame_id = frame_id
    if resolved_frame_id is None:
        resolved_frame_id = objects[0].frame_id if objects else ""
    resolved_ts = ts
    if resolved_ts is None:
        resolved_ts = objects[0].ts if objects else 0.0
    return Detection3DArray(
        header=Header(stamp=time_from_seconds(resolved_ts), frame_id=resolved_frame_id),
        detections=detections,
    )
