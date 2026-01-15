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

import cv2
from dimos_lcm.geometry_msgs import Pose
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection3D as ROSDetection3D, Detection3DArray
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.base import Detection3D

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.perception.detection.type.detection2d import ImageDetections2D


@dataclass(kw_only=True)
class Object(Detection3D):
    """3D object detection combining bounding box and pointcloud representations.

    Represents a detected object in 3D space with support for accumulating
    multiple detections over time.
    """

    object_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    center: Vector3
    size: Vector3
    pose: PoseStamped
    pointcloud: PointCloud2
    camera_transform: Transform | None = None
    mask: np.ndarray[Any, np.dtype[np.uint8]] | None = None
    detections_count: int = 1

    def update_object(self, other: Object) -> None:
        """Update this object with data from another detection.

        Accumulates pointclouds by transforming the new pointcloud to world frame
        and adding it to the existing pointcloud. Updates center and camera_transform,
        and increments the detections_count.

        Args:
            other: Another Object instance with newer detection data.
        """
        # Accumulate pointclouds if transforms are available
        if other.camera_transform is not None:
            # Transform new pointcloud to world frame and add to existing
            # transformed_pc = other.pointcloud.transform(other.camera_transform)
            # self.pointcloud = self.pointcloud + transformed_pc

            # Recompute center from accumulated pointcloud
            self.pointcloud = other.pointcloud
            pc_center = other.pointcloud.center
            self.center = Vector3(pc_center.x, pc_center.y, pc_center.z)
        else:
            # No transform available, just replace
            self.pointcloud = other.pointcloud
            self.center = other.center

        self.camera_transform = other.camera_transform
        self.size = other.size
        self.pose = other.pose
        self.track_id = other.track_id
        self.mask = other.mask
        self.name = other.name
        self.bbox = other.bbox
        self.confidence = other.confidence
        self.class_id = other.class_id
        self.ts = other.ts
        self.frame_id = other.frame_id
        self.image = other.image
        self.detections_count += 1

    def get_oriented_bounding_box(self) -> Any:
        """Get oriented bounding box of the pointcloud."""
        return self.pointcloud.get_oriented_bounding_box()

    def scene_entity_label(self) -> str:
        """Get label for scene visualization."""
        if self.detections_count > 1:
            return f"{self.name} ({self.detections_count})"
        return f"{self.track_id}/{self.name} ({self.confidence:.0%})"

    def to_detection3d_msg(self) -> ROSDetection3D:
        """Convert to ROS Detection3D message."""
        obb = self.get_oriented_bounding_box()  # type: ignore[no-untyped-call]
        orientation = Quaternion.from_rotation_matrix(obb.R)

        msg = ROSDetection3D()
        msg.header = Header(self.ts, self.frame_id)
        msg.id = str(self.track_id)
        msg.bbox.center = Pose(
            position=Vector3(obb.center[0], obb.center[1], obb.center[2]),
            orientation=orientation,
        )
        msg.bbox.size = Vector3(obb.extent[0], obb.extent[1], obb.extent[2])

        return msg

    def agent_encode(self) -> dict[str, Any]:
        """Encode for agent consumption."""
        return {
            "id": self.track_id,
            "name": self.name,
            "detections": self.detections_count,
            "last_seen": f"{round(time.time() - self.ts)}s ago",
        }

    def to_dict(self) -> dict[str, Any]:
        """Convert object to dictionary with all relevant data."""
        return {
            "object_id": self.object_id,
            "track_id": self.track_id,
            "class_id": self.class_id,
            "name": self.name,
            "mask": self.mask,
            "pointcloud": self.pointcloud.as_numpy(),
            "image": self.image.as_numpy() if self.image else None,
        }

    @classmethod
    def from_2d_to_list(
        cls,
        detections_2d: ImageDetections2D[Detection2DSeg],
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        camera_transform: Transform | None = None,
        depth_scale: float = 1.0,
        depth_trunc: float = 10.0,
        statistical_nb_neighbors: int = 10,
        statistical_std_ratio: float = 0.5,
        voxel_downsample: float = 0.005,
        mask_erode_pixels: int = 3,
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

        Returns:
            List of Object instances with pointclouds
        """
        color_cv = color_image.to_opencv()
        if color_cv.ndim == 3 and color_cv.shape[2] == 3:
            color_cv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2RGB)

        depth_cv = depth_image.to_opencv()
        h, w = depth_cv.shape[:2]

        # Build Open3D camera intrinsics
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
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
                    mask_uint8 = mask_uint8 * 255  # type: ignore[assignment]
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

            pc0 = PointCloud2(
                pcd,
                frame_id=depth_image.frame_id,
                ts=depth_image.ts,
            ).voxel_downsample(voxel_downsample)

            pcd_filtered, _ = pc0.pointcloud.remove_statistical_outlier(
                nb_neighbors=statistical_nb_neighbors,
                std_ratio=statistical_std_ratio,
            )

            if len(pcd_filtered.points) < 10:
                continue

            pc = PointCloud2(
                pcd_filtered,
                frame_id=depth_image.frame_id,
                ts=depth_image.ts,
            )

            # Transform pointcloud to world frame if camera_transform is provided
            if camera_transform is not None:
                pc = pc.transform(camera_transform)
                frame_id = camera_transform.frame_id
            else:
                frame_id = depth_image.frame_id

            # Compute center from pointcloud
            obb = pc.pointcloud.get_oriented_bounding_box()
            center = Vector3(obb.center[0], obb.center[1], obb.center[2])
            size = Vector3(obb.extent[0], obb.extent[1], obb.extent[2])
            orientation = Quaternion.from_rotation_matrix(obb.R)
            pose = PoseStamped(
                ts=det.ts,
                frame_id=frame_id,
                position=center,
                orientation=orientation,
            )

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

    Each object's points are colored based on its track_id.

    Args:
        objects: List of Object instances with pointclouds

    Returns:
        Combined PointCloud2 with all points colored by object (empty if no points).
    """
    if not objects:
        return PointCloud2(pointcloud=o3d.geometry.PointCloud(), frame_id="", ts=0.0)

    all_points = []
    all_colors = []

    for _i, obj in enumerate(objects):
        points, colors = obj.pointcloud.as_numpy()
        if len(points) == 0:
            continue

        try:
            seed = int(obj.object_id, 16)
        except (ValueError, TypeError):
            seed = abs(hash(obj.object_id))
        np.random.seed(abs(seed) % (2**32 - 1))
        track_color = np.random.randint(50, 255, 3) / 255.0

        if colors is not None:
            blended = np.clip(0.6 * colors + 0.4 * track_color, 0.0, 1.0)
        else:
            blended = np.tile(track_color, (len(points), 1))

        all_points.append(points)
        all_colors.append(blended)

    if not all_points:
        return PointCloud2(
            pointcloud=o3d.geometry.PointCloud(), frame_id=objects[0].frame_id, ts=objects[0].ts
        )

    combined_points = np.vstack(all_points)
    combined_colors = np.vstack(all_colors)

    pc = PointCloud2.from_numpy(
        combined_points,
        frame_id=objects[0].frame_id,
        timestamp=objects[0].ts,
    )
    pcd = pc.pointcloud
    pcd.colors = o3d.utility.Vector3dVector(combined_colors)
    pc.pointcloud = pcd

    return pc


def to_detection3d_array(objects: list[Object]) -> Detection3DArray:
    """Convert a list of Objects to a ROS Detection3DArray message.

    Args:
        objects: List of Object instances

    Returns:
        Detection3DArray ROS message
    """
    array = Detection3DArray()

    if objects:
        array.header = Header(objects[0].ts, objects[0].frame_id)

    for obj in objects:
        array.detections.append(obj.to_detection3d_msg())

    return array
