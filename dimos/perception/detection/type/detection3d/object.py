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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D as ROSDetection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.base import Detection3D
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC, ProjectedCloud
from dimos.perception.detection.type.detection3d.pointcloud_filters import PointCloudFilter

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


# Per-object accumulated-cloud bound (see Object.update_object): above this
# many points the merged cloud is voxel-downsampled so per-object memory and
# per-frame publish cost stay proportional to object size, not sighting count.
_MAX_ACCUMULATED_POINTS = 20_000
_ACCUMULATION_VOXEL_M = 0.02


def _bounding_box(
    pcd: Any, use_aabb: bool = False
) -> tuple[Vector3, tuple[float, float, float], Quaternion]:
    """Center, (sx, sy, sz), and orientation of an open3d pointcloud's box.

    OBB gives the tighter fit, but Qhull fails on flat/degenerate clusters —
    e.g. a lidar cluster entirely on the floor plane — so fall back to the
    axis-aligned box (identity orientation) when it does, or when the caller
    asks for AABB outright. Single source of truth for the three call sites
    below (to_detection3d_msg / from_2d_to_list / from_2d_to_list_lidar).
    """
    if not use_aabb:
        try:
            obb = pcd.get_oriented_bounding_box()
            return (
                Vector3(obb.center[0], obb.center[1], obb.center[2]),
                (float(obb.extent[0]), float(obb.extent[1]), float(obb.extent[2])),
                Quaternion.from_rotation_matrix(obb.R),
            )
        except RuntimeError:
            pass
    aabb = pcd.get_axis_aligned_bounding_box()
    center = (aabb.min_bound + aabb.max_bound) / 2.0
    extent = aabb.max_bound - aabb.min_bound
    return (
        Vector3(center[0], center[1], center[2]),
        (float(extent[0]), float(extent[1]), float(extent[2])),
        Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _sharper(candidate: Image | None, current: Image | None) -> bool:
    """Whether ``candidate`` is a better labeling frame than ``current``.

    Any real image beats no image; otherwise the higher Laplacian-variance
    (sharper, less motion-blurred) frame wins. ``Image.sharpness`` downsamples
    to ~160px first, so this stays cheap even when called on every merge.
    """
    if candidate is None:
        return False
    if current is None:
        return True
    return candidate.sharpness > current.sharpness


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
    # Contextual-labeling refinement (see dimos/perception/contextual_labeling.py).
    # `name` stays the raw detector label; these hold the VLM's educated guess.
    refined_name: str | None = None
    refined_description: str | None = None

    @property
    def display_name(self) -> str:
        """Best available name: the VLM-refined label when present, else the detector's."""
        return self.refined_name or self.name

    def update_object(self, other: Object) -> None:
        """Update this object with data from another detection.

        Accumulates pointclouds by transforming the new pointcloud to world frame
        and adding it to the existing pointcloud. Updates center and camera_transform,
        and increments the detections_count.

        Args:
            other: Another Object instance with newer detection data.
        """
        # Accumulate pointclouds (both are already in world frame) for visualization,
        # but use the latest single-detection geometry for obstacle sizing.
        # Recomputing size from accumulated clouds inflates obstacles unrealistically.
        if other.camera_transform is not None and self.camera_transform is not None:
            merged = self.pointcloud + other.pointcloud
            # Bound accumulation: raw concatenation grows without limit over a
            # session (an object seen N times carries N overlapping clouds),
            # making every downstream publish/OBB pass slower each frame. A
            # voxel pass keeps the cloud bounded by the object's geometry
            # instead of by its sighting count.
            if len(merged) > _MAX_ACCUMULATED_POINTS:
                merged = merged.voxel_downsample(_ACCUMULATION_VOXEL_M)
            self.pointcloud = merged
        else:
            self.pointcloud = other.pointcloud

        # Always use the latest detection's geometry (not accumulated cloud OBB)
        self.center = other.center
        self.size = other.size
        self.pose = other.pose

        self.camera_transform = other.camera_transform
        self.track_id = other.track_id
        self.name = other.name
        self.confidence = other.confidence
        self.class_id = other.class_id
        self.ts = other.ts
        self.frame_id = other.frame_id

        # Keep the SHARPEST observation's appearance (full frame + the matching
        # 2D bbox) as the crop a VLM will later label from. A dark or
        # motion-smeared frame yields a useless crop even when the 3D geometry
        # is fine, and dark frames also embed semantically close to everything
        # -- so we quality-select the labeling appearance instead of blindly
        # taking the latest. Geometry above always tracks the latest detection;
        # image/bbox move together because bbox indexes into image's pixel grid.
        if _sharper(other.image, self.image):
            self.image = other.image
            self.bbox = other.bbox
        # The mask, by contrast, must ALWAYS track the latest detection: its
        # only consumer applies it to the CURRENT depth frame
        # (get_full_scene_pointcloud's exclude-object path, used by grasping),
        # so a mask pinned to an older sharp frame would zero the wrong pixels
        # once the camera moves.
        self.mask = other.mask
        self.detections_count += 1

    def get_oriented_bounding_box(self) -> Any:
        """Get oriented bounding box of the pointcloud."""
        return self.pointcloud.oriented_bounding_box

    def scene_entity_label(self) -> str:
        """Get label for scene visualization."""
        if self.detections_count > 1:
            return f"{self.display_name} ({self.detections_count})"
        return f"{self.track_id}/{self.display_name} ({self.confidence:.0%})"

    def to_detection3d_msg(self) -> ROSDetection3D:
        """Convert to ROS Detection3D message."""
        center, (sx, sy, sz), orientation = _bounding_box(self.pointcloud.pointcloud)
        size = Vector3(sx, sy, sz)

        msg = ROSDetection3D()
        msg.header = Header(self.ts, self.frame_id)
        msg.id = str(self.track_id)
        msg.bbox.center = Pose(position=center, orientation=orientation)
        msg.bbox.size = size

        return msg

    def agent_encode(self, now: float | None = None) -> dict[str, Any]:
        """Encode for agent consumption.

        Args:
            now: Reference time for the "last seen" age. Defaults to wall
                clock; pass the ObjectDB's stream time (``ObjectDB.now``) so
                replayed recordings report sensible ages.
        """
        encoded = {
            "object_id": self.object_id,
            "track_id": self.track_id,
            "name": self.display_name,
            "detections": self.detections_count,
            "last_seen": f"{round((now if now is not None else time.time()) - self.ts)}s ago",
        }
        if self.refined_name:
            encoded["detector_name"] = self.name
        return encoded

    def locate_encode(self, now: float | None = None) -> dict[str, Any]:
        """Encode for agent consumption including world position and confidence.

        Unlike agent_encode(), this exposes the object's world-frame location
        (center), size, and detection confidence so an agent can reason about
        *where* each observed item is, not just what it is.

        Args:
            now: Reference time for the "last seen" age (see agent_encode).
        """
        encoded = {
            "object_id": self.object_id,
            "name": self.display_name,
            "position": {"x": self.center.x, "y": self.center.y, "z": self.center.z},
            "frame": self.frame_id,
            "confidence": round(float(self.confidence), 2),
            "size": {"x": self.size.x, "y": self.size.y, "z": self.size.z},
            "detections": self.detections_count,
            "last_seen": f"{round((now if now is not None else time.time()) - self.ts)}s ago",
        }
        if self.refined_name:
            encoded["detector_name"] = self.name
            encoded["description"] = self.refined_description
        return encoded

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

            # Compute bounding box: AABB for stable upright obstacles, OBB for
            # tighter fit (with AABB fallback on degenerate clusters).
            center, (sx, sy, sz), orientation = _bounding_box(pc.pointcloud, use_aabb=use_aabb)

            if max_obstacle_width > 0:
                sx = min(sx, max_obstacle_width)
                sy = min(sy, max_obstacle_width)
            size = Vector3(sx, sy, sz)
            pose = PoseStamped(
                ts=det.ts,
                frame_id=frame_id,
                position=center,
                orientation=orientation,
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

    @classmethod
    def from_2d_to_list_lidar(
        cls,
        detections_2d: ImageDetections2D[Any],
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
        use_aabb: bool = False,
        max_distance: float = 0.0,
        max_obstacle_width: float = 0.0,
    ) -> list[Object]:
        """Create 3D Objects from 2D detections by projecting a world-frame lidar cloud.

        Sensor-alternative to :meth:`from_2d_to_list` for robots that have lidar
        but no depth camera (e.g. the Go2 quadruped). For each 2D detection this
        reuses :meth:`Detection3DPC.from_2d` to select the world-frame points that
        project inside the detection, then computes the object's center/size the
        same way :meth:`from_2d_to_list` does, so a single ``Object`` type flows
        into the ``ObjectDB``.

        Args:
            detections_2d: 2D detections (bbox is used to select points; masks,
                when present, are stored for downstream use).
            world_pointcloud: Lidar/global-map pointcloud already in world frame.
            camera_info: Camera intrinsics used to project points into the image.
            world_to_optical_transform: Transform from world to the camera optical
                frame (as returned by ``tf.get(camera_optical_frame, world)``).
            filters: Pointcloud noise filters applied per detection. When None,
                ``Detection3DPC.from_2d`` applies its own defaults; callers with
                sparse clouds may pass a lighter set.
            use_aabb: Use an axis-aligned bounding box (upright, identity
                orientation) instead of an oriented one.
            max_distance: Discard objects whose center is farther than this from
                the origin (meters). 0 disables the filter.
            max_obstacle_width: Clamp X/Y size to this value (meters). 0 disables.

        Returns:
            List of Object instances with world-frame pointclouds.
        """
        objects: list[Object] = []
        frame_id = world_pointcloud.frame_id

        # Project the full cloud into the camera ONCE — the projection is
        # detection-independent, and redoing it per detection made ingestion
        # O(detections x cloud size) per frame.
        projected = ProjectedCloud.project(world_pointcloud, camera_info, world_to_optical_transform)
        if projected is None:
            return []

        for det in detections_2d.detections:
            det3d = Detection3DPC.from_2d(
                det=det,
                world_pointcloud=world_pointcloud,
                camera_info=camera_info,
                world_to_optical_transform=world_to_optical_transform,
                filters=filters,
                projected=projected,
            )
            if det3d is None:
                continue

            pc = det3d.pointcloud
            if len(pc.pointcloud.points) < 3:
                continue

            # Compute bounding box. OBB can fail on near-coplanar/degenerate
            # sparse lidar clusters; _bounding_box falls back to AABB when it does.
            center, (sx, sy, sz), orientation = _bounding_box(pc.pointcloud, use_aabb=use_aabb)

            if max_obstacle_width > 0:
                sx = min(sx, max_obstacle_width)
                sy = min(sy, max_obstacle_width)
            size = Vector3(sx, sy, sz)

            if max_distance > 0:
                dist = (center.x**2 + center.y**2 + center.z**2) ** 0.5
                if dist > max_distance:
                    continue

            pose = PoseStamped(
                ts=det.ts,
                frame_id=frame_id,
                position=center,
                orientation=orientation,
            )

            store_mask = det.mask if isinstance(det, Detection2DSeg) else None

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
                    camera_transform=world_to_optical_transform,
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

    for obj in objects:
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
