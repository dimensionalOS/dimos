# Copyright 2026 Dimensional Inc.
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

"""Lidar coloring module.

Subscribes to camera_info, color_image and lidar streams. On every lidar
frame, projects the points into the camera image, samples nearest-neighbour
colors, and emits a colored ``PointCloud2`` on the ``colored_lidar`` output.
Also renders the camera frustum (4 corner rays) into rerun for visual
sanity-check.

Frame handling:
- The static ``T_camera_optical <- <lidar_frame>`` extrinsic is looked up via
  ``self.tf.get(...)`` at the lidar timestamp. If TF can't provide it, the
  frame is dropped (with a rate-limited warning).
- Output ``PointCloud2`` is published in the *same* frame as the input
  (positions untouched), only with a ``colors`` channel added.

This module uses ``rr.log`` *directly*, not through the dimos→rerun bridge.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
import rerun as rr

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.pointcloud.projection import Camera
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()


class Config(ModuleConfig):
    ray_length: float = 2.0
    frustum_color: tuple[int, int, int] = (255, 200, 0)
    entity_path: str = "world/lidar_color/frustum"
    # Connect to the same rerun gRPC server the vis bridge runs.
    # Set to None to skip the connect (e.g. when running standalone).
    rerun_grpc_url: str | None = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"
    # TF lookup tolerance for the static camera<-lidar extrinsic. Static so
    # this can be generous; we just need any sample.
    tf_tolerance: float = 1.0
    # Camera optical frame name we look up the extrinsic against. The bridge
    # / static_tf publisher should publish ``camera_optical -> <lidar_frame>``.
    camera_optical_frame: str = "camera_optical"


def color_pointcloud(
    points_lidar: np.ndarray,
    image: Image,
    camera_info: CameraInfo,
    T_camera_lidar: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Pure projection + nearest-neighbour color sampling.

    Args:
        points_lidar: ``(N, 3)`` float points in the lidar frame.
        image: latest ``Image`` to sample from. BGR is converted to RGB.
        camera_info: intrinsics + distortion model.
        T_camera_lidar: ``(4, 4)`` SE(3) such that ``p_cam = T @ p_lidar``.

    Returns:
        ``(valid_positions, valid_colors)`` — only the points that landed in
        front of the camera *and* inside the image. Positions are returned in
        the *original lidar frame* (unchanged from input). Colors are ``uint8``
        ``(M, 3)`` RGB.
    """
    if points_lidar.shape[0] == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)

    # Lidar -> camera optical frame.
    R = T_camera_lidar[:3, :3]
    t = T_camera_lidar[:3, 3:4]
    pts_cam = (R @ points_lidar.T + t).T  # (N, 3)

    # Project. Camera with identity pose because pts are already in cam frame.
    cam = Camera(camera_info, Pose())
    pixels, valid = cam.project(pts_cam)

    if not valid.any():
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)

    # Sample image at valid pixels (nearest-neighbour).
    data = image.data[..., ::-1] if image.format == ImageFormat.BGR else image.data
    uv = np.round(pixels[valid]).astype(np.int64)
    # The valid mask already excluded out-of-frame pixels, but the rounding
    # can push a pixel at u=W-0.4 to u=W. Clamp defensively.
    H, W = data.shape[:2]
    uv[:, 0] = np.clip(uv[:, 0], 0, W - 1)
    uv[:, 1] = np.clip(uv[:, 1], 0, H - 1)
    rgb = data[uv[:, 1], uv[:, 0]]  # (M, 3) uint8

    positions = points_lidar[valid].astype(np.float32)
    return positions, rgb.astype(np.uint8)


class LidarColorModule(Module):
    """Colors lidar points using the latest camera image + CameraInfo + TF.

    Outputs a colored ``PointCloud2`` on every incoming lidar frame, in the
    original lidar frame. Also renders the camera frustum into rerun.
    """

    config: Config

    camera_info: In[CameraInfo]
    color_image: In[Image]
    lidar: In[PointCloud2]

    colored_lidar: Out[PointCloud2]

    # Latest cached inputs. None until the corresponding stream produces one.
    _latest_camera_info: CameraInfo | None = None
    _latest_image: Image | None = None
    # Frustum endpoints in camera optical frame (depends only on CameraInfo).
    _corner_endpoints: np.ndarray | None = None

    # Counters for rate-limited logging.
    _info_seen: int = 0
    _images_seen: int = 0
    _lidars_seen: int = 0
    _frustums_logged: int = 0
    _colored_published: int = 0
    _tf_misses: int = 0

    @rpc
    def start(self) -> None:
        super().start()

        # Each worker process has its own rerun recording stream. Connect to
        # the bridge's gRPC server so our rr.log calls reach the same viewer.
        # Deliberately NOT calling rerun.init.rerun_init: it re-logs a static
        # AnnotationContext at "/" (the turbo colormap) and overwrites the
        # bridge's, breaking height-based pointcloud coloring.
        if self.config.rerun_grpc_url:
            try:
                rr.init("dimos")
                rr.connect_grpc(url=self.config.rerun_grpc_url)
                logger.info(
                    "LidarColorModule connected to rerun gRPC",
                    url=self.config.rerun_grpc_url,
                )
            except Exception as e:
                logger.warning(
                    "LidarColorModule failed to connect to rerun",
                    url=self.config.rerun_grpc_url,
                    error=str(e),
                )

        self.camera_info.observable().subscribe(self._on_camera_info)
        self.color_image.observable().subscribe(self._on_image)
        self.lidar.observable().subscribe(self._on_lidar)
        logger.info("LidarColorModule subscribed to camera_info + color_image + lidar")

    # --- subscribers ---

    def _on_camera_info(self, info: CameraInfo) -> None:
        """Cache the latest intrinsics; recompute frustum corner rays."""
        self._latest_camera_info = info
        W, H = info.width, info.height
        corners = np.array(
            [[0.0, 0.0], [W - 1.0, 0.0], [W - 1.0, H - 1.0], [0.0, H - 1.0]],
            dtype=np.float64,
        )
        cam = Camera(info, Pose())  # identity pose -> rays are in camera frame
        _, dirs = cam.unproject(corners)
        self._corner_endpoints = dirs * self.config.ray_length

        self._info_seen += 1
        if self._info_seen == 1 or self._info_seen % 100 == 0:
            logger.info(
                "LidarColorModule got camera_info",
                count=self._info_seen,
                width=W,
                height=H,
                model=info.distortion_model,
            )

    def _on_image(self, image: Image) -> None:
        """Cache the latest image (used by both frustum render and lidar coloring)."""
        self._latest_image = image
        self._images_seen += 1
        self._render_frustum(image)

    def _on_lidar(self, lidar: PointCloud2) -> None:
        """Project lidar into the image, sample colors, publish colored cloud."""
        self._lidars_seen += 1

        if self._latest_camera_info is None or self._latest_image is None:
            if self._lidars_seen == 1 or self._lidars_seen % 30 == 0:
                logger.info(
                    "LidarColorModule waiting for image+camera_info before coloring",
                    lidars_seen=self._lidars_seen,
                    has_image=self._latest_image is not None,
                    has_camera_info=self._latest_camera_info is not None,
                )
            return

        tf = self.tf.get(
            self.config.camera_optical_frame,
            lidar.frame_id,
            lidar.ts,
            self.config.tf_tolerance,
        )
        if tf is None:
            self._tf_misses += 1
            if self._tf_misses == 1 or self._tf_misses % 30 == 0:
                logger.warning(
                    "LidarColorModule TF lookup failed; dropping frame",
                    parent=self.config.camera_optical_frame,
                    child=lidar.frame_id,
                    ts=lidar.ts,
                    misses=self._tf_misses,
                )
            return

        points = lidar.points_f32()
        if points.shape[0] == 0:
            return

        positions, colors = color_pointcloud(
            points_lidar=points,
            image=self._latest_image,
            camera_info=self._latest_camera_info,
            T_camera_lidar=tf.to_matrix(),
        )

        if positions.shape[0] == 0:
            # All points outside the camera FOV — nothing to publish.
            return

        # Build a tensor PointCloud2 with positions and colors (0..1 floats).
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(positions, dtype=o3c.float32)
        pcd_t.point["colors"] = o3c.Tensor((colors.astype(np.float32) / 255.0), dtype=o3c.float32)
        colored = PointCloud2(pointcloud=pcd_t, ts=lidar.ts, frame_id=lidar.frame_id)
        self.colored_lidar.publish(colored)

        self._colored_published += 1
        if self._colored_published == 1 or self._colored_published % 50 == 0:
            logger.info(
                "LidarColorModule published colored cloud",
                count=self._colored_published,
                points_in=int(points.shape[0]),
                points_out=int(positions.shape[0]),
                frame_id=lidar.frame_id,
            )

    # --- rerun rendering (debug aid) ---

    def _render_frustum(self, image: Image) -> None:
        """Log the four corner rays parented to the camera optical TF frame."""
        endpoints = self._corner_endpoints
        if endpoints is None:
            if self._images_seen == 1 or self._images_seen % 30 == 0:
                logger.info(
                    "LidarColorModule waiting for camera_info before rendering frustum",
                    images_seen=self._images_seen,
                )
            return

        origin = [0.0, 0.0, 0.0]
        line_strips = [[origin, endpoint.tolist()] for endpoint in endpoints]

        rr.log(
            self.config.entity_path,
            rr.LineStrips3D(line_strips, colors=[self.config.frustum_color], radii=0.01),
        )
        rr.log(self.config.entity_path, rr.Transform3D(parent_frame="tf#/camera_optical"))

        self._frustums_logged += 1
        if self._frustums_logged == 1 or self._frustums_logged % 50 == 0:
            logger.info(
                "LidarColorModule logged frustum",
                count=self._frustums_logged,
                entity=self.config.entity_path,
            )
