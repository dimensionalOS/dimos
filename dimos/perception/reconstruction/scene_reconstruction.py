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

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.reconstruction_msgs.ReconstructionStatus import ReconstructionStatus
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class SceneReconstructionModuleConfig(ModuleConfig):
    target_frame: str = "world"
    workspace_center: tuple[float, float, float] = (0.45, 0.0, 0.18)
    workspace_size: float = 0.3
    resolution: int = 40
    truncation_distance: float | None = None
    reconstruction_fps: float = 2.0
    depth_trunc: float = 2.0
    depth16_scale: float = 0.001
    tf_time_tolerance: float = 0.1


class SceneReconstructionModule(Module):
    """Integrate depth images into a TSDF and reconstructed scene pointcloud."""

    config: SceneReconstructionModuleConfig

    depth_image: In[Image]
    depth_camera_info: In[CameraInfo]

    scene_pointcloud: Out[PointCloud2]
    tsdf: Out[TSDFGrid]
    status: Out[ReconstructionStatus]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._camera_info: CameraInfo | None = None
        self._workspace_origin = _origin_from_center(
            self.config.workspace_center,
            self.config.workspace_size,
        )
        self._integrated_frames = 0
        self._dropped_frames = 0
        self._latest_integration_ts: float | None = None
        self._last_error = ""
        self._paused = False
        self._active = True
        self._last_publish_ts: float | None = None
        self._volume = self._new_volume()

    @rpc
    def start(self) -> None:
        super().start()
        self._active = True
        self.register_disposable(
            Disposable(self.depth_camera_info.subscribe(self._set_camera_info))
        )
        self.register_disposable(Disposable(self.depth_image.subscribe(self._process_depth_image)))

    @rpc
    def stop(self) -> None:
        self._active = False
        super().stop()

    @rpc
    def reset_scene(self) -> str:
        """Clear integrated scene data and reset reconstruction counters."""
        self._volume = self._new_volume()
        self._integrated_frames = 0
        self._dropped_frames = 0
        self._latest_integration_ts = None
        self._last_error = ""
        self._last_publish_ts = None
        self._publish_outputs(time.time())
        return "Scene reconstruction reset"

    @rpc
    def pause_integration(self) -> str:
        """Pause depth integration without clearing the current reconstruction."""
        self._paused = True
        self._publish_status(time.time())
        return "Scene reconstruction paused"

    @rpc
    def resume_integration(self) -> str:
        """Resume depth integration."""
        self._paused = False
        self._publish_status(time.time())
        return "Scene reconstruction resumed"

    @rpc
    def set_workspace(
        self,
        center_x: float,
        center_y: float,
        center_z: float,
        size: float | None = None,
    ) -> str:
        """Set cubic workspace by center; internally stores a min-corner origin."""
        workspace_size = float(size if size is not None else self.config.workspace_size)
        if workspace_size <= 0:
            raise ValueError("workspace size must be positive")
        self._workspace_origin = _origin_from_center((center_x, center_y, center_z), workspace_size)
        self.config.workspace_size = workspace_size
        self._volume = self._new_volume()
        self._integrated_frames = 0
        self._latest_integration_ts = None
        self._last_publish_ts = None
        self._publish_outputs(time.time())
        return "Scene reconstruction workspace updated"

    @rpc
    def snapshot_scene(self) -> str:
        """Publish the current reconstructed pointcloud, TSDF, and status streams."""
        self._publish_outputs(time.time())
        return "Scene reconstruction snapshot published"

    @rpc
    def get_reconstruction_status(self) -> ReconstructionStatus:
        """Return current reconstruction status metadata."""
        return self._make_status(time.time())

    @property
    def workspace_origin(self) -> Vector3:
        return self._workspace_origin

    def _set_camera_info(self, camera_info: CameraInfo) -> None:
        self._camera_info = camera_info

    def _process_depth_image(self, depth_image: Image) -> None:
        if not self._active or self._paused:
            return
        if self._camera_info is None:
            self._drop_frame("missing depth camera info")
            return
        if not _camera_info_matches_depth(self._camera_info, depth_image):
            self._drop_frame("depth image does not match camera info")
            return

        camera_transform = self._lookup_camera_transform(depth_image)
        if camera_transform is None:
            self._drop_frame("missing transform")
            return

        try:
            self._integrate_depth(depth_image, self._camera_info, camera_transform)
        except (RuntimeError, ValueError) as exc:
            self._drop_frame(str(exc))
            logger.warning("Failed to integrate depth frame: %s", exc)
            return

        self._integrated_frames += 1
        self._latest_integration_ts = depth_image.ts
        self._last_error = ""

        if self._should_publish(depth_image.ts):
            self._publish_outputs(depth_image.ts)

    def _lookup_camera_transform(self, depth_image: Image) -> Transform | None:
        if self.config.target_frame == depth_image.frame_id:
            return Transform(
                frame_id=self.config.target_frame,
                child_frame_id=depth_image.frame_id,
                ts=depth_image.ts,
            )
        return self.tf.get(
            self.config.target_frame,
            depth_image.frame_id,
            depth_image.ts,
            self.config.tf_time_tolerance,
        )

    def _integrate_depth(
        self,
        depth_image: Image,
        camera_info: CameraInfo,
        target_from_camera: Transform,
    ) -> None:
        depth_m = _depth_to_meters(depth_image, self.config.depth16_scale)
        if depth_m.shape != (camera_info.height, camera_info.width):
            raise ValueError(
                f"depth image shape {depth_m.shape} does not match camera info "
                f"{camera_info.height}x{camera_info.width}"
            )

        valid_depth = np.isfinite(depth_m) & (depth_m > 0.0)
        if not np.any(valid_depth):
            raise ValueError("depth image contains no valid depth pixels")
        depth_filtered = depth_m.copy()
        depth_filtered[~valid_depth] = 0.0

        color = np.zeros((*depth_filtered.shape, 3), dtype=np.uint8)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color),
            o3d.geometry.Image(np.ascontiguousarray(depth_filtered.astype(np.float32))),
            depth_scale=1.0,
            depth_trunc=self.config.depth_trunc,
            convert_rgb_to_intensity=False,
        )
        intrinsic = _make_intrinsic(camera_info)
        self._volume.integrate(
            rgbd, intrinsic, self._camera_from_workspace_matrix(target_from_camera)
        )

    def _new_volume(self) -> o3d.pipelines.integration.UniformTSDFVolume:
        resolution = int(self.config.resolution)
        if resolution <= 0:
            raise ValueError("TSDF resolution must be positive")
        workspace_size = float(self.config.workspace_size)
        truncation = self.config.truncation_distance
        truncation_distance = (
            float(truncation) if truncation is not None else 4.0 * workspace_size / resolution
        )
        return o3d.pipelines.integration.UniformTSDFVolume(
            length=workspace_size,
            resolution=resolution,
            sdf_trunc=truncation_distance,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        )

    def _camera_from_workspace_matrix(self, target_from_camera: Transform) -> NDArray[np.float64]:
        target_from_workspace = np.eye(4, dtype=np.float64)
        target_from_workspace[:3, 3] = [
            self._workspace_origin.x,
            self._workspace_origin.y,
            self._workspace_origin.z,
        ]
        camera_from_target = np.linalg.inv(target_from_camera.to_matrix()).astype(np.float64)
        return camera_from_target @ target_from_workspace

    def _extract_tsdf_grid(self, ts: float) -> TSDFGrid:
        raw = np.asarray(self._volume.extract_volume_tsdf(), dtype=np.float32)
        resolution = int(self.config.resolution)
        expected = resolution * resolution * resolution
        if raw.shape != (expected, 2):
            raise RuntimeError(f"Open3D returned unexpected TSDF shape {raw.shape}")
        field = raw[:, 0].reshape((1, resolution, resolution, resolution)).astype(np.float32)
        weights = raw[:, 1].reshape((resolution, resolution, resolution)).astype(np.float32)
        distances = np.where(
            weights.reshape((1, resolution, resolution, resolution)) > 0.0, field, 1.0
        )
        return TSDFGrid(
            distances=distances,
            voxel_size=float(self.config.workspace_size) / resolution,
            truncation_distance=self._truncation_distance,
            origin=self._workspace_origin,
            weights=weights,
            frame_id=self.config.target_frame,
            ts=ts,
        )

    @property
    def _truncation_distance(self) -> float:
        if self.config.truncation_distance is not None:
            return float(self.config.truncation_distance)
        return 4.0 * float(self.config.workspace_size) / int(self.config.resolution)

    def _extract_scene_pointcloud(self, ts: float) -> PointCloud2:
        pointcloud = self._volume.extract_point_cloud()
        if len(pointcloud.points) > 0:
            translation = np.eye(4, dtype=np.float64)
            translation[:3, 3] = [
                self._workspace_origin.x,
                self._workspace_origin.y,
                self._workspace_origin.z,
            ]
            pointcloud.transform(translation)
        return PointCloud2(pointcloud=pointcloud, frame_id=self.config.target_frame, ts=ts)

    def _publish_outputs(self, ts: float) -> None:
        self.scene_pointcloud.publish(self._extract_scene_pointcloud(ts))
        self.tsdf.publish(self._extract_tsdf_grid(ts))
        self._publish_status(ts)
        self._last_publish_ts = ts

    def _publish_status(self, ts: float) -> None:
        self.status.publish(self._make_status(ts))

    def _make_status(self, ts: float) -> ReconstructionStatus:
        return ReconstructionStatus(
            integrated_frames=self._integrated_frames,
            dropped_frames=self._dropped_frames,
            last_error=self._last_error,
            active=self._active,
            paused=self._paused,
            latest_integration_ts=self._latest_integration_ts,
            workspace_origin=self._workspace_origin,
            workspace_size=self.config.workspace_size,
            frame_id=self.config.target_frame,
            ts=ts,
        )

    def _drop_frame(self, reason: str) -> None:
        self._dropped_frames += 1
        self._last_error = reason
        self._publish_status(time.time())

    def _should_publish(self, ts: float) -> bool:
        if self._last_publish_ts is None:
            return True
        fps = float(self.config.reconstruction_fps)
        if fps <= 0.0:
            return False
        return ts - self._last_publish_ts >= 1.0 / fps


def _origin_from_center(center: tuple[float, float, float], size: float) -> Vector3:
    half = float(size) / 2.0
    return Vector3(float(center[0]) - half, float(center[1]) - half, float(center[2]) - half)


def _camera_info_matches_depth(camera_info: CameraInfo, depth_image: Image) -> bool:
    return depth_image.data.shape[:2] == (camera_info.height, camera_info.width)


def _depth_to_meters(depth_image: Image, depth16_scale: float) -> NDArray[np.float32]:
    depth_obj: object = depth_image.to_opencv()
    if not isinstance(depth_obj, np.ndarray) and hasattr(depth_obj, "get"):
        depth_obj = depth_obj.get()
    arr = np.asarray(depth_obj)
    if depth_image.format == ImageFormat.DEPTH16 or arr.dtype == np.uint16:
        return np.ascontiguousarray(arr.astype(np.float32) * np.float32(depth16_scale))
    return np.ascontiguousarray(arr.astype(np.float32))


def _make_intrinsic(camera_info: CameraInfo) -> o3d.camera.PinholeCameraIntrinsic:
    intrinsic = camera_info.get_K_matrix()
    return o3d.camera.PinholeCameraIntrinsic(
        width=camera_info.width,
        height=camera_info.height,
        fx=float(intrinsic[0, 0]),
        fy=float(intrinsic[1, 1]),
        cx=float(intrinsic[0, 2]),
        cy=float(intrinsic[1, 2]),
    )
