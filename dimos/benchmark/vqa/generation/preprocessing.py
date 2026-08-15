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

# Copyright 2026 Dimensional Inc.
"""Prepare calibrated, rectified VQA frames from Go2 Memory2 recordings."""

from __future__ import annotations

from types import TracebackType
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation
    from dimos.protocol.tf.tf import TFLookup


class Go2FramePreprocessor:
    """Reuse recording streams and calibration while preparing sampled frames."""

    def __init__(self, recording: str, tolerance_s: float = 0.25) -> None:
        if not recording or tolerance_s <= 0:
            raise ValueError("recording must be set and tolerance_s must be positive")
        self._store = SqliteStore(path=recording, must_exist=True)
        self._tolerance_s = tolerance_s
        self._images: Any | None = None
        self._lidar: Any | None = None
        self._camera_info: Any | None = None
        self._tf: TFLookup | None = None
        self._rectifier = _ImageRectifier()
        self._started = False

    def __enter__(self) -> Go2FramePreprocessor:
        return self.start()

    def start(self) -> Go2FramePreprocessor:
        """Open the recording and cache its reusable stream handles."""
        if self._started:
            return self
        self._store.start()
        try:
            streams = set(self._store.list_streams())
            lidar_name = "pointlio_lidar" if "pointlio_lidar" in streams else "lidar"
            self._images = self._store.stream("color_image", Image).order_by("ts")
            self._lidar = self._store.stream(lidar_name, PointCloud2).order_by("ts")
            self._camera_info = (
                self._store.stream("camera_info", CameraInfo).order_by("ts")
                if "camera_info" in streams
                else None
            )
            self._tf = StreamTF.from_store(self._store)
            self._started = True
        except BaseException:
            self._store.stop()
            raise
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.stop()

    def stop(self) -> None:
        """Close the recording after all requested frames are prepared."""
        if not self._started:
            return
        self._store.stop()
        self._started = False
        self._images = None
        self._lidar = None
        self._camera_info = None
        self._tf = None

    def load(self, frame_index: int) -> CalibratedFrame:
        """Return one aligned, calibrated frame without running object perception."""
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        if self._images is None or self._lidar is None:
            raise RuntimeError("frame preprocessor must be used as a context manager")

        image_query = self._images.offset(frame_index).limit(1)
        image_obs = image_query.first()
        lidar_obs = _align_one(image_query, self._lidar, self._tolerance_s)
        source_camera_info = (
            _align_one(image_query, self._camera_info, self._tolerance_s).data
            if self._camera_info is not None
            else GO2Connection.camera_info_static
        )
        image, camera_info = self._rectifier.rectify(image_obs.data, source_camera_info)
        pointcloud_to_camera = _resolve_pointcloud_to_camera(
            image_obs,
            lidar_obs,
            camera_info.frame_id,
            self._tf,
            self._tolerance_s,
        )
        return CalibratedFrame(
            id=f"go2-{frame_index}",
            image=image,
            pointcloud=lidar_obs.data,
            camera_info=camera_info,
            pointcloud_to_camera=pointcloud_to_camera,
            image_is_rectified=True,
        )


class _ImageRectifier:
    """Cache rectification maps for stable recording camera calibration."""

    def __init__(self) -> None:
        self._maps: dict[tuple[object, ...], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}

    def rectify(self, image: Image, source: CameraInfo) -> tuple[Image, CameraInfo]:
        if (source.width, source.height) != (image.width, image.height):
            raise ValueError("camera calibration dimensions must match the image")
        key = (
            image.width,
            image.height,
            source.distortion_model,
            tuple(source.K),
            tuple(source.D),
            tuple(source.R),
            tuple(source.P),
        )
        maps = self._maps.get(key)
        if maps is None:
            maps = _rectification_maps(image, source)
            self._maps[key] = maps
        map_x, map_y, output_matrix = maps

        import cv2

        data = cv2.remap(image.data, map_x, map_y, interpolation=cv2.INTER_LINEAR)
        frame_id = source.frame_id or image.frame_id or "camera_optical"
        camera_info = CameraInfo.from_intrinsics(
            output_matrix[0, 0],
            output_matrix[1, 1],
            output_matrix[0, 2],
            output_matrix[1, 2],
            image.width,
            image.height,
            frame_id=frame_id,
        )
        camera_info.ts = image.ts
        return Image(data=data, format=image.format, frame_id=frame_id, ts=image.ts), camera_info


def _align_one(primary: Any, secondary: Any, tolerance_s: float) -> Observation[Any]:
    """Return the nearest secondary observation aligned to one primary observation."""
    try:
        return cast(
            "Observation[Any]", primary.align(secondary, tolerance=tolerance_s).first().data[1]
        )
    except LookupError as exc:
        raise ValueError("recording has no synchronized observation within tolerance") from exc


def _resolve_pointcloud_to_camera(
    image_obs: Observation[Image],
    lidar_obs: Observation[PointCloud2],
    camera_frame: str,
    tf: TFLookup | None,
    tolerance_s: float,
) -> Transform:
    """Resolve camera <- point-cloud from recorded TF or captured observation poses."""
    cloud_frame = lidar_obs.data.frame_id
    if not cloud_frame:
        raise ValueError("recorded point cloud requires a frame_id")
    if tf is not None:
        transform = tf.get(
            camera_frame,
            cloud_frame,
            time_point=image_obs.ts,
            time_tolerance=tolerance_s,
        )
        if transform is not None:
            return transform

    robot_pose = image_obs.pose
    if robot_pose is None:
        raise ValueError("recording requires robot pose or a complete recorded TF chain")
    world_from_camera = Transform.from_pose("base_link", robot_pose) + BASE_TO_OPTICAL
    camera_from_world = -world_from_camera
    if cloud_frame == "world":
        return camera_from_world
    cloud_pose = lidar_obs.pose
    if cloud_pose is None:
        raise ValueError("recording requires point-cloud pose or a complete recorded TF chain")
    world_from_cloud = Transform(
        translation=cloud_pose.position,
        rotation=cloud_pose.orientation,
        frame_id="world",
        child_frame_id=cloud_frame,
        ts=lidar_obs.ts,
    )
    return camera_from_world + world_from_cloud


def _rectification_maps(
    image: Image, source: CameraInfo
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    import cv2

    matrix = np.asarray(source.K, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(source.D, dtype=np.float64)
    rectification = np.asarray(source.R, dtype=np.float64).reshape(3, 3)
    projection = np.asarray(source.P, dtype=np.float64).reshape(3, 4)[:, :3]
    output_matrix = projection if projection[0, 0] > 0 and projection[1, 1] > 0 else matrix
    size = (image.width, image.height)
    model = source.distortion_model.strip().lower()
    if model in ("equidistant", "fisheye", "kannala_brandt"):
        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
            matrix, distortion, rectification, output_matrix, size, cv2.CV_32FC1
        )
    elif model in ("", "plumb_bob", "rational_polynomial"):
        map_x, map_y = cv2.initUndistortRectifyMap(
            matrix, distortion, rectification, output_matrix, size, cv2.CV_32FC1
        )
    else:
        raise ValueError(f"unsupported camera distortion model: {source.distortion_model}")
    return map_x, map_y, output_matrix
