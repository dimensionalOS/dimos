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

"""Prepare image-aligned point-cloud frames from recorded VQA datasets."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from typing_extensions import Self

from dimos.memory.cli.dataset import open_dataset
from dimos.memory.store.base import Store
from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation
    from dimos.protocol.tf.tf import TFLookup


@dataclass(frozen=True)
class PointCloudFrame:
    """One rectified image and synchronized point cloud ready for VQA primitives."""

    index: int
    image: Image
    pointcloud: PointCloud2
    camera_info: CameraInfo
    pointcloud_to_camera: Transform
    image_observation_timestamp: float
    pointcloud_observation_timestamp: float
    calibration_source: str

    @property
    def synchronization_delta_s(self) -> float:
        return abs(self.image_observation_timestamp - self.pointcloud_observation_timestamp)


class PointCloudFrameUnavailableError(ValueError):
    """A selected image lacks the recorded evidence needed for metric geometry."""


class SynchronizedPointCloudNotFoundError(PointCloudFrameUnavailableError):
    """No point cloud is close enough to the selected image observation."""


class PointCloudFrameLoader:
    """Load image-aligned point-cloud frames from one Memory dataset."""

    def __init__(
        self,
        dataset: str | Path,
        tolerance_s: float = 0.1,
    ) -> None:
        if not dataset:
            raise ValueError("dataset must be set")
        if tolerance_s <= 0:
            raise ValueError("tolerance_s must be positive")
        self._dataset = dataset
        self._tolerance_s = tolerance_s
        self._store: Store | None = None
        self._images: Stream[Image] | None = None
        self._lidar: Stream[PointCloud2] | None = None
        self._recorded_camera_info: Stream[CameraInfo] | None = None
        self._recorded_tf: TFLookup | None = None
        self._rectifier = _ImageRectifier()

    def __enter__(self) -> Self:
        return self.start()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.stop()

    def start(self) -> PointCloudFrameLoader:
        """Open the dataset and cache all stream and calibration handles."""
        if self._store is not None:
            return self

        store = open_dataset(self._dataset)
        store.start()
        try:
            streams = set(store.list_streams())
            missing = {"color_image", "camera_info", "tf"} - streams
            if missing:
                listing = ", ".join(sorted(missing))
                raise ValueError(f"dataset is missing required stream(s): {listing}")
            lidar_names = [name for name in ("pointlio_lidar", "lidar") if name in streams]
            if not lidar_names:
                raise ValueError("dataset has no 'pointlio_lidar' or 'lidar' stream")

            images = store.stream("color_image", Image).order_by("ts")
            try:
                images.first()
            except LookupError as exc:
                raise ValueError("required color_image stream is empty") from exc
            self._images = images

            recorded_camera_info = store.stream("camera_info", CameraInfo).order_by("ts")
            try:
                recorded_camera_info.first()
            except LookupError as exc:
                raise ValueError("required camera_info stream is empty") from exc
            self._recorded_camera_info = recorded_camera_info

            recorded_tf = StreamTF.from_store(store)
            if recorded_tf is None:
                raise ValueError("required tf stream is unavailable")
            try:
                recorded_tf.stream.first()
            except LookupError as exc:
                raise ValueError("required tf stream is empty") from exc
            self._recorded_tf = recorded_tf

            for lidar_name in lidar_names:
                lidar = store.stream(lidar_name, PointCloud2).order_by("ts")
                try:
                    lidar.first()
                except LookupError:
                    continue
                self._lidar = lidar
                break
            if self._lidar is None:
                raise ValueError("required pointlio_lidar/lidar stream is empty")
            self._store = store
        except BaseException:
            store.stop()
            self._clear_state()
            raise
        return self

    def stop(self) -> None:
        """Close the dataset and release cached stream handles."""
        store = self._store
        self._clear_state()
        if store is not None:
            store.stop()

    @property
    def image_count(self) -> int:
        if self._images is None:
            raise RuntimeError(
                "point-cloud frame loader must be started before reading image_count"
            )
        return self._images.count()

    def _clear_state(self) -> None:
        self._store = None
        self._images = None
        self._lidar = None
        self._recorded_camera_info = None
        self._recorded_tf = None

    def load(self, frame_index: int) -> PointCloudFrame:
        """Load the indexed image and its nearest image-aligned point cloud."""
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        if self._images is None or self._lidar is None:
            raise RuntimeError("point-cloud frame loader must be started before loading frames")

        image_query = self._images.offset(frame_index).limit(1)
        image_obs = image_query.first()
        lidar_obs = _align_one(image_query, self._lidar, self._tolerance_s)
        source_image = image_obs.data.copy()
        source_image.ts = image_obs.ts

        recorded_camera_info = self._recorded_camera_info
        recorded_tf = self._recorded_tf
        if recorded_camera_info is None or recorded_tf is None:
            raise RuntimeError("recorded calibration was not initialized")
        source_camera_info = _camera_info_at(recorded_camera_info, image_obs.ts)
        image, camera_info = self._rectifier.rectify(source_image, source_camera_info)
        pointcloud_to_camera = _recorded_pointcloud_to_camera(
            image_obs,
            lidar_obs,
            camera_info.frame_id,
            recorded_tf,
            self._tolerance_s,
        )
        rectification = np.eye(4, dtype=np.float64)
        rectification[:3, :3] = np.asarray(source_camera_info.R, dtype=np.float64).reshape(3, 3)
        pointcloud_to_camera = Transform.from_matrix(
            rectification @ pointcloud_to_camera.to_matrix(),
            ts=image_obs.ts,
            frame_id=camera_info.frame_id,
            child_frame_id=lidar_obs.data.frame_id,
        )
        if not np.all(np.isfinite(pointcloud_to_camera.to_matrix())):
            raise PointCloudFrameUnavailableError(
                "recorded point-cloud-to-camera transform must be finite"
            )

        return PointCloudFrame(
            index=frame_index,
            image=image,
            pointcloud=lidar_obs.data,
            camera_info=camera_info,
            pointcloud_to_camera=pointcloud_to_camera,
            image_observation_timestamp=image_obs.ts,
            pointcloud_observation_timestamp=lidar_obs.ts,
            calibration_source="recorded",
        )

    def load_image(self, frame_index: int) -> Image:
        """Load a rectified image when synchronized point-cloud evidence is unavailable."""
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        if self._images is None:
            raise RuntimeError("point-cloud frame loader must be started before loading frames")
        observation = self._images.offset(frame_index).limit(1).first()
        image = observation.data.copy()
        image.ts = observation.ts
        recorded_camera_info = self._recorded_camera_info
        if recorded_camera_info is None:
            raise RuntimeError("camera calibration was not initialized")
        camera_info = _camera_info_at(recorded_camera_info, observation.ts)
        return self._rectifier.rectify(image, camera_info)[0]


class _ImageRectifier:
    """Rectify images while caching maps for static camera calibration."""

    def __init__(self) -> None:
        self._maps: dict[tuple[Any, ...], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}

    def rectify(self, image: Image, source: CameraInfo) -> tuple[Image, CameraInfo]:
        # OpenCV is intentionally lazy because importing it loads a large native library.
        import cv2

        if image.width <= 0 or image.height <= 0:
            raise ValueError("image dimensions must be positive")
        if (source.width, source.height) != (image.width, image.height):
            raise ValueError(
                "camera calibration dimensions "
                f"{source.width}x{source.height} do not match image dimensions "
                f"{image.width}x{image.height}"
            )
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
        data = cv2.remap(image.data, map_x, map_y, interpolation=cv2.INTER_LINEAR)
        frame_id = source.frame_id or image.frame_id
        if not frame_id:
            raise ValueError("camera calibration requires a camera frame_id")
        camera_info = CameraInfo.from_intrinsics(
            output_matrix[0, 0],
            output_matrix[1, 1],
            output_matrix[0, 2],
            output_matrix[1, 2],
            image.width,
            image.height,
            frame_id=frame_id,
        ).with_ts(image.ts)
        return Image(data=data, format=image.format, frame_id=frame_id, ts=image.ts), camera_info


def _align_one(
    primary: Stream[Any], secondary: Stream[Any], tolerance_s: float
) -> Observation[Any]:
    """Return the nearest secondary using observation/database timestamps."""
    try:
        pair = primary.align(secondary, tolerance=tolerance_s).first().data
    except LookupError as exc:
        raise SynchronizedPointCloudNotFoundError(
            "dataset has no synchronized point cloud within tolerance"
        ) from exc
    return cast("Observation[Any]", pair[1])


def _camera_info_at(stream: Stream[CameraInfo], timestamp: float) -> CameraInfo:
    """Return the latest recorded calibration applicable to a frame timestamp."""
    try:
        return stream.time_range(float("-inf"), timestamp).order_by("ts", desc=True).first().data
    except LookupError as exc:
        raise ValueError(
            f"dataset has no camera calibration at or before image timestamp {timestamp}"
        ) from exc


def _recorded_pointcloud_to_camera(
    image_obs: Observation[Image],
    lidar_obs: Observation[PointCloud2],
    camera_frame: str,
    tf: TFLookup,
    tolerance_s: float,
) -> Transform:
    cloud_frame = lidar_obs.data.frame_id
    if not cloud_frame:
        raise PointCloudFrameUnavailableError("recorded point cloud requires a frame_id")
    camera_from_cloud = tf.get(
        camera_frame,
        cloud_frame,
        time_point=image_obs.ts,
        time_tolerance=tolerance_s,
    )
    if camera_from_cloud is None:
        raise PointCloudFrameUnavailableError(
            f"recorded tf cannot resolve {camera_frame!r} <- {cloud_frame!r} "
            f"at image timestamp {image_obs.ts}"
        )
    return camera_from_cloud


def _rectification_maps(
    image: Image, source: CameraInfo
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # OpenCV is intentionally lazy because importing it loads a large native library.
    import cv2

    intrinsics = np.asarray(source.K, dtype=np.float64)
    if intrinsics.size != 9:
        raise ValueError("camera intrinsics must contain nine values")
    matrix = intrinsics.reshape(3, 3)
    if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera intrinsics must be finite with positive focal lengths")
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
