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

"""Canonical calibrated-frame boundary for point-cloud VQA generation."""

from __future__ import annotations

from collections.abc import Collection
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import TYPE_CHECKING, Any, Literal, Protocol, cast

import numpy as np

from dimos.memory.cli.dataset import open_dataset
from dimos.memory.store.base import Store
from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection

if TYPE_CHECKING:
    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation
    from dimos.protocol.tf.tf import TFLookup


@dataclass(frozen=True)
class CalibratedFrame:
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


class CalibratedFrameLoader(Protocol):
    """Prepare canonical frames without exposing recording details downstream."""

    def load(self, frame_index: int) -> CalibratedFrame: ...


CalibrationProfile = Literal["go2"]


def recorded_calibration_available(streams: Collection[str]) -> bool:
    """Return whether a recording contains the complete calibration stream pair."""
    has_camera_info = "camera_info" in streams
    has_tf = "tf" in streams
    if has_camera_info != has_tf:
        raise ValueError(
            "recording calibration is incomplete; camera_info and tf streams must both be present "
            "or both be absent"
        )
    return has_camera_info


class FrameGeometryUnavailableError(ValueError):
    """A selected image lacks the recorded evidence needed for metric geometry."""


class SynchronizedPointCloudNotFoundError(FrameGeometryUnavailableError):
    """No point cloud is close enough to the selected image observation."""


class RecordingFramePreprocessor:
    """Load synchronized, rectified frames from one open Memory recording."""

    def __init__(
        self,
        recording: str | Path,
        calibration_profile: CalibrationProfile | None = None,
        tolerance_s: float = 0.1,
    ) -> None:
        if not recording:
            raise ValueError("recording must be set")
        if calibration_profile not in (None, "go2"):
            raise ValueError(f"unsupported calibration profile: {calibration_profile!r}")
        if tolerance_s <= 0:
            raise ValueError("tolerance_s must be positive")
        self._recording = recording
        self._calibration_profile = calibration_profile
        self._tolerance_s = tolerance_s
        self._store: Store | None = None
        self._images: Stream[Image] | None = None
        self._lidar: Stream[PointCloud2] | None = None
        self._odom: Stream[PoseStamped] | None = None
        self._recorded_camera_info: CameraInfo | None = None
        self._recorded_tf: TFLookup | None = None
        self._calibration_source: str | None = None
        self._rectifier = _ImageRectifier()

    def __enter__(self) -> RecordingFramePreprocessor:
        return self.start()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.stop()

    def start(self) -> RecordingFramePreprocessor:
        """Open the recording and cache all stream and calibration handles."""
        if self._store is not None:
            return self

        store = open_dataset(self._recording)
        store.start()
        try:
            streams = set(store.list_streams())
            if "color_image" not in streams:
                raise ValueError("recording has no 'color_image' stream")
            has_recorded_calibration = recorded_calibration_available(streams)
            lidar_name = next(
                (name for name in ("pointlio_lidar", "lidar") if name in streams), None
            )
            if lidar_name is None:
                raise ValueError("recording has no 'pointlio_lidar' or 'lidar' stream")

            self._images = store.stream("color_image", Image).order_by("ts")
            self._lidar = store.stream(lidar_name, PointCloud2).order_by("ts")

            if has_recorded_calibration:
                try:
                    self._recorded_camera_info = (
                        store.stream("camera_info", CameraInfo).order_by("ts").first().data
                    )
                except LookupError as exc:
                    raise ValueError("recorded camera_info stream is empty") from exc
                self._recorded_tf = StreamTF.from_store(store)
                self._calibration_source = "recorded"
            elif self._calibration_profile == "go2":
                if "odom" not in streams:
                    raise ValueError("Go2 calibration profile requires an 'odom' stream")
                self._odom = store.stream("odom", PoseStamped).order_by("ts")
                self._calibration_source = "profile:go2"
            else:
                raise ValueError(
                    "recording has no camera_info or tf calibration streams; "
                    "set calibration_profile='go2' explicitly to use the Go2 profile"
                )
            self._store = store
        except BaseException:
            store.stop()
            self._clear_state()
            raise
        return self

    def stop(self) -> None:
        """Close the recording and release cached stream handles."""
        store = self._store
        self._clear_state()
        if store is not None:
            store.stop()

    @property
    def image_count(self) -> int:
        if self._images is None:
            raise RuntimeError("frame preprocessor must be started before reading image_count")
        return self._images.count()

    def _clear_state(self) -> None:
        self._store = None
        self._images = None
        self._lidar = None
        self._odom = None
        self._recorded_camera_info = None
        self._recorded_tf = None
        self._calibration_source = None

    def load(self, frame_index: int) -> CalibratedFrame:
        """Load the indexed image and its nearest calibrated point cloud."""
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        if self._images is None or self._lidar is None or self._calibration_source is None:
            raise RuntimeError("frame preprocessor must be started before loading frames")

        image_query = self._images.offset(frame_index).limit(1)
        image_obs = image_query.first()
        lidar_obs = _align_one(image_query, self._lidar, self._tolerance_s)
        source_image = image_obs.data.copy()
        source_image.ts = image_obs.ts

        if self._calibration_source == "recorded":
            source_camera_info = self._recorded_camera_info
            if source_camera_info is None or self._recorded_tf is None:
                raise RuntimeError("recorded calibration was not initialized")
            image, camera_info = self._rectifier.rectify(source_image, source_camera_info)
            pointcloud_to_camera = _recorded_pointcloud_to_camera(
                image_obs,
                lidar_obs,
                camera_info.frame_id,
                self._recorded_tf,
                self._tolerance_s,
            )
        else:
            if self._odom is None:
                raise RuntimeError("Go2 calibration profile was not initialized")
            try:
                odom_obs = cast(
                    "Observation[PoseStamped]",
                    image_query.align(self._odom, tolerance=self._tolerance_s).first().data[1],
                )
            except LookupError as exc:
                raise FrameGeometryUnavailableError(
                    "recording has no synchronized odometry within tolerance"
                ) from exc
            image, camera_info = self._rectifier.rectify(
                source_image, GO2Connection.camera_info_static
            )
            pointcloud_to_camera = _profile_pointcloud_to_camera(odom_obs, lidar_obs)

        return CalibratedFrame(
            index=frame_index,
            image=image,
            pointcloud=lidar_obs.data,
            camera_info=camera_info,
            pointcloud_to_camera=pointcloud_to_camera,
            image_observation_timestamp=image_obs.ts,
            pointcloud_observation_timestamp=lidar_obs.ts,
            calibration_source=self._calibration_source,
        )

    def load_image(self, frame_index: int) -> Image:
        """Load a rectified image when synchronized point-cloud evidence is unavailable."""
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        if self._images is None or self._calibration_source is None:
            raise RuntimeError("frame preprocessor must be started before loading frames")
        observation = self._images.offset(frame_index).limit(1).first()
        image = observation.data.copy()
        image.ts = observation.ts
        camera_info = (
            self._recorded_camera_info
            if self._calibration_source == "recorded"
            else GO2Connection.camera_info_static
        )
        if camera_info is None:
            raise RuntimeError("camera calibration was not initialized")
        return self._rectifier.rectify(image, camera_info)[0]


class _ImageRectifier:
    """Rectify images while caching maps for static camera calibration."""

    def __init__(self) -> None:
        self._maps: dict[tuple[Any, ...], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}

    def rectify(self, image: Image, source: CameraInfo) -> tuple[Image, CameraInfo]:
        # OpenCV is intentionally lazy because importing it loads a large native library.
        import cv2

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
            "recording has no synchronized point cloud within tolerance"
        ) from exc
    return cast("Observation[Any]", pair[1])


def _recorded_pointcloud_to_camera(
    image_obs: Observation[Image],
    lidar_obs: Observation[PointCloud2],
    camera_frame: str,
    tf: TFLookup,
    tolerance_s: float,
) -> Transform:
    cloud_frame = lidar_obs.data.frame_id
    if not cloud_frame:
        raise FrameGeometryUnavailableError("recorded point cloud requires a frame_id")
    camera_from_world = tf.get(
        camera_frame,
        "world",
        time_point=image_obs.ts,
        time_tolerance=tolerance_s,
    )
    if camera_from_world is None:
        raise FrameGeometryUnavailableError(
            f"recorded tf cannot resolve {camera_frame!r} <- 'world' "
            f"at image timestamp {image_obs.ts}"
        )
    if cloud_frame == "world":
        return camera_from_world
    world_from_cloud = tf.get(
        "world",
        cloud_frame,
        time_point=lidar_obs.ts,
        time_tolerance=tolerance_s,
    )
    if world_from_cloud is None:
        raise FrameGeometryUnavailableError(
            f"recorded tf cannot resolve 'world' <- {cloud_frame!r} "
            f"at point-cloud timestamp {lidar_obs.ts}"
        )
    return camera_from_world + world_from_cloud


def _profile_pointcloud_to_camera(
    odom_obs: Observation[PoseStamped], lidar_obs: Observation[PointCloud2]
) -> Transform:
    """Resolve the explicit Go2 profile from base odometry and its static mount."""
    odom = odom_obs.data
    if odom.frame_id != "world":
        raise FrameGeometryUnavailableError(
            "Go2 calibration profile requires odometry with frame_id='world'"
        )
    world_from_camera = Transform.from_pose("base_link", odom) + BASE_TO_OPTICAL
    camera_from_world = -world_from_camera
    cloud_frame = lidar_obs.data.frame_id
    if cloud_frame != "world":
        raise FrameGeometryUnavailableError(
            "Go2 calibration profile requires a world-frame point cloud"
        )
    return camera_from_world


def _rectification_maps(
    image: Image, source: CameraInfo
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # OpenCV is intentionally lazy because importing it loads a large native library.
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
