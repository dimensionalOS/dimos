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

"""Booster SDK Python RGB-D camera bridge."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
import struct
import threading
import time
from typing import Any, Protocol

import booster_robotics_sdk_python as booster
import cv2
import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.metrics_msgs.MetricsArray import MetricsArray
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.booster.b1.camera_config import (
    DEFAULT_COLOR_CAMERA_INFO_TOPIC,
    DEFAULT_COLOR_TOPIC,
    DEFAULT_DEPTH_CAMERA_INFO_TOPIC,
    DEFAULT_DEPTH_SCALE,
    DEFAULT_DEPTH_TOPIC,
    camera_network_interface_from_env,
)
from dimos.spec import perception
from dimos.teleop.utils.stream_stats import pcts
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class BoosterCameraPythonConfig(ModuleConfig):
    """Configuration for Booster camera DDS streams consumed by the Python SDK."""

    network_interface: str | None = Field(default_factory=camera_network_interface_from_env)
    depth_scale: float = Field(default=DEFAULT_DEPTH_SCALE, gt=0.0)
    metrics_interval_seconds: float = Field(default=1.0, gt=0.0)
    publish_rate_hz: float | None = Field(default=None, gt=0.0)
    color_compressed: bool = True
    depth_enabled: bool = False
    depth_compressed: bool = True
    color_topic: str = DEFAULT_COLOR_TOPIC
    depth_topic: str = DEFAULT_DEPTH_TOPIC
    color_camera_info_topic: str = DEFAULT_COLOR_CAMERA_INFO_TOPIC
    depth_camera_info_topic: str = DEFAULT_DEPTH_CAMERA_INFO_TOPIC


class _Subscriber(Protocol):
    def InitChannel(self) -> None: ...

    def CloseChannel(self) -> None: ...


class _PublishRateLimiter:
    """Thread-safe, non-blocking rate limiter for one image stream."""

    def __init__(self, rate_hz: float | None) -> None:
        self._minimum_interval = 0.0 if rate_hz is None else 1.0 / rate_hz
        self._next_allowed_at = 0.0
        self._lock = threading.Lock()

    def allow(self, now: float) -> bool:
        """Return whether a frame arriving at the monotonic time should publish."""
        if self._minimum_interval == 0.0:
            return True
        with self._lock:
            if now < self._next_allowed_at:
                return False
            self._next_allowed_at = now + self._minimum_interval
            return True


@dataclass(frozen=True)
class _FrameAgeSnapshot:
    window_seconds: float
    frames: int
    payload_bytes: int
    invalid_timestamps: int
    negative_ages: int
    frame_age_ms: dict[str, float] | None
    bridge_ms: dict[str, float] | None


class _FrameAgeMetrics:
    """Thread-safe frame-age and callback-processing metrics for one camera stream."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._window_started = time.perf_counter()
        self._frames = 0
        self._payload_bytes = 0
        self._invalid_timestamps = 0
        self._negative_ages = 0
        self._frame_age_ms: list[float] = []
        self._bridge_ms: list[float] = []

    def record(
        self,
        source_timestamp: float,
        processing_started: float,
        payload_bytes: int,
    ) -> None:
        """Record one frame after its output publish has completed."""
        published_at = time.time()
        processing_finished = time.perf_counter()
        processing_ms = (processing_finished - processing_started) * 1_000.0
        valid_timestamp = source_timestamp > 0.0 and math.isfinite(source_timestamp)
        frame_age_ms = (published_at - source_timestamp) * 1_000.0

        with self._lock:
            self._frames += 1
            self._payload_bytes += payload_bytes
            self._bridge_ms.append(processing_ms)
            if not valid_timestamp:
                self._invalid_timestamps += 1
                return
            self._frame_age_ms.append(frame_age_ms)
            if frame_age_ms < 0.0:
                self._negative_ages += 1

    def snapshot_and_reset(self) -> _FrameAgeSnapshot:
        """Return aggregates for the current window and start a fresh window."""
        now = time.perf_counter()
        with self._lock:
            window_seconds = now - self._window_started
            frames = self._frames
            payload_bytes = self._payload_bytes
            invalid_timestamps = self._invalid_timestamps
            negative_ages = self._negative_ages
            frame_age_ms = self._frame_age_ms
            bridge_ms = self._bridge_ms

            self._window_started = now
            self._frames = 0
            self._payload_bytes = 0
            self._invalid_timestamps = 0
            self._negative_ages = 0
            self._frame_age_ms = []
            self._bridge_ms = []

        return _FrameAgeSnapshot(
            window_seconds=window_seconds,
            frames=frames,
            payload_bytes=payload_bytes,
            invalid_timestamps=invalid_timestamps,
            negative_ages=negative_ages,
            frame_age_ms=pcts(frame_age_ms),
            bridge_ms=pcts(bridge_ms),
        )


def _timestamp(header: Any) -> float:
    seconds = int(header.stamp.sec)
    nanoseconds = int(header.stamp.nanosec)
    if not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError(f"invalid Booster timestamp nanoseconds: {nanoseconds}")
    return seconds + nanoseconds / 1_000_000_000


def _byte_array(data: Sequence[int] | bytes | bytearray | memoryview) -> NDArray[np.uint8]:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return np.frombuffer(data, dtype=np.uint8)
    return np.asarray(data, dtype=np.uint8)


def _unpack_rows(
    message: Any,
    dtype: np.dtype[Any],
    channels: int,
) -> NDArray[Any]:
    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid Booster image dimensions: {width}x{height}")

    packed_step = width * dtype.itemsize * channels
    source_step = int(message.step) or packed_step
    if source_step < packed_step:
        raise ValueError(
            f"Booster image step {source_step} is shorter than packed row {packed_step}"
        )

    source = _byte_array(message.data)
    required_size = height * source_step
    if source.size < required_size:
        raise ValueError(
            f"Booster image has {source.size} bytes, expected at least {required_size}"
        )

    packed = np.ascontiguousarray(
        source[:required_size].reshape(height, source_step)[:, :packed_step]
    )
    source_dtype = dtype.newbyteorder(">" if bool(message.is_bigendian) else "<")
    image = packed.view(source_dtype)
    shape = (height, width) if channels == 1 else (height, width, channels)
    return image.reshape(shape).astype(dtype, copy=False)


def booster_image_to_dimos(message: Any) -> Image:
    """Convert an uncompressed Booster color image to a DimOS image."""
    encoding = str(message.encoding).lower()
    specifications: dict[str, tuple[np.dtype[Any], int, ImageFormat]] = {
        "rgb8": (np.dtype(np.uint8), 3, ImageFormat.RGB),
        "bgr8": (np.dtype(np.uint8), 3, ImageFormat.BGR),
        "rgba8": (np.dtype(np.uint8), 4, ImageFormat.RGBA),
        "bgra8": (np.dtype(np.uint8), 4, ImageFormat.BGRA),
        "mono8": (np.dtype(np.uint8), 1, ImageFormat.GRAY),
        "mono16": (np.dtype(np.uint16), 1, ImageFormat.GRAY16),
    }
    try:
        dtype, channels, image_format = specifications[encoding]
    except KeyError:
        raise ValueError(f"unsupported Booster color encoding {message.encoding!r}") from None

    return Image(
        data=_unpack_rows(message, dtype, channels),
        format=image_format,
        frame_id=str(message.header.frame_id),
        ts=_timestamp(message.header),
    )


def booster_compressed_image_to_dimos(message: Any) -> Image:
    """Decode a Booster JPEG/PNG color image into a DimOS image."""
    encoded = _byte_array(message.data)
    decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if decoded is None:
        raise ValueError(f"failed to decode Booster compressed image format {message.format!r}")

    if decoded.ndim == 2:
        image_format = ImageFormat.GRAY16 if decoded.dtype == np.uint16 else ImageFormat.GRAY
    elif decoded.shape[2] == 3:
        image_format = ImageFormat.BGR
    elif decoded.shape[2] == 4:
        image_format = ImageFormat.BGRA
    else:
        raise ValueError(f"unsupported decoded Booster image shape {decoded.shape}")

    return Image(
        data=decoded,
        format=image_format,
        frame_id=str(message.header.frame_id),
        ts=_timestamp(message.header),
    )


def booster_depth_image_to_dimos(message: Any, depth_scale: float) -> Image:
    """Convert a Booster depth image to a float32 DimOS depth image in meters."""
    encoding = str(message.encoding).lower()
    if encoding == "16uc1":
        depth = _unpack_rows(message, np.dtype(np.uint16), 1).astype(np.float32)
        depth *= np.float32(depth_scale)
    elif encoding == "32fc1":
        depth = _unpack_rows(message, np.dtype(np.float32), 1)
    else:
        raise ValueError(f"unsupported Booster depth encoding {message.encoding!r}")

    return Image(
        data=depth,
        format=ImageFormat.DEPTH,
        frame_id=str(message.header.frame_id),
        ts=_timestamp(message.header),
    )


_COMPRESSED_DEPTH_HEADER = struct.Struct("=iff")


def booster_compressed_depth_image_to_dimos(message: Any, depth_scale: float) -> Image:
    """Decode a ROS compressedDepth PNG into float32 depth in meters."""
    format_parts = str(message.format).split(";", maxsplit=1)
    source_encoding = format_parts[0].strip().lower()
    transport_format = format_parts[1].strip() if len(format_parts) == 2 else ""
    if "compressedDepth" not in transport_format:
        raise ValueError(f"unsupported Booster compressed depth format {message.format!r}")
    if transport_format not in {"compressedDepth", "compressedDepth png"}:
        raise ValueError(f"unsupported Booster compressed depth format {message.format!r}")

    data = _byte_array(message.data)
    if data.size <= _COMPRESSED_DEPTH_HEADER.size:
        raise ValueError("Booster compressed depth payload is missing its PNG data")
    _, depth_quant_a, depth_quant_b = _COMPRESSED_DEPTH_HEADER.unpack_from(data)
    decoded = cv2.imdecode(
        data[_COMPRESSED_DEPTH_HEADER.size :],
        cv2.IMREAD_UNCHANGED,
    )
    if decoded is None or decoded.ndim != 2 or decoded.dtype != np.uint16:
        raise ValueError("Booster compressed depth payload is not a uint16 PNG")

    if source_encoding == "16uc1":
        depth = decoded.astype(np.float32)
        depth *= np.float32(depth_scale)
    elif source_encoding == "32fc1":
        inverse_depth = decoded.astype(np.float32)
        depth = np.full(decoded.shape, np.nan, dtype=np.float32)
        valid = decoded != 0
        depth[valid] = np.float32(depth_quant_a) / (
            inverse_depth[valid] - np.float32(depth_quant_b)
        )
    else:
        raise ValueError(f"unsupported Booster depth encoding {source_encoding!r}")

    return Image(
        data=depth,
        format=ImageFormat.DEPTH,
        frame_id=str(message.header.frame_id),
        ts=_timestamp(message.header),
    )


def booster_camera_info_to_dimos(message: Any) -> CameraInfo:
    """Convert Booster camera calibration and ROI data to DimOS."""
    result = CameraInfo(
        height=int(message.height),
        width=int(message.width),
        distortion_model=str(message.distortion_model),
        D=list(message.d),
        K=list(message.k),
        R=list(message.r),
        P=list(message.p),
        binning_x=int(message.binning_x),
        binning_y=int(message.binning_y),
        frame_id=str(message.header.frame_id),
        ts=_timestamp(message.header),
    )
    result.roi_x_offset = int(message.roi.x_offset)
    result.roi_y_offset = int(message.roi.y_offset)
    result.roi_height = int(message.roi.height)
    result.roi_width = int(message.roi.width)
    result.roi_do_rectify = bool(message.roi.do_rectify)
    return result


class BoosterCameraPython(Module, perception.DepthCamera):
    """RGB-D camera streamed through Booster's Python SDK."""

    dedicated_worker = True
    config: BoosterCameraPythonConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    camera_metrics: Out[MetricsArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._subscribers: list[_Subscriber] = []
        self._lifecycle_lock = threading.Lock()
        self._metrics_log_lock = threading.Lock()
        self._started = threading.Event()
        self._reset_metrics()
        self._reset_publish_rate_limiters()

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if self._started.is_set():
                return
            super().start()
            self._reset_metrics()
            self._reset_publish_rate_limiters()
            booster.ChannelFactory.Instance().Init(0, self.config.network_interface or "")

            if self.config.color_compressed:
                color_subscriber = booster.CompressedImageSubscriber(
                    self._handle_compressed_color,
                    self.config.color_topic,
                )
            else:
                color_subscriber = booster.ImageSubscriber(
                    self._handle_color,
                    self.config.color_topic,
                )

            subscribers: list[_Subscriber] = [color_subscriber]
            if self.config.depth_enabled:
                if self.config.depth_compressed:
                    depth_subscriber = booster.CompressedImageSubscriber(
                        self._handle_compressed_depth,
                        self.config.depth_topic,
                    )
                else:
                    depth_subscriber = booster.ImageSubscriber(
                        self._handle_depth,
                        self.config.depth_topic,
                    )
                subscribers.append(depth_subscriber)
            subscribers.append(
                booster.CameraInfoSubscriber(
                    self._handle_color_camera_info,
                    self.config.color_camera_info_topic,
                )
            )
            if self.config.depth_enabled:
                subscribers.append(
                    booster.CameraInfoSubscriber(
                        self._handle_depth_camera_info,
                        self.config.depth_camera_info_topic,
                    )
                )
            try:
                for subscriber in subscribers:
                    self._subscribers.append(subscriber)
                    subscriber.InitChannel()
            except Exception:
                self._close_subscribers()
                raise
            self._started.set()

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            self._close_subscribers()
            self._started.clear()
        super().stop()

    def _close_subscribers(self) -> None:
        for subscriber in reversed(self._subscribers):
            try:
                subscriber.CloseChannel()
            except Exception:
                logger.exception("Failed to close Booster camera subscriber")
        self._subscribers.clear()

    def _reset_metrics(self) -> None:
        self._color_metrics = _FrameAgeMetrics()
        self._depth_metrics = _FrameAgeMetrics()
        self._color_camera_info_metrics = _FrameAgeMetrics()
        self._depth_camera_info_metrics = _FrameAgeMetrics()
        self._last_metrics_log = time.perf_counter()

    def _reset_publish_rate_limiters(self) -> None:
        self._color_publish_rate = _PublishRateLimiter(self.config.publish_rate_hz)
        self._depth_publish_rate = _PublishRateLimiter(self.config.publish_rate_hz)

    def _log_metrics(self, stream: str, snapshot: _FrameAgeSnapshot) -> None:
        window_seconds = max(snapshot.window_seconds, 0.001)
        frame_age_ms = snapshot.frame_age_ms or {"p50": 0.0, "p95": 0.0, "max": 0.0}
        bridge_ms = snapshot.bridge_ms or {"p50": 0.0, "p95": 0.0, "max": 0.0}
        fps = snapshot.frames / window_seconds
        payload_mbps = snapshot.payload_bytes * 8.0 / window_seconds / 1_000_000.0
        values = {
            "fps": fps,
            "payload_mbps": payload_mbps,
            "frame_age/p50_ms": frame_age_ms["p50"],
            "frame_age/p95_ms": frame_age_ms["p95"],
            "frame_age/max_ms": frame_age_ms["max"],
            "bridge/p50_ms": bridge_ms["p50"],
            "bridge/p95_ms": bridge_ms["p95"],
            "bridge/max_ms": bridge_ms["max"],
            "invalid_timestamps": snapshot.invalid_timestamps,
            "negative_ages": snapshot.negative_ages,
        }
        try:
            self.camera_metrics.publish(
                MetricsArray.from_numeric_values(
                    f"metrics/camera/{stream}",
                    values,
                    hardware_id="booster_b1",
                )
            )
        except Exception:
            logger.exception("Failed to publish structured camera metrics")

    def _maybe_log_metrics(self) -> None:
        now = time.perf_counter()
        if now - self._last_metrics_log < self.config.metrics_interval_seconds:
            return
        with self._metrics_log_lock:
            now = time.perf_counter()
            if now - self._last_metrics_log < self.config.metrics_interval_seconds:
                return
            self._last_metrics_log = now
            self._log_metrics("color", self._color_metrics.snapshot_and_reset())
            if self.config.depth_enabled:
                self._log_metrics("depth", self._depth_metrics.snapshot_and_reset())
            self._log_metrics(
                "color_camera_info",
                self._color_camera_info_metrics.snapshot_and_reset(),
            )
            if self.config.depth_enabled:
                self._log_metrics(
                    "depth_camera_info",
                    self._depth_camera_info_metrics.snapshot_and_reset(),
                )

    def _handle_color(self, message: Any) -> None:
        processing_started = time.perf_counter()
        if not self._color_publish_rate.allow(processing_started):
            return
        try:
            image = booster_image_to_dimos(message)
            self.color_image.publish(image)
            self._color_metrics.record(image.ts, processing_started, image.data.nbytes)
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster color frame")

    def _handle_compressed_color(self, message: Any) -> None:
        processing_started = time.perf_counter()
        if not self._color_publish_rate.allow(processing_started):
            return
        try:
            image = booster_compressed_image_to_dimos(message)
            self.color_image.publish(image)
            self._color_metrics.record(image.ts, processing_started, image.data.nbytes)
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster compressed color frame")

    def _handle_depth(self, message: Any) -> None:
        processing_started = time.perf_counter()
        if not self._depth_publish_rate.allow(processing_started):
            return
        try:
            image = booster_depth_image_to_dimos(message, self.config.depth_scale)
            self.depth_image.publish(image)
            self._depth_metrics.record(image.ts, processing_started, image.data.nbytes)
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster depth frame")

    def _handle_compressed_depth(self, message: Any) -> None:
        processing_started = time.perf_counter()
        if not self._depth_publish_rate.allow(processing_started):
            return
        try:
            image = booster_compressed_depth_image_to_dimos(
                message,
                self.config.depth_scale,
            )
            self.depth_image.publish(image)
            self._depth_metrics.record(image.ts, processing_started, image.data.nbytes)
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster compressed depth frame")

    def _handle_color_camera_info(self, message: Any) -> None:
        processing_started = time.perf_counter()
        try:
            camera_info = booster_camera_info_to_dimos(message)
            payload_bytes = len(camera_info.lcm_encode())
            self.camera_info.publish(camera_info)
            self._color_camera_info_metrics.record(
                camera_info.ts,
                processing_started,
                payload_bytes,
            )
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster color camera info")

    def _handle_depth_camera_info(self, message: Any) -> None:
        processing_started = time.perf_counter()
        try:
            camera_info = booster_camera_info_to_dimos(message)
            payload_bytes = len(camera_info.lcm_encode())
            self.depth_camera_info.publish(camera_info)
            self._depth_camera_info_metrics.record(
                camera_info.ts,
                processing_started,
                payload_bytes,
            )
            self._maybe_log_metrics()
        except Exception:
            logger.exception("Failed to process Booster depth camera info")
