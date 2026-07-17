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

"""Replay M20 navigation inputs reconstructed from a Rerun recording."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from threading import Event, Thread
import time
from typing import Any, Literal, NamedTuple

import numpy as np
from pydantic import Field
import rerun_bindings as rr_bindings

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.deeprobotics.m20.tf import (
    FRONT_CAMERA_OPTICAL_FRAME,
    REAR_CAMERA_OPTICAL_FRAME,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_POINTS_ENTITY = "/world/slam_aligned_points"
_ODOMETRY_ENTITY = "/world/tf/base_link"
_FRONT_IMAGE_ENTITY = "/world/color_image"
_REAR_IMAGE_ENTITY = "/world/color_image_rear"
_POINTS_COLUMN = "Points3D:positions"
_TRANSLATION_COLUMN = "Transform3D:translation"
_QUATERNION_COLUMN = "Transform3D:quaternion"
_IMAGE_BUFFER_COLUMN = "Image:buffer"
_IMAGE_FORMAT_COLUMN = "Image:format"
_TIME_COLUMN = "log_time"

_IMAGE_FORMATS = {
    1: (ImageFormat.GRAY, 1),
    2: (ImageFormat.RGB, 3),
    3: (ImageFormat.RGBA, 4),
    4: (ImageFormat.BGR, 3),
    5: (ImageFormat.BGRA, 4),
}


class ReplayEvent(NamedTuple):
    timestamp: float
    kind: Literal["points", "odometry", "front_image", "rear_image"]
    message: PointCloud2 | Odometry | Image


class M20RrdReplayConfig(ModuleConfig):
    rrd_path: str | None = None
    playback_speed: float = Field(default=1.0, gt=0.0)


def _timestamp(batch: Any, row: int) -> float:
    return float(batch.column(_TIME_COLUMN)[row].value) / 1_000_000_000.0


def _fixed_size_vectors(batch: Any, column: str, row: int, width: int) -> np.ndarray:
    vectors = batch.column(column)[row].values
    return vectors.values.to_numpy(zero_copy_only=False).reshape(-1, width)


def _pointcloud_from_batch(batch: Any, row: int) -> PointCloud2:
    timestamp = _timestamp(batch, row)
    points = _fixed_size_vectors(batch, _POINTS_COLUMN, row, 3)
    return PointCloud2.from_numpy(points, frame_id="map", timestamp=timestamp)


def _odometry_from_batch(batch: Any, row: int) -> Odometry:
    timestamp = _timestamp(batch, row)
    translation = _fixed_size_vectors(batch, _TRANSLATION_COLUMN, row, 3)[0]
    quaternion = _fixed_size_vectors(batch, _QUATERNION_COLUMN, row, 4)[0]
    return Odometry(
        ts=timestamp,
        frame_id="map",
        child_frame_id="base_link",
        pose=Pose(translation, quaternion),
    )


def _image_from_batch(batch: Any, row: int, frame_id: str) -> Image:
    timestamp = _timestamp(batch, row)
    formats = batch.column(_IMAGE_FORMAT_COLUMN)[row].as_py()
    if len(formats) != 1:
        raise ValueError(f"Expected one Rerun image format, got {len(formats)}")

    image_metadata = formats[0]
    if image_metadata["pixel_format"] is not None or image_metadata["channel_datatype"] != 6:
        raise ValueError(f"Unsupported Rerun image format: {image_metadata}")

    color_model = image_metadata["color_model"]
    if color_model not in _IMAGE_FORMATS:
        raise ValueError(f"Unsupported Rerun color model: {color_model}")
    image_format, channels = _IMAGE_FORMATS[color_model]

    buffer = batch.column(_IMAGE_BUFFER_COLUMN)[row].values
    pixels = buffer[0].values.to_numpy(zero_copy_only=False)
    height = image_metadata["height"]
    width = image_metadata["width"]
    shape = (height, width) if channels == 1 else (height, width, channels)
    return Image.from_numpy(
        pixels.reshape(shape), format=image_format, frame_id=frame_id, ts=timestamp
    )


def iter_rrd_events(path: str | Path) -> Iterator[ReplayEvent]:
    """Yield reconstructed sensor messages in file order."""
    for chunk in rr_bindings.RrdReaderInternal(str(path)).stream():
        if chunk.entity_path not in {
            _POINTS_ENTITY,
            _ODOMETRY_ENTITY,
            _FRONT_IMAGE_ENTITY,
            _REAR_IMAGE_ENTITY,
        }:
            continue

        batch = chunk.to_record_batch()
        columns = set(batch.schema.names)
        if chunk.entity_path == _POINTS_ENTITY and _POINTS_COLUMN in columns:
            for row in range(batch.num_rows):
                message = _pointcloud_from_batch(batch, row)
                yield ReplayEvent(message.ts, "points", message)
        elif chunk.entity_path == _ODOMETRY_ENTITY and {
            _TRANSLATION_COLUMN,
            _QUATERNION_COLUMN,
        }.issubset(columns):
            for row in range(batch.num_rows):
                message = _odometry_from_batch(batch, row)
                yield ReplayEvent(message.ts, "odometry", message)
        elif chunk.entity_path in {_FRONT_IMAGE_ENTITY, _REAR_IMAGE_ENTITY} and {
            _IMAGE_BUFFER_COLUMN,
            _IMAGE_FORMAT_COLUMN,
        }.issubset(columns):
            is_front = chunk.entity_path == _FRONT_IMAGE_ENTITY
            frame_id = FRONT_CAMERA_OPTICAL_FRAME if is_front else REAR_CAMERA_OPTICAL_FRAME
            kind: Literal["front_image", "rear_image"] = "front_image" if is_front else "rear_image"
            for row in range(batch.num_rows):
                message = _image_from_batch(batch, row, frame_id)
                yield ReplayEvent(message.ts, kind, message)


class M20RrdReplay(Module):
    """Publish M20 SLAM and camera data from a Rerun recording."""

    dedicated_worker = True

    config: M20RrdReplayConfig
    slam_aligned_points: Out[PointCloud2]
    slam_odom: Out[Odometry]
    color_image: Out[Image]
    color_image_rear: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = Event()
        self._replay_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        path = Path(self.config.rrd_path or self.config.g.replay_db).expanduser()
        if path.suffix.lower() != ".rrd":
            raise ValueError(f"M20 replay requires an .rrd file, got: {path}")
        if not path.is_file():
            raise FileNotFoundError(f"M20 RRD recording does not exist: {path}")

        super().start()
        self._stop_event.clear()
        self._replay_thread = Thread(
            target=self._replay,
            args=(path,),
            daemon=True,
            name="m20-rrd-replay",
        )
        self._replay_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._replay_thread and self._replay_thread.is_alive():
            self._replay_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _replay(self, path: Path) -> None:
        first_recorded_time: float | None = None
        replay_started = time.monotonic()
        count = 0

        logger.info("Replaying M20 navigation data from %s", path)
        for event in iter_rrd_events(path):
            if self._stop_event.is_set():
                return
            if first_recorded_time is None:
                first_recorded_time = event.timestamp

            elapsed = (event.timestamp - first_recorded_time) / self.config.playback_speed
            deadline = replay_started + max(0.0, elapsed)
            if self._stop_event.wait(max(0.0, deadline - time.monotonic())):
                return

            if event.kind == "points":
                assert isinstance(event.message, PointCloud2)
                self.slam_aligned_points.publish(event.message)
            elif event.kind == "odometry":
                assert isinstance(event.message, Odometry)
                self.slam_odom.publish(event.message)
            elif event.kind == "front_image":
                assert isinstance(event.message, Image)
                self.color_image.publish(event.message)
            else:
                assert isinstance(event.message, Image)
                self.color_image_rear.publish(event.message)
            count += 1

        logger.info("M20 RRD replay completed after publishing %d messages", count)
