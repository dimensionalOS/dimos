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

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.evals.vqa.calibrated_frame import (
    RecordingFramePreprocessor,
    _align_one,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def test_align_one_rejects_observation_outside_tolerance() -> None:
    with MemoryStore() as store:
        images = store.stream("color_image", str)
        lidar = store.stream("lidar", str)
        images.append("image", ts=10.0)
        lidar.append("cloud", ts=10.11)

        with pytest.raises(ValueError, match="within tolerance"):
            _align_one(images.order_by("ts"), lidar.order_by("ts"), 0.1)


def test_requires_recorded_calibration(tmp_path: Path) -> None:
    recording = tmp_path / "recording.db"
    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image)
        store.stream("lidar", PointCloud2)

    preprocessor = RecordingFramePreprocessor(recording)
    with pytest.raises(ValueError, match="camera_info, tf"):
        preprocessor.start()


@pytest.mark.parametrize(
    ("stream_name", "stream_type"), (("camera_info", CameraInfo), ("tf", TFMessage))
)
def test_rejects_incomplete_recorded_calibration(
    stream_name: str, stream_type: type[Any], tmp_path: Path
) -> None:
    recording = tmp_path / "recording.db"
    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image)
        store.stream("lidar", PointCloud2)
        store.stream(stream_name, stream_type)

    preprocessor = RecordingFramePreprocessor(recording)
    missing = "tf" if stream_name == "camera_info" else "camera_info"
    with pytest.raises(ValueError, match=missing):
        preprocessor.start()


def test_load_uses_recorded_camera_info_and_tf(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    recording = tmp_path / "recording.db"
    image = Image.from_numpy(
        np.zeros((2, 2, 3), dtype=np.uint8),
        frame_id="camera_optical",
        ts=10.0,
    )
    cloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 1.0]]),
        frame_id="world",
        timestamp=10.0,
    )
    camera_info = CameraInfo.from_intrinsics(
        1.0, 1.0, 0.0, 0.0, image.width, image.height, frame_id="camera_optical"
    )
    world_from_camera = Transform(
        translation=Vector3(1.0, 2.0, 0.5),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=10.0,
    )

    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image).append(image, ts=10.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("camera_info", CameraInfo).append(camera_info, ts=9.0)
        store.stream("tf", TFMessage).append(TFMessage(world_from_camera), ts=10.0)

    preprocessor = RecordingFramePreprocessor(recording)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, camera_info.with_ts(source.ts)

    monkeypatch.setattr(
        preprocessor._rectifier,
        "rectify",
        rectify,
    )
    with preprocessor:
        frame = preprocessor.load(0)

    assert calibrations == [camera_info]
    assert frame.calibration_source == "recorded"
    assert frame.camera_info.ts == image.ts
    assert np.allclose(frame.pointcloud_to_camera.to_matrix(), (-world_from_camera).to_matrix())


def test_load_selects_camera_info_for_each_image_timestamp(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    recording = tmp_path / "recording.db"
    images = (
        Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8), ts=10.0),
        Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8), ts=20.0),
    )
    camera_infos = (
        CameraInfo.from_intrinsics(10.0, 10.0, 0.0, 0.0, 2, 2, frame_id="camera_early"),
        CameraInfo.from_intrinsics(100.0, 100.0, 0.0, 0.0, 2, 2, frame_id="camera_late"),
    )
    camera_transforms = (
        Transform(frame_id="world", child_frame_id="camera_early", ts=10.0),
        Transform(frame_id="world", child_frame_id="camera_late", ts=20.0),
    )

    with SqliteStore(path=recording) as store:
        image_stream = store.stream("color_image", Image)
        lidar_stream = store.stream("lidar", PointCloud2)
        camera_info_stream = store.stream("camera_info", CameraInfo)
        tf_stream = store.stream("tf", TFMessage)
        for image, timestamp in zip(images, (10.0, 20.0), strict=True):
            image_stream.append(image, ts=timestamp)
            lidar_stream.append(
                PointCloud2.from_numpy(
                    np.array([[0.0, 0.0, 1.0]]),
                    frame_id="world",
                    timestamp=timestamp,
                ),
                ts=timestamp,
            )
        camera_info_stream.append(camera_infos[0], ts=9.0)
        camera_info_stream.append(camera_infos[1], ts=19.0)
        for transform in camera_transforms:
            tf_stream.append(TFMessage(transform), ts=transform.ts)

    preprocessor = RecordingFramePreprocessor(recording)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, calibration.with_ts(source.ts)

    monkeypatch.setattr(preprocessor._rectifier, "rectify", rectify)
    with preprocessor:
        early = preprocessor.load(0)
        late = preprocessor.load(1)

    assert calibrations == list(camera_infos)
    assert early.camera_info.frame_id == "camera_early"
    assert late.camera_info.frame_id == "camera_late"
