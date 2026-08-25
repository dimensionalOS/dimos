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

import numpy as np
import pytest

from dimos.evals.vqa.pointcloud_frame import (
    PointCloudFrameLoader,
    PointCloudFrameUnavailableError,
    _align_one,
    _ImageRectifier,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _populate_required_streams(
    store: SqliteStore,
    *,
    empty: str | None = None,
    include_empty_pointlio: bool = False,
    resolvable_tf: bool = True,
) -> None:
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
    camera_info = CameraInfo.from_intrinsics(1.0, 1.0, 0.0, 0.0, 2, 2, frame_id="camera_optical")
    transform = Transform(
        frame_id="world" if resolvable_tf else "unrelated_parent",
        child_frame_id="camera_optical" if resolvable_tf else "unrelated_child",
        ts=10.0,
    )

    images = store.stream("color_image", Image)
    camera_infos = store.stream("camera_info", CameraInfo)
    transforms = store.stream("tf", TFMessage)
    lidar = store.stream("lidar", PointCloud2)
    if include_empty_pointlio:
        store.stream("pointlio_lidar", PointCloud2)
    if empty != "color_image":
        images.append(image, ts=10.0)
    if empty != "camera_info":
        camera_infos.append(camera_info, ts=10.0)
    if empty != "tf":
        transforms.append(TFMessage(transform), ts=10.0)
    if empty != "lidar":
        lidar.append(cloud, ts=10.0)


def test_align_one_rejects_observation_outside_tolerance() -> None:
    with MemoryStore() as store:
        images = store.stream("color_image", str)
        lidar = store.stream("lidar", str)
        images.append("image", ts=10.0)
        lidar.append("cloud", ts=10.11)

        with pytest.raises(ValueError, match="within tolerance"):
            _align_one(images.order_by("ts"), lidar.order_by("ts"), 0.1)


def test_rectifier_rejects_invalid_camera_intrinsics() -> None:
    image = Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8))
    camera_info = CameraInfo.from_intrinsics(0.0, 1.0, 0.0, 0.0, 2, 2)

    with pytest.raises(ValueError, match="positive focal lengths"):
        _ImageRectifier().rectify(image, camera_info)


def test_rectifier_rejects_calibration_dimension_mismatch() -> None:
    image = Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8))
    camera_info = CameraInfo.from_intrinsics(1.0, 1.0, 0.0, 0.0, 1, 2)

    with pytest.raises(ValueError, match="do not match image dimensions"):
        _ImageRectifier().rectify(image, camera_info)


@pytest.mark.parametrize("missing_stream", ("color_image", "camera_info", "tf", "lidar"))
def test_rejects_missing_required_stream(missing_stream: str, tmp_path: Path) -> None:
    dataset = tmp_path / "recording.db"
    with SqliteStore(path=dataset) as store:
        if missing_stream != "color_image":
            store.stream("color_image", Image)
        if missing_stream != "camera_info":
            store.stream("camera_info", CameraInfo)
        if missing_stream != "tf":
            store.stream("tf", TFMessage)
        if missing_stream != "lidar":
            store.stream("lidar", PointCloud2)

    loader = PointCloudFrameLoader(dataset)
    expected = "pointlio_lidar.*lidar" if missing_stream == "lidar" else missing_stream
    with pytest.raises(ValueError, match=expected):
        loader.start()


@pytest.mark.parametrize("empty_stream", ("color_image", "camera_info", "tf", "lidar"))
def test_rejects_empty_required_stream(empty_stream: str, tmp_path: Path) -> None:
    dataset = tmp_path / "recording.db"
    with SqliteStore(path=dataset) as store:
        _populate_required_streams(store, empty=empty_stream)

    loader = PointCloudFrameLoader(dataset)
    expected = "pointlio_lidar/lidar" if empty_stream == "lidar" else empty_stream
    with pytest.raises(ValueError, match=rf"required {expected} stream is empty"):
        loader.start()


def test_uses_nonempty_lidar_when_pointlio_stream_is_empty(tmp_path: Path) -> None:
    dataset = tmp_path / "recording.db"
    with SqliteStore(path=dataset) as store:
        _populate_required_streams(store, include_empty_pointlio=True)

    with PointCloudFrameLoader(dataset) as loader:
        assert loader.load(0).pointcloud.frame_id == "world"


def test_unresolvable_tf_still_allows_image_only_fallback(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "recording.db"
    with SqliteStore(path=dataset) as store:
        _populate_required_streams(store, resolvable_tf=False)

    loader = PointCloudFrameLoader(dataset)
    monkeypatch.setattr(loader._rectifier, "rectify", lambda image, info: (image, info))
    with loader:
        with pytest.raises(PointCloudFrameUnavailableError, match="cannot resolve"):
            loader.load(0)
        assert loader.load_image(0).shape == (2, 2, 3)


def test_load_uses_recorded_camera_info_and_tf(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
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

    with SqliteStore(path=dataset) as store:
        store.stream("color_image", Image).append(image, ts=10.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("camera_info", CameraInfo).append(camera_info, ts=9.0)
        store.stream("tf", TFMessage).append(TFMessage(world_from_camera), ts=10.0)

    loader = PointCloudFrameLoader(dataset)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, camera_info.with_ts(source.ts)

    monkeypatch.setattr(
        loader._rectifier,
        "rectify",
        rectify,
    )
    with loader:
        frame = loader.load(0)

    assert calibrations == [camera_info]
    assert frame.calibration_source == "recorded"
    assert frame.camera_info.ts == image.ts
    assert np.allclose(frame.pointcloud_to_camera.to_matrix(), (-world_from_camera).to_matrix())


def test_load_selects_camera_info_for_each_image_timestamp(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
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

    with SqliteStore(path=dataset) as store:
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

    loader = PointCloudFrameLoader(dataset)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, calibration.with_ts(source.ts)

    monkeypatch.setattr(loader._rectifier, "rectify", rectify)
    with loader:
        early = loader.load(0)
        late = loader.load(1)

    assert calibrations == list(camera_infos)
    assert early.camera_info.frame_id == "camera_early"
    assert late.camera_info.frame_id == "camera_late"
