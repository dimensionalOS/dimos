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

from dataclasses import replace
from pathlib import Path

from dimos_cdrgen.registry import encode
import numpy as np
import pytest

from dimos.evals.vqa.pointcloud_frame import (
    PointCloudFrameLoader,
    PointCloudFrameUnavailableError,
    _align_one,
    _ImageRectifier,
    select_quality_indices,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.message_utils import (
    camera_info as camera_info_utils,
    image as image_utils,
    pointcloud as pointcloud_utils,
)
from dimos.message_utils.geometry import inverse_transform, transform_matrix
from dimos.message_utils.time import time_from_seconds, to_seconds
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage


def _populate_required_streams(
    store: SqliteStore,
    *,
    empty: str | None = None,
    include_empty_pointlio: bool = False,
    resolvable_tf: bool = True,
) -> None:
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(10.0), frame_id="camera_optical"),
    )
    cloud = pointcloud_utils.from_xyz(
        np.array([[0.0, 0.0, 1.0]]), header=Header(stamp=time_from_seconds(10.0), frame_id="world")
    )
    camera_info = camera_info_utils.from_intrinsics(
        1.0,
        1.0,
        0.0,
        0.0,
        2,
        2,
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(0.0)),
    )
    transform = TransformStamped(
        header=Header(
            stamp=time_from_seconds(10.0), frame_id="world" if resolvable_tf else "unrelated_parent"
        ),
        child_frame_id="camera_optical" if resolvable_tf else "unrelated_child",
        transform=Transform(
            translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
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
        transforms.append(TFMessage(transforms=[transform]), ts=10.0)
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
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(0.0), frame_id=""),
    )
    camera_info = replace(
        camera_info_utils.from_intrinsics(
            1.0, 1.0, 0.0, 0.0, 2, 2, header=Header(frame_id="", stamp=time_from_seconds(0.0))
        ),
        k=np.zeros(9, dtype=np.float64),
    )

    with pytest.raises(ValueError, match="positive focal lengths"):
        _ImageRectifier().rectify(image, camera_info)


def test_rectifier_rejects_calibration_dimension_mismatch() -> None:
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(0.0), frame_id=""),
    )
    camera_info = camera_info_utils.from_intrinsics(
        1.0, 1.0, 0.0, 0.0, 1, 2, header=Header(frame_id="", stamp=time_from_seconds(0.0))
    )

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
        assert loader.load(0).pointcloud.header.frame_id == "world"


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
        assert image_utils.view(loader.load_image(0)).shape == (2, 2, 3)


def test_load_uses_recorded_camera_info_and_tf(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(10.0), frame_id="camera_optical"),
    )
    cloud = pointcloud_utils.from_xyz(
        np.array([[0.0, 0.0, 1.0]]), header=Header(stamp=time_from_seconds(10.0), frame_id="world")
    )
    camera_info = camera_info_utils.from_intrinsics(
        1.0,
        1.0,
        0.0,
        0.0,
        image.width,
        image.height,
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(0.0)),
    )
    world_from_camera = TransformStamped(
        header=Header(stamp=time_from_seconds(10.0), frame_id="world"),
        child_frame_id="camera_optical",
        transform=Transform(
            translation=Vector3(1.0, 2.0, 0.5), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
    )

    with SqliteStore(path=dataset) as store:
        store.stream("color_image", Image).append(image, ts=10.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("camera_info", CameraInfo).append(camera_info, ts=9.0)
        store.stream("tf", TFMessage).append(TFMessage(transforms=[world_from_camera]), ts=10.0)

    loader = PointCloudFrameLoader(dataset)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, replace(
            camera_info, header=replace(camera_info.header, stamp=source.header.stamp)
        )

    monkeypatch.setattr(
        loader._rectifier,
        "rectify",
        rectify,
    )
    with loader:
        frame = loader.load(0)

    assert len(calibrations) == 1
    assert encode(calibrations[0]) == encode(camera_info)
    assert frame.calibration_source == "recorded"
    assert to_seconds(frame.camera_info.header.stamp) == to_seconds(image.header.stamp)
    assert np.allclose(
        transform_matrix(frame.pointcloud_to_camera.transform),
        transform_matrix(inverse_transform(world_from_camera).transform),
    )


def test_load_resolves_tf_through_non_world_root(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(10.0), frame_id="camera_optical"),
    )
    cloud = pointcloud_utils.from_xyz(
        np.array([[0.0, 0.0, 1.0]]), header=Header(stamp=time_from_seconds(10.0), frame_id="lidar")
    )
    camera_info = camera_info_utils.from_intrinsics(
        1.0,
        1.0,
        0.0,
        0.0,
        2,
        2,
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(0.0)),
    )
    transforms = (
        TransformStamped(
            header=Header(stamp=time_from_seconds(10.0), frame_id="map"),
            child_frame_id="camera_optical",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        ),
        TransformStamped(
            header=Header(stamp=time_from_seconds(10.0), frame_id="map"),
            child_frame_id="lidar",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        ),
    )
    with SqliteStore(path=dataset) as store:
        store.stream("color_image", Image).append(image, ts=10.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("camera_info", CameraInfo).append(camera_info, ts=10.0)
        store.stream("tf", TFMessage).append(TFMessage(transforms=list(transforms)), ts=10.0)

    loader = PointCloudFrameLoader(dataset)
    monkeypatch.setattr(loader._rectifier, "rectify", lambda source, info: (source, info))
    with loader:
        frame = loader.load(0)

    assert np.allclose(transform_matrix(frame.pointcloud_to_camera.transform), np.eye(4))


def test_load_applies_camera_rectification_to_pointcloud_transform(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(stamp=time_from_seconds(10.0), frame_id="camera_optical"),
    )
    cloud = pointcloud_utils.from_xyz(
        np.array([[0.0, 0.0, 1.0]]), header=Header(stamp=time_from_seconds(10.0), frame_id="lidar")
    )
    camera_info = camera_info_utils.from_intrinsics(
        1.0,
        1.0,
        0.0,
        0.0,
        2,
        2,
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(0.0)),
    )
    rectification = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    camera_info.r = rectification.ravel()
    with SqliteStore(path=dataset) as store:
        store.stream("color_image", Image).append(image, ts=10.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("camera_info", CameraInfo).append(camera_info, ts=10.0)
        store.stream("tf", TFMessage).append(
            TFMessage(
                transforms=[
                    TransformStamped(
                        header=Header(stamp=time_from_seconds(10.0), frame_id="camera_optical"),
                        child_frame_id="lidar",
                        transform=Transform(
                            translation=Vector3(0.0, 0.0, 0.0),
                            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                        ),
                    )
                ]
            ),
            ts=10.0,
        )

    loader = PointCloudFrameLoader(dataset)
    monkeypatch.setattr(loader._rectifier, "rectify", lambda source, info: (source, info))
    with loader:
        frame = loader.load(0)

    assert np.allclose(
        transform_matrix(frame.pointcloud_to_camera.transform)[:3, :3], rectification
    )


def test_load_selects_camera_info_for_each_image_timestamp(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    dataset = tmp_path / "recording.db"
    images = (
        image_utils.from_array(
            np.zeros((2, 2, 3), dtype=np.uint8),
            encoding="rgb8",
            header=Header(stamp=time_from_seconds(10.0), frame_id=""),
        ),
        image_utils.from_array(
            np.zeros((2, 2, 3), dtype=np.uint8),
            encoding="rgb8",
            header=Header(stamp=time_from_seconds(20.0), frame_id=""),
        ),
    )
    camera_infos = (
        camera_info_utils.from_intrinsics(
            10.0,
            10.0,
            0.0,
            0.0,
            2,
            2,
            header=Header(frame_id="camera_early", stamp=time_from_seconds(0.0)),
        ),
        camera_info_utils.from_intrinsics(
            100.0,
            100.0,
            0.0,
            0.0,
            2,
            2,
            header=Header(frame_id="camera_late", stamp=time_from_seconds(0.0)),
        ),
    )
    camera_transforms = (
        TransformStamped(
            header=Header(stamp=time_from_seconds(10.0), frame_id="world"),
            child_frame_id="camera_early",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        ),
        TransformStamped(
            header=Header(stamp=time_from_seconds(20.0), frame_id="world"),
            child_frame_id="camera_late",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        ),
    )

    with SqliteStore(path=dataset) as store:
        image_stream = store.stream("color_image", Image)
        lidar_stream = store.stream("lidar", PointCloud2)
        camera_info_stream = store.stream("camera_info", CameraInfo)
        tf_stream = store.stream("tf", TFMessage)
        for image, timestamp in zip(images, (10.0, 20.0), strict=True):
            image_stream.append(image, ts=timestamp)
            lidar_stream.append(
                pointcloud_utils.from_xyz(
                    np.array([[0.0, 0.0, 1.0]]),
                    header=Header(stamp=time_from_seconds(timestamp), frame_id="world"),
                ),
                ts=timestamp,
            )
        camera_info_stream.append(camera_infos[0], ts=9.0)
        camera_info_stream.append(camera_infos[1], ts=19.0)
        for transform in camera_transforms:
            tf_stream.append(
                TFMessage(transforms=[transform]), ts=to_seconds(transform.header.stamp)
            )

    loader = PointCloudFrameLoader(dataset)
    calibrations: list[CameraInfo] = []

    def rectify(source: Image, calibration: CameraInfo) -> tuple[Image, CameraInfo]:
        calibrations.append(calibration)
        return source, replace(
            calibration, header=replace(calibration.header, stamp=source.header.stamp)
        )

    monkeypatch.setattr(loader._rectifier, "rectify", rectify)
    with loader:
        early = loader.load(0)
        late = loader.load(1)

    assert [encode(info) for info in calibrations] == [encode(info) for info in camera_infos]
    assert early.camera_info.header.frame_id == "camera_early"
    assert late.camera_info.header.frame_id == "camera_late"


def test_select_quality_indices_uses_mem_quality_window(monkeypatch) -> None:
    monkeypatch.setattr(image_utils, "sharpness", lambda image: image.sharpness)
    monkeypatch.setattr(image_utils, "brightness", lambda image: image.brightness)
    from types import SimpleNamespace

    with MemoryStore() as store:
        images = store.stream("color_image")
        # Two 1s windows: sharpest at index 1 then index 4.
        for ts, sharpness, brightness in (
            (0.0, 0.3, 0.5),
            (0.4, 0.9, 0.5),
            (0.8, 0.1, 0.5),
            (1.0, 0.5, 0.5),
            (1.4, 0.8, 0.5),
        ):
            images.append(
                SimpleNamespace(sharpness=sharpness, brightness=brightness),
                ts=ts,
            )

        stream = store.stream("color_image").order_by("ts")
        assert select_quality_indices(stream, 0, 5, quality_window=1.0) == (1, 4)
        assert select_quality_indices(stream, 0, 5, quality_window=1.0, min_brightness=0.2) == (
            1,
            4,
        )
        # Dark mid-frame (index 2) is dropped before the window pick.
        assert select_quality_indices(stream, 0, 3, quality_window=1.0, min_brightness=0.2) == (1,)
