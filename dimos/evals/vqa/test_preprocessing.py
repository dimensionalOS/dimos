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
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from dimos.evals.vqa.preprocessing import (
    FrameGeometryUnavailableError,
    RecordingFramePreprocessor,
    _align_one,
    _profile_pointcloud_to_camera,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.calibration import BASE_TO_OPTICAL


def test_align_one_rejects_observation_outside_tolerance() -> None:
    with MemoryStore() as store:
        images = store.stream("color_image", str)
        lidar = store.stream("lidar", str)
        images.append("image", ts=10.0)
        lidar.append("cloud", ts=10.11)

        with pytest.raises(ValueError, match="within tolerance"):
            _align_one(images.order_by("ts"), lidar.order_by("ts"), 0.1)


@pytest.mark.parametrize("profile", ["GO2", "robot", ""])
def test_calibration_profile_validation(profile: Any, tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="unsupported calibration profile"):
        RecordingFramePreprocessor(tmp_path / "recording.db", calibration_profile=profile)


def test_go2_profile_must_be_explicit_for_uncalibrated_recording(tmp_path: Path) -> None:
    recording = tmp_path / "recording.db"
    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image)
        store.stream("lidar", PointCloud2)

    preprocessor = RecordingFramePreprocessor(recording)
    with pytest.raises(ValueError, match="calibration_profile='go2'"):
        preprocessor.start()


def test_go2_profile_requires_odom_stream(tmp_path: Path) -> None:
    recording = tmp_path / "recording.db"
    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image)
        store.stream("lidar", PointCloud2)

    preprocessor = RecordingFramePreprocessor(recording, calibration_profile="go2")
    with pytest.raises(ValueError, match="requires an 'odom' stream"):
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

    preprocessor = RecordingFramePreprocessor(recording, calibration_profile="go2")
    with pytest.raises(ValueError, match="camera_info and tf streams must both be present"):
        preprocessor.start()


def test_go2_profile_applies_camera_mount_once_to_world_from_base_odometry() -> None:
    odom = Observation[PoseStamped](
        ts=10.0,
        _data=PoseStamped(
            ts=10.0,
            frame_id="world",
            position=(1.0, 2.0, 0.5),
            orientation=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    lidar = Observation[PointCloud2](
        ts=10.0,
        _data=cast("PointCloud2", SimpleNamespace(frame_id="world")),
    )

    pointcloud_to_camera = _profile_pointcloud_to_camera(odom, lidar)

    expected = -(Transform.from_pose("base_link", odom.data) + BASE_TO_OPTICAL)
    assert pointcloud_to_camera.frame_id == "camera_optical"
    assert pointcloud_to_camera.child_frame_id == "world"
    assert np.allclose(pointcloud_to_camera.to_matrix(), expected.to_matrix())


def test_go2_profile_load_uses_nearest_odom_instead_of_optical_image_pose(
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
    nearest_odom = PoseStamped(
        ts=10.01,
        frame_id="world",
        position=(1.0, 2.0, 0.5),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    world_from_camera = Transform.from_pose("base_link", nearest_odom) + BASE_TO_OPTICAL

    with SqliteStore(path=recording) as store:
        store.stream("color_image", Image).append(image, ts=10.0, pose=world_from_camera.to_pose())
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        odom = store.stream("odom", PoseStamped)
        odom.append(PoseStamped(ts=9.95, frame_id="world", position=(9.0, 9.0, 0.0)), ts=9.95)
        odom.append(nearest_odom, ts=10.01)

    preprocessor = RecordingFramePreprocessor(recording, calibration_profile="go2")
    output_info = CameraInfo.from_intrinsics(
        1.0, 1.0, 0.0, 0.0, image.width, image.height, frame_id="camera_optical"
    )
    monkeypatch.setattr(
        preprocessor._rectifier,
        "rectify",
        lambda source, calibration: (source, output_info),
    )
    with preprocessor:
        frame = preprocessor.load(0)

    correct = -world_from_camera
    double_mounted = -(
        Transform.from_pose("base_link", world_from_camera.to_pose()) + BASE_TO_OPTICAL
    )
    assert np.allclose(frame.pointcloud_to_camera.to_matrix(), correct.to_matrix())
    assert not np.allclose(frame.pointcloud_to_camera.to_matrix(), double_mounted.to_matrix())


def test_go2_profile_rejects_ambiguous_odometry_and_pointcloud_frames() -> None:
    lidar = Observation[PointCloud2](
        ts=10.0,
        _data=cast("PointCloud2", SimpleNamespace(frame_id="world")),
    )
    odom = Observation[PoseStamped](ts=10.0, _data=PoseStamped(ts=10.0, frame_id="odom"))

    with pytest.raises(FrameGeometryUnavailableError, match="frame_id='world'"):
        _profile_pointcloud_to_camera(odom, lidar)

    odom.data.frame_id = "world"
    lidar.data.frame_id = "lidar"
    with pytest.raises(FrameGeometryUnavailableError, match="world-frame point cloud"):
        _profile_pointcloud_to_camera(odom, lidar)
