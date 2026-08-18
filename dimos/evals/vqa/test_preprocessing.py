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
    RecordingFramePreprocessor,
    _align_one,
    _profile_pointcloud_to_camera,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL


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


def test_go2_profile_treats_legacy_image_pose_as_world_from_base() -> None:
    image = Observation[Image](
        ts=10.0,
        pose_tuple=(1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0),
        _data=cast("Image", object()),
    )
    lidar = Observation[PointCloud2](
        ts=10.0,
        _data=cast("PointCloud2", SimpleNamespace(frame_id="world")),
    )

    pointcloud_to_camera = _profile_pointcloud_to_camera(image, lidar)

    assert image.pose is not None
    expected = -(Transform.from_pose("base_link", image.pose) + BASE_TO_OPTICAL)
    assert pointcloud_to_camera.frame_id == "camera_optical"
    assert pointcloud_to_camera.child_frame_id == "world"
    assert np.allclose(pointcloud_to_camera.to_matrix(), expected.to_matrix())
