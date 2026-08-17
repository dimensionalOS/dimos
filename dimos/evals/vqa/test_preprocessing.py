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

import pytest

from dimos.evals.vqa.preprocessing import RecordingFramePreprocessor, _align_one
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_align_one_uses_nearest_observation_timestamp() -> None:
    with MemoryStore() as store:
        images = store.stream("color_image", str)
        lidar = store.stream("lidar", str)
        images.append("payload timestamp is irrelevant", ts=10.0)
        lidar.append("early", ts=9.8)
        lidar.append("nearest", ts=9.97)
        lidar.append("late", ts=10.08)

        aligned = _align_one(images.order_by("ts"), lidar.order_by("ts"), 0.1)

    assert aligned.data == "nearest"
    assert aligned.ts == 9.97


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
