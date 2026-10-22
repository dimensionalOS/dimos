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

import time

import numpy as np
import pytest

from dimos.core.introspection.module.info import StreamInfo
from dimos.evals.agents.mobile_scripted import green_box, public_output
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@pytest.fixture
def observations():
    ts = time.time()
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    image[:, :, 1] = 180
    color = Image(data=image, format=ImageFormat.RGB, ts=ts, frame_id="camera")
    depth = Image(
        data=np.full((10, 10), 2.0, dtype=np.float32),
        format=ImageFormat.DEPTH,
        ts=ts,
        frame_id="camera",
    )
    info = CameraInfo.from_intrinsics(10, 10, 4.5, 4.5, 10, 10, frame_id="camera")
    info.ts = ts
    transform = Transform(translation=Vector3(1, 2, 3), child_frame_id="camera", ts=ts)
    return color, depth, info, transform


def test_colour_and_measured_depth_project_through_camera_transform(observations):
    assert green_box(*observations) == Vector3(1, 2, 5)


@pytest.mark.parametrize("field,value", [("ts", 0), ("frame_id", "wrong")])
def test_stale_or_misaligned_images_are_not_detections(observations, monkeypatch, field, value):
    monkeypatch.setattr(observations[1], field, value)
    assert green_box(*observations) is None


def test_missing_green_or_invalid_depth_is_not_a_detection(observations):
    observations[1].data[:] = np.nan
    assert green_box(*observations) is None


def test_shared_channel_is_not_mistaken_for_two_cameras():
    first = StreamInfo("color_image", "Image", "connection", "out", "/camera")
    second = StreamInfo("color_image", "Image", "camera", "out", "/camera")
    assert public_output([first, second], "color_image") == first
    separate = StreamInfo("color_image", "Image", "other_camera", "out", "/other")
    with pytest.raises(ValueError, match="one public"):
        public_output([first, separate], "color_image")
    with pytest.raises(ValueError, match="one public"):
        public_output([StreamInfo("color_image", "Image", "camera", "out", None)], "color_image")
