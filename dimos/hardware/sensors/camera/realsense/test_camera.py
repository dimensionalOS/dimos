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

from __future__ import annotations

import time
from unittest.mock import patch

import numpy as np
import pytest

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera, RealSenseCameraConfig
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def test_color_compression_is_opt_in() -> None:
    # Every color consumer in the repo takes In[Image] and none takes
    # In[CompressedImage], so raw stays the default until a decode path exists.
    assert RealSenseCameraConfig().compress_color is False


def _flaky_open(succeed_on):
    calls = {"n": 0}

    def open_pipeline(self):
        calls["n"] += 1
        if calls["n"] < succeed_on:
            raise RuntimeError("Couldn't resolve requests")

    return open_pipeline, calls


def test_busy_device_does_not_abort_the_whole_deployment() -> None:
    # start_all_modules aborts every module when one raises, so a camera that
    # lost a race for the USB device must not raise.
    open_pipeline, calls = _flaky_open(succeed_on=3)
    with patch.object(RealSenseCamera, "_open_pipeline", open_pipeline):
        camera = RealSenseCamera(start_retry_seconds=0.05)
        try:
            camera.start()
            deadline = time.monotonic() + 5.0
            while calls["n"] < 3 and time.monotonic() < deadline:
                time.sleep(0.05)
            assert calls["n"] >= 3, "should keep retrying until the device frees"
        finally:
            camera.stop()


def test_camera_still_fails_fast_by_default() -> None:
    # Every other blueprint keeps the old behaviour; retry is opt-in.
    open_pipeline, _ = _flaky_open(succeed_on=99)
    with patch.object(RealSenseCamera, "_open_pipeline", open_pipeline):
        camera = RealSenseCamera()
        try:
            with pytest.raises(RuntimeError):
                camera.start()
        finally:
            camera.stop()


def test_compressed_color_logs_to_rerun_without_decoding() -> None:
    rng = np.random.default_rng(0)
    raw = Image(
        data=rng.integers(0, 255, (48, 64, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera_color_optical_frame",
    )

    compressed = CompressedImage.from_image(raw, quality=RealSenseCameraConfig().jpeg_quality)

    # The bridge hands JPEG bytes straight to the viewer; decoding them on the
    # robot would spend Jetson CPU and re-inflate the stream.
    assert type(compressed.to_rerun()).__name__ == "EncodedImage"
