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

from __future__ import annotations

import ctypes
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.core.module import Module
from dimos.hardware.sensors.camera import v4l2_capture
from dimos.hardware.sensors.camera.v4l2_capture import V4L2Capture, fourcc_code
from dimos.hardware.sensors.camera.v4l2_color_depth import V4L2ColorDepthModule
from dimos.msgs.sensor_msgs.Image import ImageFormat


@pytest.mark.skipif(ctypes.sizeof(ctypes.c_long) != 8, reason="64-bit struct layout")
def test_ioctl_numbers_match_the_kernel_headers() -> None:
    """Values from <linux/videodev2.h> on a 64-bit build; a layout slip changes them."""
    assert v4l2_capture.VIDIOC_S_FMT == 0xC0D05605
    assert v4l2_capture.VIDIOC_REQBUFS == 0xC0145608
    assert v4l2_capture.VIDIOC_QUERYBUF == 0xC0585609
    assert v4l2_capture.VIDIOC_QBUF == 0xC058560F
    assert v4l2_capture.VIDIOC_DQBUF == 0xC0585611
    assert v4l2_capture.VIDIOC_STREAMON == 0x40045612
    assert v4l2_capture.VIDIOC_STREAMOFF == 0x40045613
    assert v4l2_capture.VIDIOC_S_PARM == 0xC0CC5616


def test_fourcc_is_little_endian_and_space_padded() -> None:
    assert fourcc_code("Z16") == fourcc_code("Z16 ") == 0x2036315A
    assert fourcc_code("YUYV") == 0x56595559


def test_absent_device_is_closed_not_raised(tmp_path: Any) -> None:
    cap = V4L2Capture(str(tmp_path / "video-index0"), 848, 480, 30.0)

    assert not cap.isOpened()
    assert cap.error
    assert cap.read() == (False, None)
    cap.release()  # idempotent


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> V4L2ColorDepthModule:
    def _fake_init(self: Any, **kwargs: Any) -> None:
        self.config = SimpleNamespace(
            color_device=str(tmp_path / "video-index4"),
            depth_device=str(tmp_path / "video-index0"),
            color_width=640,
            color_height=480,
            depth_width=848,
            depth_height=480,
            fps=30.0,
            frame_id="wrist_left_optical",
            retry_s=0.0,
            max_silent_s=3.0,
            stats_period_s=0.0,
        )

    monkeypatch.setattr(Module, "__init__", _fake_init)
    module = V4L2ColorDepthModule()
    module._open_failed = False
    return module


def test_yuyv_colour_is_published_as_bgr(module: V4L2ColorDepthModule) -> None:
    # Mid-grey in YUYV: Y=128 for both pixels of a pair, U=V=128.
    frame = np.full((480, 640, 2), 128, dtype=np.uint8)

    image = module._color_image(frame)

    assert image.format == ImageFormat.BGR
    assert image.data.shape == (480, 640, 3)
    assert abs(int(image.data[0, 0, 0]) - 128) <= 2


def test_depth_is_published_as_depth16_millimetres(module: V4L2ColorDepthModule) -> None:
    frame = np.full((480, 848), 612, dtype=np.uint16)

    image = module._depth_image(frame)

    assert image.format == ImageFormat.DEPTH16
    assert image.data.dtype == np.uint16
    assert (image.width, image.height) == (848, 480)
    assert image.frame_id == "wrist_left_optical"


def test_camera_that_cannot_open_waits(module: V4L2ColorDepthModule) -> None:
    assert module._open() is None
    assert module._open_failed
