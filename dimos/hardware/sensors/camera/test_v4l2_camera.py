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

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.core.module import Module
from dimos.hardware.sensors.camera.v4l2_camera import V4L2CameraModule, capture_source
from dimos.msgs.sensor_msgs.Image import ImageFormat


def test_by_path_link_resolves_to_the_node_index(tmp_path: Any) -> None:
    """cv2 only reliably opens V4L2 devices by index, so links are resolved."""
    link = tmp_path / "platform-usb-0:2:1.0-video-index4"
    link.symlink_to("/dev/video18")

    assert capture_source(str(link)) == 18
    assert capture_source("/dev/video7") == 7
    assert capture_source("rtsp://camera/stream") == "rtsp://camera/stream"


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> V4L2CameraModule:
    def _fake_init(self: Any, **kwargs: Any) -> None:
        self.config = SimpleNamespace(
            device="/dev/v4l/by-path/fake-video-index4",
            width=848,
            height=480,
            fps=30.0,
            fourcc="YUYV",
            frame_id="wrist_left_optical",
            retry_s=0.0,
            max_missed_reads=3,
            stats_period_s=0.0,
        )

    monkeypatch.setattr(Module, "__init__", _fake_init)
    module = V4L2CameraModule()
    module.image_out = MagicMock()
    return module


class _FakeCapture:
    """Enough of cv2.VideoCapture for the read loop: a scripted read sequence."""

    def __init__(self, reads: list[bool], opened: bool = True) -> None:
        self._reads = list(reads)
        self._opened = opened
        self.released = False
        self.props: dict[int, float] = {}

    def isOpened(self) -> bool:  # cv2 spelling
        return self._opened

    def set(self, prop: int, value: float) -> bool:
        self.props[prop] = value
        return True

    def get(self, prop: int) -> float:
        return self.props.get(prop, 0.0)

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self._reads:
            return False, None
        ok = self._reads.pop(0)
        return (True, np.zeros((480, 848, 3), dtype=np.uint8)) if ok else (False, None)

    def release(self) -> None:
        self.released = True


def _fake_cv2(cap: _FakeCapture) -> SimpleNamespace:
    return SimpleNamespace(
        VideoCapture=lambda *_: cap,
        CAP_V4L2=200,
        CAP_PROP_FOURCC=6,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        VideoWriter_fourcc=lambda *c: 0,
    )


def test_frames_are_published_as_bgr_images(module: V4L2CameraModule) -> None:
    cap = _FakeCapture(reads=[True, True, False, False, False])
    cap.set(3, 848)
    cap.set(4, 480)
    cap.set(5, 30.0)
    assert module._open(_fake_cv2(cap)) is cap

    module._pump(cap)  # returns once max_missed_reads failures pile up

    assert module.image_out.publish.call_count == 2
    image = module.image_out.publish.call_args[0][0]
    assert image.format == ImageFormat.BGR
    assert (image.width, image.height) == (848, 480)
    assert image.frame_id == "wrist_left_optical"


def test_a_missed_read_between_frames_is_tolerated(module: V4L2CameraModule) -> None:
    cap = _FakeCapture(reads=[True, False, True, False, False, False])

    module._pump(cap)

    assert module.image_out.publish.call_count == 2


def test_device_held_by_another_process_does_not_raise(module: V4L2CameraModule) -> None:
    """The vendor node still owning the camera is a wait, not a crash."""
    cap = _FakeCapture(reads=[], opened=False)

    assert module._open(_fake_cv2(cap)) is None
    assert cap.released
    assert module.image_out.publish.call_count == 0
