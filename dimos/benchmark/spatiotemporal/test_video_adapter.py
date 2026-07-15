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

from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.benchmark.spatiotemporal.models import BoundingBox2D
from dimos.benchmark.spatiotemporal.ports import DetectedObject
from dimos.benchmark.spatiotemporal.video_adapter import OpenCVVideoSampler
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class _FakeCapture:
    def __init__(
        self,
        frames: list[np.ndarray],
        *,
        declared_frame_count: int | None = None,
        fps: float = 2.0,
        opened: bool = True,
    ) -> None:
        self._frames = iter(frames)
        self._declared_frame_count = (
            len(frames) if declared_frame_count is None else declared_frame_count
        )
        self._fps = fps
        self._opened = opened
        self.released = False

    def isOpened(self) -> bool:
        return self._opened

    def get(self, property_id: int) -> float:
        if property_id == cv2.CAP_PROP_FPS:
            return self._fps
        if property_id == cv2.CAP_PROP_FRAME_COUNT:
            return float(self._declared_frame_count)
        raise AssertionError(f"unexpected capture property: {property_id}")

    def read(self) -> tuple[bool, np.ndarray | None]:
        frame = next(self._frames, None)
        return frame is not None, frame

    def release(self) -> None:
        self.released = True


class _FakeDetector:
    def __init__(self) -> None:
        self.images: list[Image] = []
        self.closed = False

    def detect(self, image: Image) -> tuple[DetectedObject, ...]:
        self.images.append(image)
        return (
            DetectedObject(
                object_id="mug-1",
                label="mug",
                box=BoundingBox2D(x_min=0.25, y_min=0.25, x_max=0.75, y_max=0.75),
                confidence=0.9,
            ),
        )

    def close(self) -> None:
        self.closed = True


def test_sampler_deterministically_normalizes_sampled_frames_and_cleans_up(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    frames = [
        np.full((2, 3, 3), (index, index + 1, index + 2), dtype=np.uint8) for index in range(4)
    ]
    capture = _FakeCapture(frames)
    detector = _FakeDetector()
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=detector, frame_stride=2)
    observations = sampler.sample(Path("episode.mp4"), episode_id="episode-1")
    sampler.close()

    assert [(item.frame_id, item.timestamp_s) for item in observations] == [
        (0, 0.0),
        (2, 1.0),
    ]
    assert all(item.episode_id == "episode-1" for item in observations)
    assert all(item.object_id == "mug-1" for item in observations)
    assert [image.format for image in detector.images] == [ImageFormat.RGB, ImageFormat.RGB]
    assert detector.images[0].frame_id == "0"
    assert detector.images[0].ts == 0.0
    np.testing.assert_array_equal(detector.images[0].data[0, 0], np.array([2, 1, 0]))
    assert capture.released
    assert detector.closed


def test_sampler_rejects_non_positive_frame_stride() -> None:
    with pytest.raises(ValueError, match="frame_stride must be positive"):
        OpenCVVideoSampler(detector=_FakeDetector(), frame_stride=0)


def test_sampler_reports_early_decode_failure_and_releases_capture(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    capture = _FakeCapture(
        [np.zeros((2, 3, 3), dtype=np.uint8)],
        declared_frame_count=2,
    )
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=_FakeDetector())
    with pytest.raises(RuntimeError, match="failed to decode frame 1"):
        sampler.sample(Path("truncated.mp4"), episode_id="episode-1")

    assert capture.released


def test_sampler_reports_unopened_video_and_releases_capture(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    capture = _FakeCapture([], opened=False)
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=_FakeDetector())
    with pytest.raises(RuntimeError, match="failed to open video: missing.mp4"):
        sampler.sample(Path("missing.mp4"), episode_id="episode-1")

    assert capture.released


def test_sampler_reports_zero_frame_video_and_releases_capture(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    capture = _FakeCapture([])
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=_FakeDetector())
    with pytest.raises(RuntimeError, match="video contains no decodable frames"):
        sampler.sample(Path("empty.mp4"), episode_id="episode-1")

    assert capture.released


@pytest.mark.parametrize("fps", [0.0, -1.0, float("nan")])
def test_sampler_rejects_invalid_fps_and_releases_capture(
    monkeypatch: pytest.MonkeyPatch,
    fps: float,
) -> None:
    capture = _FakeCapture([np.zeros((2, 3, 3), dtype=np.uint8)], fps=fps)
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=_FakeDetector())
    with pytest.raises(ValueError, match="video FPS must be finite and positive"):
        sampler.sample(Path("invalid-fps.mp4"), episode_id="episode-1")

    assert capture.released


def test_sampler_rejects_non_bgr_frame_dimensions_and_releases_capture(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    capture = _FakeCapture([np.zeros((2, 3), dtype=np.uint8)])
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)

    sampler = OpenCVVideoSampler(detector=_FakeDetector())
    with pytest.raises(ValueError, match="decoded frame 0 must have BGR dimensions"):
        sampler.sample(Path("grayscale.mp4"), episode_id="episode-1")

    assert capture.released
