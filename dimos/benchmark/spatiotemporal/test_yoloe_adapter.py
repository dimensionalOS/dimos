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

from dataclasses import dataclass

import numpy as np
import pytest

from dimos.benchmark.spatiotemporal.yoloe_adapter import YoloeObservationDetector
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@dataclass
class _FakeDetection:
    bbox: tuple[float, float, float, float]
    track_id: int
    confidence: float
    name: str


class _FakeYoloeDetector:
    def __init__(self, frames: list[list[_FakeDetection]]) -> None:
        self._frames = iter(frames)
        self.images: list[Image] = []
        self.cleanup_calls: list[str] = []
        self.fail_stop = False
        self.fail_prompts = False
        self.prompts: list[str] | None = None

    def set_prompts(self, text: list[str]) -> None:
        self.prompts = text
        if self.fail_prompts:
            raise RuntimeError("prompt setup failed")

    def process_image(self, image: Image) -> object:
        self.images.append(image)
        return type("Detections", (), {"detections": next(self._frames)})()

    def stop(self) -> None:
        self.cleanup_calls.append("stop")
        if self.fail_stop:
            raise RuntimeError("stop failed")

    def close(self) -> None:
        self.cleanup_calls.append("close")


def _image() -> Image:
    return Image.from_numpy(
        np.zeros((100, 200, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
    )


def test_adapter_preserves_native_ids_and_uses_unique_prompt_fallback() -> None:
    detector = _FakeYoloeDetector(
        [
            [
                _FakeDetection((20.0, 10.0, 60.0, 50.0), 17, 0.9, "bottle"),
                _FakeDetection((100.0, 20.0, 180.0, 80.0), -1, 0.8, "mug"),
            ]
        ]
    )
    adapter = YoloeObservationDetector(
        prompts=("bottle", "mug"),
        detector_factory=lambda: detector,
    )

    detections = adapter.detect(_image())

    assert [(item.object_id, item.label) for item in detections] == [
        ("17", "bottle"),
        ("mug", "mug"),
    ]
    assert detections[0].box.model_dump() == {
        "x_min": 0.1,
        "y_min": 0.1,
        "x_max": 0.3,
        "y_max": 0.5,
    }
    assert detector.images


def test_adapter_rejects_duplicate_fallback_labels() -> None:
    detector = _FakeYoloeDetector(
        [
            [
                _FakeDetection((20.0, 10.0, 60.0, 50.0), -1, 0.9, "mug"),
                _FakeDetection((100.0, 20.0, 180.0, 80.0), -1, 0.8, "mug"),
            ]
        ]
    )
    adapter = YoloeObservationDetector(
        prompts=("mug",),
        detector_factory=lambda: detector,
    )

    with pytest.raises(ValueError, match="duplicate fallback label: mug"):
        adapter.detect(_image())


def test_adapter_rejects_fallback_when_tracked_detection_has_same_label() -> None:
    detector = _FakeYoloeDetector(
        [
            [
                _FakeDetection((20.0, 10.0, 60.0, 50.0), 17, 0.9, "mug"),
                _FakeDetection((100.0, 20.0, 180.0, 80.0), -1, 0.8, "mug"),
            ]
        ]
    )
    adapter = YoloeObservationDetector(
        prompts=("mug",),
        detector_factory=lambda: detector,
    )

    with pytest.raises(ValueError, match="duplicate fallback label: mug"):
        adapter.detect(_image())


def test_adapter_rejects_fallback_for_non_prompt_label() -> None:
    detector = _FakeYoloeDetector([[_FakeDetection((20.0, 10.0, 60.0, 50.0), -1, 0.9, "plate")]])
    adapter = YoloeObservationDetector(
        prompts=("mug",),
        detector_factory=lambda: detector,
    )

    with pytest.raises(ValueError, match="fallback label is not a configured prompt: plate"):
        adapter.detect(_image())


def test_adapter_records_continuity_and_drop_statistics() -> None:
    detector = _FakeYoloeDetector(
        [
            [
                _FakeDetection((20.0, 10.0, 60.0, 50.0), 17, 0.9, "bottle"),
                _FakeDetection((100.0, 20.0, 180.0, 80.0), -1, 0.8, "mug"),
            ],
            [
                _FakeDetection((25.0, 10.0, 65.0, 50.0), 17, 0.9, "bottle"),
                _FakeDetection((105.0, 20.0, 185.0, 80.0), 18, 0.8, "mug"),
            ],
            [_FakeDetection((110.0, 20.0, 190.0, 80.0), 18, 0.8, "mug")],
        ]
    )
    adapter = YoloeObservationDetector(
        prompts=("bottle", "mug"),
        detector_factory=lambda: detector,
    )

    adapter.detect(_image())
    adapter.detect(_image())
    adapter.detect(_image())

    assert adapter.statistics.frames_processed == 3
    assert adapter.statistics.detections_emitted == 5
    assert adapter.statistics.fallback_detections == 1
    assert adapter.statistics.continued_tracks == 2
    assert adapter.statistics.dropped_tracks == 2


def test_adapter_closes_detector_even_when_stop_fails() -> None:
    detector = _FakeYoloeDetector([[]])
    detector.fail_stop = True
    adapter = YoloeObservationDetector(
        prompts=("mug",),
        detector_factory=lambda: detector,
    )
    adapter.detect(_image())

    with pytest.raises(RuntimeError, match="stop failed"):
        adapter.close()

    assert detector.cleanup_calls == ["stop", "close"]


def test_close_before_detection_is_terminal_without_constructing_detector() -> None:
    constructions = 0

    def factory() -> _FakeYoloeDetector:
        nonlocal constructions
        constructions += 1
        return _FakeYoloeDetector([[]])

    adapter = YoloeObservationDetector(prompts=("mug",), detector_factory=factory)

    adapter.close()

    with pytest.raises(RuntimeError, match="detector is closed"):
        adapter.detect(_image())
    assert constructions == 0


def test_failed_prompt_setup_cleans_up_and_retries_construction() -> None:
    failed = _FakeYoloeDetector([[]])
    failed.fail_prompts = True
    recovered = _FakeYoloeDetector([[]])
    candidates = iter((failed, recovered))
    adapter = YoloeObservationDetector(
        prompts=("mug",),
        detector_factory=lambda: next(candidates),
    )

    with pytest.raises(RuntimeError, match="prompt setup failed"):
        adapter.detect(_image())

    assert failed.cleanup_calls == ["stop", "close"]
    assert adapter.detect(_image()) == ()
    assert recovered.prompts == ["mug"]
