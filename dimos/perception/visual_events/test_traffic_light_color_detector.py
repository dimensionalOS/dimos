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

import cv2
import numpy as np

from dimos.perception.visual_events.traffic_light import (
    StableStateDebouncer,
    TrafficLightColorDetector,
)


def _circle_bgr(color: tuple[int, int, int], radius: int = 100) -> np.ndarray:
    image = np.zeros((320, 320, 3), dtype=np.uint8)
    cv2.circle(image, (160, 160), radius, color, thickness=-1)
    return image


def test_detects_red_circle() -> None:
    detector = TrafficLightColorDetector(min_area_ratio=0.01)
    result = detector.classify(_circle_bgr((0, 0, 255)), color_space="BGR")
    assert result.state == "red"
    assert result.confidence > 0.9
    assert result.red_score > result.green_score


def test_detects_yellow_circle() -> None:
    detector = TrafficLightColorDetector(min_area_ratio=0.01)
    result = detector.classify(_circle_bgr((0, 255, 255)), color_space="BGR")
    assert result.state == "yellow"
    assert result.confidence > 0.9
    assert result.yellow_score > result.red_score


def test_detects_green_circle() -> None:
    detector = TrafficLightColorDetector(min_area_ratio=0.01)
    result = detector.classify(_circle_bgr((0, 255, 0)), color_space="BGR")
    assert result.state == "green"
    assert result.confidence > 0.9
    assert result.green_score > result.red_score


def test_unknown_for_dark_or_invalid_frames() -> None:
    detector = TrafficLightColorDetector(min_area_ratio=0.01)
    assert detector.classify(np.zeros((320, 320, 3), dtype=np.uint8)).state == "unknown"
    assert detector.classify(np.zeros((320, 320), dtype=np.uint8)).state == "unknown"
    assert detector.classify(np.zeros((0, 0, 3), dtype=np.uint8)).state == "unknown"


def test_roi_ignores_colored_signal_outside_crop() -> None:
    detector = TrafficLightColorDetector(min_area_ratio=0.01)
    image = np.zeros((320, 320, 3), dtype=np.uint8)
    cv2.circle(image, (40, 40), 35, (0, 0, 255), thickness=-1)

    full_frame = detector.classify(image, color_space="BGR")
    cropped = detector.classify(image, color_space="BGR", roi=(0.35, 0.35, 0.95, 0.95))

    assert full_frame.state == "red"
    assert cropped.state == "unknown"


def test_raises_for_unsupported_color_space() -> None:
    detector = TrafficLightColorDetector()
    image = _circle_bgr((0, 0, 255))

    try:
        detector.classify(image, color_space="HSV")  # type: ignore[arg-type]
    except ValueError as exc:
        assert "Unsupported color_space" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unsupported color_space")


def test_debouncer_emits_after_stable_frames() -> None:
    debouncer = StableStateDebouncer(stable_frames=3, cooldown_s=0.0)

    assert debouncer.update("red", now=1.0) is None
    assert debouncer.update("red", now=2.0) is None
    assert debouncer.update("red", now=3.0) == "red"
    assert debouncer.update("red", now=4.0) is None

    assert debouncer.update("green", now=5.0) is None
    assert debouncer.update("green", now=6.0) is None
    assert debouncer.update("green", now=7.0) == "green"


def test_debouncer_resets_on_unknown_and_honors_cooldown() -> None:
    debouncer = StableStateDebouncer(stable_frames=2, cooldown_s=10.0)

    assert debouncer.update("red", now=1.0) is None
    assert debouncer.update("unknown", now=2.0) is None
    assert debouncer.update("red", now=3.0) is None
    assert debouncer.update("red", now=4.0) is None
    assert debouncer.update("red", now=14.1) == "red"

