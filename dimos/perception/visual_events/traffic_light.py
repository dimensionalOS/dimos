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

from dataclasses import dataclass
import time
from typing import Literal

import cv2
import numpy as np

TrafficLightState = Literal["red", "yellow", "green", "unknown"]


@dataclass(frozen=True)
class TrafficLightDetection:
    """Result from a lightweight color-state visual event detector."""

    state: TrafficLightState
    confidence: float
    red_score: float
    yellow_score: float
    green_score: float
    area_ratio: float
    ts: float


class TrafficLightColorDetector:
    """Classify large red, yellow, or green visual signals with HSV masks.

    This detector is intended for prototype visual-event triggers such as demo
    traffic signals, facility status lights, warning signs, and companion robot
    alerts. It is not a production traffic-safety perception system.
    """

    def __init__(
        self,
        min_area_ratio: float = 0.015,
        min_saturation: int = 80,
        min_value: int = 80,
        blur_kernel: int = 5,
    ) -> None:
        self.min_area_ratio = float(min_area_ratio)
        self.min_saturation = int(min_saturation)
        self.min_value = int(min_value)
        self.blur_kernel = int(blur_kernel)

    def classify(
        self,
        frame: np.ndarray,
        *,
        color_space: Literal["RGB", "BGR"] = "RGB",
        roi: tuple[float, float, float, float] | None = None,
    ) -> TrafficLightDetection:
        """Classify an image as red, yellow, green, or unknown.

        Args:
            frame: HxWx3 uint8 image.
            color_space: Whether the input frame is RGB or BGR.
            roi: Optional normalized crop as (x1, y1, x2, y2), each in [0, 1].
        """

        if frame is None or frame.size == 0:
            return self._unknown()
        if frame.ndim != 3 or frame.shape[2] < 3:
            return self._unknown()

        image = frame[:, :, :3]
        if color_space == "RGB":
            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif color_space == "BGR":
            bgr = image
        else:
            raise ValueError(f"Unsupported color_space: {color_space}")

        bgr = self._crop_normalized(bgr, roi)
        if bgr.size == 0:
            return self._unknown()

        if self.blur_kernel > 1:
            kernel_size = self.blur_kernel if self.blur_kernel % 2 == 1 else self.blur_kernel + 1
            bgr = cv2.GaussianBlur(bgr, (kernel_size, kernel_size), 0)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        red_mask_1 = cv2.inRange(
            hsv,
            np.array([0, self.min_saturation, self.min_value]),
            np.array([10, 255, 255]),
        )
        red_mask_2 = cv2.inRange(
            hsv,
            np.array([170, self.min_saturation, self.min_value]),
            np.array([179, 255, 255]),
        )
        red_score = self._mask_area_ratio(cv2.bitwise_or(red_mask_1, red_mask_2))

        yellow_score = self._mask_area_ratio(
            cv2.inRange(
                hsv,
                np.array([18, self.min_saturation, self.min_value]),
                np.array([34, 255, 255]),
            )
        )
        green_score = self._mask_area_ratio(
            cv2.inRange(
                hsv,
                np.array([35, self.min_saturation, self.min_value]),
                np.array([90, 255, 255]),
            )
        )

        scores: dict[TrafficLightState, float] = {
            "red": red_score,
            "yellow": yellow_score,
            "green": green_score,
        }
        best_state = max(scores, key=scores.__getitem__)
        best_score = scores[best_state]

        if best_score < self.min_area_ratio:
            return TrafficLightDetection(
                state="unknown",
                confidence=0.0,
                red_score=red_score,
                yellow_score=yellow_score,
                green_score=green_score,
                area_ratio=best_score,
                ts=time.time(),
            )

        total_score = red_score + yellow_score + green_score
        confidence = best_score / max(total_score, 1e-9)
        return TrafficLightDetection(
            state=best_state,
            confidence=float(min(max(confidence, 0.0), 1.0)),
            red_score=red_score,
            yellow_score=yellow_score,
            green_score=green_score,
            area_ratio=best_score,
            ts=time.time(),
        )

    def draw_debug_overlay(
        self,
        frame: np.ndarray,
        detection: TrafficLightDetection,
        *,
        color_space: Literal["RGB", "BGR"] = "RGB",
    ) -> np.ndarray:
        """Return a BGR debug frame with state and score overlays."""

        if color_space == "RGB":
            out = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        else:
            out = frame[:, :, :3].copy()

        cv2.putText(
            out,
            f"{detection.state.upper()} conf={detection.confidence:.2f} area={detection.area_ratio:.3f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            f"R={detection.red_score:.3f} Y={detection.yellow_score:.3f} G={detection.green_score:.3f}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return out

    @staticmethod
    def _crop_normalized(
        frame: np.ndarray,
        roi: tuple[float, float, float, float] | None,
    ) -> np.ndarray:
        if roi is None:
            return frame

        h, w = frame.shape[:2]
        x1, y1, x2, y2 = roi
        x1_i = max(0, min(w - 1, int(x1 * w)))
        y1_i = max(0, min(h - 1, int(y1 * h)))
        x2_i = max(x1_i + 1, min(w, int(x2 * w)))
        y2_i = max(y1_i + 1, min(h, int(y2 * h)))
        return frame[y1_i:y2_i, x1_i:x2_i]

    @staticmethod
    def _mask_area_ratio(mask: np.ndarray) -> float:
        kernel = np.ones((5, 5), np.uint8)
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
        return float(cv2.countNonZero(clean)) / float(mask.shape[0] * mask.shape[1])

    @staticmethod
    def _unknown() -> TrafficLightDetection:
        return TrafficLightDetection(
            state="unknown",
            confidence=0.0,
            red_score=0.0,
            yellow_score=0.0,
            green_score=0.0,
            area_ratio=0.0,
            ts=time.time(),
        )


class StableStateDebouncer:
    """Emit a visual event only after repeated stable frames."""

    def __init__(self, stable_frames: int = 3, cooldown_s: float = 2.0) -> None:
        self.stable_frames = int(stable_frames)
        self.cooldown_s = float(cooldown_s)
        self._candidate: TrafficLightState = "unknown"
        self._candidate_count = 0
        self._last_emitted: TrafficLightState = "unknown"
        self._last_emit_ts = 0.0

    def update(self, state: TrafficLightState, now: float | None = None) -> TrafficLightState | None:
        """Return a newly stable state, or None if no event should fire."""

        timestamp = time.time() if now is None else now

        if state == "unknown":
            self._candidate = "unknown"
            self._candidate_count = 0
            return None

        if state != self._candidate:
            self._candidate = state
            self._candidate_count = 1
            return None

        self._candidate_count += 1
        if self._candidate_count < self.stable_frames:
            return None
        if state == self._last_emitted:
            return None
        if timestamp - self._last_emit_ts < self.cooldown_s:
            return None

        self._last_emitted = state
        self._last_emit_ts = timestamp
        return state

