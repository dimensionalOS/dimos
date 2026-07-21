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

"""Lazy YOLO-E adapter for the frozen observation-detector seam."""

from collections import Counter
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

from dimos.benchmark.spatiotemporal.models import BoundingBox2D
from dimos.benchmark.spatiotemporal.ports import DetectedObject
from dimos.msgs.sensor_msgs.Image import Image


@dataclass(frozen=True)
class YoloeAdapterStatistics:
    """Accumulated identity continuity statistics."""

    frames_processed: int
    detections_emitted: int
    fallback_detections: int
    continued_tracks: int
    dropped_tracks: int


class YoloeObservationDetector:
    """Normalize persisted YOLO-E detections while preserving object identity."""

    def __init__(
        self,
        prompts: Sequence[str],
        detector_factory: Callable[[], Any] | None = None,
    ) -> None:
        self._prompts = tuple(prompts)
        self._detector_factory = detector_factory
        self._detector: Any | None = None
        self._closed = False
        self._previous_object_ids: set[str] = set()
        self._frames_processed = 0
        self._detections_emitted = 0
        self._fallback_detections = 0
        self._continued_tracks = 0
        self._dropped_tracks = 0

    def detect(self, image: Image) -> tuple[DetectedObject, ...]:
        """Run persisted tracking and normalize its detections."""
        if self._closed:
            raise RuntimeError("detector is closed")
        detector = self._get_detector()
        result = detector.process_image(image)
        label_counts = Counter(detection.name for detection in result.detections)
        fallback_labels: set[str] = set()
        for detection in result.detections:
            if detection.track_id == -1:
                if detection.name not in self._prompts:
                    raise ValueError(f"fallback label is not a configured prompt: {detection.name}")
                if label_counts[detection.name] > 1:
                    raise ValueError(f"duplicate fallback label: {detection.name}")
                fallback_labels.add(detection.name)
        normalized = tuple(self._normalize(detection, image) for detection in result.detections)
        object_ids = {detection.object_id for detection in normalized}
        self._frames_processed += 1
        self._detections_emitted += len(normalized)
        self._fallback_detections += len(fallback_labels)
        self._continued_tracks += len(self._previous_object_ids & object_ids)
        self._dropped_tracks += len(self._previous_object_ids - object_ids)
        self._previous_object_ids = object_ids
        return normalized

    @property
    def statistics(self) -> YoloeAdapterStatistics:
        """Return an immutable snapshot of accumulated identity statistics."""
        return YoloeAdapterStatistics(
            frames_processed=self._frames_processed,
            detections_emitted=self._detections_emitted,
            fallback_detections=self._fallback_detections,
            continued_tracks=self._continued_tracks,
            dropped_tracks=self._dropped_tracks,
        )

    def close(self) -> None:
        """Release an initialized detector without forcing lazy construction."""
        if self._closed:
            return
        self._closed = True
        if self._detector is None:
            return
        self._cleanup_detector(self._detector)

    @staticmethod
    def _cleanup_detector(detector: Any) -> None:
        try:
            stop = getattr(detector, "stop", None)
            if stop is not None:
                stop()
        finally:
            close = getattr(detector, "close", None)
            if close is not None:
                close()

    def _get_detector(self) -> Any:
        if self._detector is None:
            if self._detector_factory is not None:
                detector = self._detector_factory()
            else:
                from dimos.perception.detection.detectors.yoloe import (
                    Yoloe2DDetector,
                    YoloePromptMode,
                )

                detector = Yoloe2DDetector(prompt_mode=YoloePromptMode.PROMPT)
            try:
                detector.set_prompts(text=list(self._prompts))
            except Exception:
                try:
                    self._cleanup_detector(detector)
                finally:
                    raise
            self._detector = detector
        return self._detector

    @staticmethod
    def _normalize(detection: Any, image: Image) -> DetectedObject:
        x_min, y_min, x_max, y_max = detection.bbox
        object_id = str(detection.track_id) if detection.track_id != -1 else detection.name
        return DetectedObject(
            object_id=object_id,
            label=detection.name,
            box=BoundingBox2D(
                x_min=x_min / image.width,
                y_min=y_min / image.height,
                x_max=x_max / image.width,
                y_max=y_max / image.height,
            ),
            confidence=detection.confidence,
        )
