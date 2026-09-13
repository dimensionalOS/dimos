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

from collections.abc import Iterator, Sequence

import numpy as np

from dimos.memory.transform import Transformer
from dimos.memory.type.observation import Observation
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class _CandidateManager:
    """Own the caller-ordered candidate queue, validation, and removals."""

    _match_min_iou: float = 0.25

    def __init__(self, candidates: Sequence[Detection2DBBox]) -> None:
        self._candidates = list(candidates)

    @property
    def candidates(self) -> list[Detection2DBBox]:
        return list(self._candidates)

    def __bool__(self) -> bool:
        return bool(self._candidates)

    def pop_candidate(self) -> Detection2DBBox:
        return self._candidates.pop(0)

    def drop_by_window(self, start: float, end: float) -> None:
        self._candidates = [
            candidate for candidate in self._candidates if not start <= candidate.ts <= end
        ]

    def matches_candidate(
        self,
        candidate: Detection2DBBox,
        prediction: ImageDetections2D,
    ) -> bool:
        return (
            bool(prediction)
            and self._bbox_iou(
                prediction[0].bbox,
                candidate.bbox,
            )
            >= self._match_min_iou
        )

    @staticmethod
    def _bbox_iou(first: Bbox, second: Bbox) -> float:
        ax1, ay1, ax2, ay2 = first
        bx1, by1, bx2, by2 = second
        intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
            0.0, min(ay2, by2) - max(ay1, by1)
        )
        union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
        return intersection / union if union > 0 else 0.0


class FindPerson(Transformer[Image, ImageDetections2D]):
    """Track person candidates through an image stream."""

    _max_lost_frames: int = 15

    def __init__(self, candidates: Sequence[Detection2DBBox]) -> None:
        if not isinstance(candidates, Sequence):
            raise TypeError("candidates must be a finite Sequence[Detection2DBBox]")

        copied = tuple(candidates)
        if not copied:
            raise ValueError("candidates must not be empty")

        for index, candidate in enumerate(copied):
            if not isinstance(candidate, Detection2DBBox):
                raise TypeError(f"candidates[{index}] must be a Detection2DBBox")
            if not candidate.is_valid():
                raise ValueError(f"candidates[{index}] must be valid")

        self._candidates = copied

    def __call__(
        self, upstream: Iterator[Observation[Image]]
    ) -> Iterator[Observation[ImageDetections2D]]:
        frames = list(upstream)
        if not frames:
            return

        tracker = EdgeTAMProcessor()
        try:
            outputs = self._track_candidates(
                frames,
                tracker,
                _CandidateManager(self._candidates),
            )
        finally:
            tracker.stop()

        for obs, detections in zip(frames, outputs, strict=True):
            yield obs.derive(data=detections)

    def _track_candidates(
        self,
        frames: Sequence[Observation[Image]],
        tracker: EdgeTAMProcessor,
        candidates: _CandidateManager,
    ) -> list[ImageDetections2D]:
        """Process candidates in caller order and combine their window outputs."""

        # Initialize the first accepted window from the first supplied candidate.
        seed = candidates.pop_candidate()
        outputs, start, end = self._track_window(tracker, frames, seed)
        windows = [(seed, start, end)]

        # Remove candidates already covered by the accepted window.
        candidates.drop_by_window(start, end)

        while candidates:
            candidate = candidates.pop_candidate()
            reference = self._nearest_window_seed(windows, candidate)

            tracker.init_track(
                reference.image,
                box=np.asarray(reference.bbox, dtype=np.float32),
            )
            prediction = tracker.process_image(candidate.image)
            if not candidates.matches_candidate(candidate, prediction):
                continue

            checked_windows = [
                (window_start, window_end) for _, window_start, window_end in windows
            ]
            tracked, start, end = self._track_window(
                tracker,
                frames,
                candidate,
                checked_windows,
            )
            outputs = [
                current if current else new for current, new in zip(outputs, tracked, strict=True)
            ]
            windows.append((candidate, start, end))
            candidates.drop_by_window(start, end)

        return outputs

    def _track_window(
        self,
        tracker: EdgeTAMProcessor,
        frames: Sequence[Observation[Image]],
        seed: Detection2DBBox,
        checked_windows: Sequence[tuple[float, float]] = (),
    ) -> tuple[list[ImageDetections2D], float, float]:
        """Track one candidate backward and forward to produce an accepted window."""
        if not frames[0].ts <= seed.ts <= frames[-1].ts:
            raise ValueError("candidate is outside the upstream time range")

        first_forward = next(index for index, obs in enumerate(frames) if obs.ts >= seed.ts)
        has_seed_frame = frames[first_forward].ts == seed.ts
        backward: list[tuple[int, ImageDetections2D]] = []

        if first_forward:
            tracker.init_track(
                seed.image,
                box=np.asarray(seed.bbox, dtype=np.float32),
            )
            backward = self._propagate_direction(
                tracker,
                frames,
                range(first_forward - 1, -1, -1),
                checked_windows,
            )

        seed_output = tracker.init_track(
            seed.image,
            box=np.asarray(seed.bbox, dtype=np.float32),
        )
        forward = self._propagate_direction(
            tracker,
            frames,
            range(first_forward + int(has_seed_frame), len(frames)),
            checked_windows,
        )

        at_seed = [(first_forward, seed_output)] if has_seed_frame else []
        samples = [*backward, *at_seed, *forward]
        if not any(detections for _, detections in samples):
            raise RuntimeError("EdgeTAM produced no detections for candidate")

        tracked = [ImageDetections2D(image=obs.data) for obs in frames]
        for index, detections in samples:
            tracked[index] = detections

        processed_indices = [index for index, _ in samples]
        start = frames[min(processed_indices)].ts
        end = frames[max(processed_indices)].ts
        return tracked, start, end

    def _propagate_direction(
        self,
        tracker: EdgeTAMProcessor,
        frames: Sequence[Observation[Image]],
        indices: range,
        checked_windows: Sequence[tuple[float, float]],
    ) -> list[tuple[int, ImageDetections2D]]:
        """Stop at a window or after more than the configured consecutive empty frames."""
        tracked = []
        lost_count = 0

        for index in indices:
            observation = frames[index]
            if any(start <= observation.ts <= end for start, end in checked_windows):
                break

            detections = tracker.process_image(observation.data)
            tracked.append((index, detections))
            lost_count = 0 if detections else lost_count + 1
            if lost_count > self._max_lost_frames:
                break

        return tracked

    @staticmethod
    def _nearest_window_seed(
        windows: Sequence[tuple[Detection2DBBox, float, float]],
        candidate: Detection2DBBox,
    ) -> Detection2DBBox:
        """Return the seed whose window boundary is closest to the candidate."""
        return min(
            windows,
            key=lambda window: min(
                abs(candidate.ts - window[1]),
                abs(candidate.ts - window[2]),
            ),
        )[0]
