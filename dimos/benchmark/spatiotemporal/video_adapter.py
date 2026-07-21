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

"""Deterministic OpenCV video sampling through the frozen detector seam."""

import math
from pathlib import Path

import cv2

from dimos.benchmark.spatiotemporal.models import ObjectObservation
from dimos.benchmark.spatiotemporal.ports import ObservationDetector
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class OpenCVVideoSampler:
    """Sample decoded video frames and normalize detector results."""

    def __init__(self, detector: ObservationDetector, frame_stride: int = 1) -> None:
        if frame_stride < 1:
            raise ValueError("frame_stride must be positive")
        self._detector = detector
        self._frame_stride = frame_stride
        self._sample_schedule: tuple[tuple[int, float], ...] = ()
        self._closed = False

    def sample(self, video_path: Path, episode_id: str) -> tuple[ObjectObservation, ...]:
        """Return detector observations for every selected source frame."""
        capture = cv2.VideoCapture(str(video_path))
        observations: list[ObjectObservation] = []
        sample_schedule: list[tuple[int, float]] = []
        try:
            if not capture.isOpened():
                raise RuntimeError(f"failed to open video: {video_path}")
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(fps) or fps <= 0.0:
                raise ValueError("video FPS must be finite and positive")
            declared_frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            frame_id = 0
            while True:
                decoded, frame = capture.read()
                if not decoded:
                    if frame_id == 0:
                        raise RuntimeError("video contains no decodable frames")
                    if frame_id < declared_frame_count:
                        raise RuntimeError(f"failed to decode frame {frame_id}")
                    break
                if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
                    raise ValueError(f"decoded frame {frame_id} must have BGR dimensions")
                if frame_id % self._frame_stride == 0:
                    timestamp_s = frame_id / fps
                    sample_schedule.append((frame_id, timestamp_s))
                    image = Image.from_numpy(
                        cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                        format=ImageFormat.RGB,
                        frame_id=str(frame_id),
                        ts=timestamp_s,
                    )
                    observations.extend(
                        ObjectObservation(
                            episode_id=episode_id,
                            frame_id=frame_id,
                            timestamp_s=timestamp_s,
                            object_id=detection.object_id,
                            label=detection.label,
                            box=detection.box,
                            confidence=detection.confidence,
                        )
                        for detection in self._detector.detect(image)
                    )
                frame_id += 1
        finally:
            capture.release()
        self._sample_schedule = tuple(sample_schedule)
        return tuple(observations)

    @property
    def sample_schedule(self) -> tuple[tuple[int, float], ...]:
        """Return every selected frame, including frames with no detections."""
        return self._sample_schedule

    def close(self) -> None:
        """Release the detector owned by this sampler."""
        if not self._closed:
            self._detector.close()
            self._closed = True
