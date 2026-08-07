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

"""Single-frame VQA private ground-truth generation."""

from __future__ import annotations

from dimos.benchmark.vqa.generation.grounding import ground_segmented_objects
from dimos.benchmark.vqa.generation.questions import generate_questions
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    ObjectDetector,
    ObjectSegmenter,
    VqaExample,
)
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


def generate_ground_truth(
    frame: CalibratedFrame,
    queries: list[str],
    detector: ObjectDetector,
    segmenter: ObjectSegmenter,
) -> list[VqaExample]:
    """Generate VQA examples from MoonDream/EdgeTAM-style image models and LiDAR."""
    segmented: list[Detection2DSeg] = []
    for query in queries:
        detections = detector.detect(frame.image, query)
        result = segmenter.segment(detections)
        segmented.extend(detection for detection in result if isinstance(detection, Detection2DSeg))
    objects = ground_segmented_objects(frame, segmented)
    return generate_questions(frame.id, objects, queries)
