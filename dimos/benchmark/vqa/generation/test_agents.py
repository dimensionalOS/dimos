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

# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from typing import cast

import numpy as np

from dimos.benchmark.vqa.contracts import (
    CalibratedFrame,
    GroundedObject,
    PrimitiveGroundingConfig,
    QuestionIntent,
)
from dimos.benchmark.vqa.generation.deterministic_answerer import DeterministicAnswerer
from dimos.benchmark.vqa.generation.families import GroundingResult, render_question
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.question_authors import ConstrainedQuestionAuthor
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class _QuestionModel:
    def query(self, image: Image, prompt: str) -> str:
        assert image.width == 6
        return """```json
        [
            {"kind":"presence","object_query":"chair"},
            {"kind":"compare_nearest_by_side","object_query":"chair"},
            {"kind":"visible_count","object_query":"chair"},
            {"kind":"compare_left_right","object_query":"chair","comparison_query":"table"}
        ]
        ```"""


class _Detector:
    def __init__(self, image: Image, detection: Detection2DSeg) -> None:
        self._image = image
        self._detection = detection

    def detect(self, image: Image, query: str) -> ImageDetections2D:
        return ImageDetections2D(image, [self._detection] if query == "chair" else [])


class _MultiDetector:
    def __init__(self, image: Image, detections: list[Detection2DSeg]) -> None:
        self._image = image
        self._detections = detections

    def detect(self, image: Image, query: str) -> ImageDetections2D:
        return ImageDetections2D(self._image, self._detections if query == "chair" else [])


def _agent(
    frame: CalibratedFrame,
    detector: _Detector | _MultiDetector,
    segmenter: _Segmenter,
    *,
    localizer: _PointLocalizer | None = None,
    point_segmenter: _PointSegmenter | None = None,
    config: PrimitiveGroundingConfig = PrimitiveGroundingConfig(),
) -> DeterministicAnswerer:
    return DeterministicAnswerer(
        FramePerceptionPrimitives(frame, detector, segmenter, localizer, point_segmenter, config)
    )


class _Segmenter:
    def segment(self, detections: ImageDetections2D) -> ImageDetections2D:
        return detections


class _PointLocalizer:
    def locate(self, image: Image, query: str) -> ImageDetections2D:
        return ImageDetections2D(image, [Detection2DPoint(3.0, 3.0, query, 0.0, image)])


class _PointSegmenter:
    def __init__(self, detection: Detection2DSeg) -> None:
        self._detection = detection

    def segment_points(self, points: ImageDetections2D) -> ImageDetections2D:
        return ImageDetections2D(points.image, [self._detection])


def _frame_and_detection() -> tuple[CalibratedFrame, Detection2DSeg]:
    image = Image.from_numpy(np.zeros((6, 6, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(
            np.array([[0.0, 0.0, 1.0], [0.4, 0.0, 1.0], [-0.4, 0.0, 1.0]], dtype=np.float32)
        ),
        camera_info=CameraInfo.from_intrinsics(3.0, 3.0, 3.0, 3.0, 6, 6),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    detection = Detection2DSeg(
        (0.0, 0.0, 5.0, 5.0), 0, -1, 1.0, "chair", 0.0, image, np.full((6, 6), 255, dtype=np.uint8)
    )
    return frame, detection


def test_constrained_question_author_parses_supported_intents() -> None:
    frame, _ = _frame_and_detection()

    intents = ConstrainedQuestionAuthor(cast("OpenAIVlModel", _QuestionModel())).propose(
        frame.image
    )

    assert intents == [
        QuestionIntent(kind="presence", object_query="chair"),
        QuestionIntent(kind="compare_nearest_by_side", object_query="chair"),
        QuestionIntent(kind="visible_count", object_query="chair"),
        QuestionIntent(kind="compare_left_right", object_query="chair", comparison_query="table"),
    ]


def test_constrained_question_author_deduplicates_identical_intents() -> None:
    class _DuplicateQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            return """[
                {"kind":"presence","object_query":"chair"},
                {"kind":"presence","object_query":"chair"}
            ]"""

    frame, _ = _frame_and_detection()

    intents = ConstrainedQuestionAuthor(cast("OpenAIVlModel", _DuplicateQuestionModel())).propose(
        frame.image
    )

    assert intents == [QuestionIntent(kind="presence", object_query="chair")]


def test_question_rendering_handles_plural_queries_and_singular_units() -> None:
    assert render_question(QuestionIntent("visible_count", "bottles")) == (
        "How many visible instances of bottles are there?"
    )
    assert render_question(QuestionIntent("within_distance", "paper towels", 1.0)) == (
        "Is any detected instance of paper towels within 1 meter? Answer yes or no."
    )


def test_distance_thresholds_produce_distinct_case_ids() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    near = agent.answer(
        QuestionIntent(kind="within_distance", object_query="chair", threshold_m=2.0)
    )
    far = agent.answer(
        QuestionIntent(kind="within_distance", object_query="chair", threshold_m=3.0)
    )

    assert near.question.id != far.question.id


def test_deterministic_answerer_records_tools_and_rejects_unsupported_question() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    answered = agent.answer(
        QuestionIntent(kind="within_distance", object_query="chair", threshold_m=3.0)
    )
    rejected = agent.answer(QuestionIntent(kind="horizontal_direction", object_query="table"))
    absent = agent.answer(QuestionIntent(kind="presence", object_query="table"))

    assert answered.status == "answered"
    assert answered.answer == "yes"
    assert answered.evidence[0].point_count == 3
    assert [item.tool for item in answered.trace] == [
        "detect_objects",
        "segment_objects",
        "get_foreground_geometry",
    ]
    assert rejected.status == "rejected"
    assert rejected.reason == "no_visual_detection"
    assert absent.status == "rejected"


def test_deterministic_answerer_presence_does_not_require_an_accepted_mask() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=37),
    )

    result = agent.answer(QuestionIntent(kind="presence", object_query="chair"))

    assert result.status == "answered"
    assert result.answer == "yes"


def test_deterministic_answerer_falls_back_to_point_prompt() -> None:
    frame, detection = _frame_and_detection()
    point_detection = Detection2DSeg(
        detection.bbox,
        detection.track_id,
        detection.class_id,
        detection.confidence,
        "plant",
        detection.ts,
        detection.image,
        detection.mask,
    )
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        localizer=_PointLocalizer(),
        point_segmenter=_PointSegmenter(point_detection),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(
        QuestionIntent(kind="within_distance", object_query="plant", threshold_m=3.0)
    )

    assert result.answer == "yes"
    assert [item.tool for item in result.trace] == [
        "detect_objects",
        "locate_object_point",
        "segment_object_point",
        "get_foreground_geometry",
    ]


def test_deterministic_answerer_compares_nearest_objects_by_side() -> None:
    image = Image.from_numpy(np.zeros((6, 6, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(
            np.array(
                [
                    [-0.5, -0.6, 1.0],
                    [-0.5, 0.0, 1.0],
                    [-0.5, 0.6, 1.0],
                    [1.0, -1.0, 2.0],
                    [1.0, 0.0, 2.0],
                    [1.0, 1.0, 2.0],
                ],
                dtype=np.float32,
            )
        ),
        camera_info=CameraInfo.from_intrinsics(3.0, 3.0, 3.0, 3.0, 6, 6),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    left_mask = np.zeros((6, 6), dtype=np.uint8)
    left_mask[:, :3] = 255
    right_mask = np.zeros((6, 6), dtype=np.uint8)
    right_mask[:, 3:] = 255
    detections = [
        Detection2DSeg((0.0, 0.0, 2.0, 5.0), 0, -1, 1.0, "chair", 0.0, image, left_mask),
        Detection2DSeg((3.0, 0.0, 5.0, 5.0), 1, -1, 1.0, "chair", 0.0, image, right_mask),
    ]
    agent = _agent(
        frame,
        _MultiDetector(image, detections),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(QuestionIntent(kind="compare_nearest_by_side", object_query="chair"))

    assert result.status == "answered"
    assert result.answer == "left"
    assert result.question.object_ids == (
        "object:v1:frame-1:0001",
        "object:v1:frame-1:0002",
    )


def test_deterministic_answerer_rejects_side_comparison_without_both_sides() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(QuestionIntent(kind="compare_nearest_by_side", object_query="chair"))

    assert result.status == "rejected"
    assert result.reason == "missing_grounded_side"


def test_deterministic_answerer_buckets_visible_count_and_camera_range() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    count = agent.answer(QuestionIntent(kind="visible_count", object_query="chair"))
    camera_range = agent.answer(QuestionIntent(kind="camera_range", object_query="chair"))

    assert count.answer == "1-2"
    assert count.question.allowed_answers == ("0", "1-2", "3-4", "5-7", "8+")
    assert camera_range.answer == "1 to under 2 m"
    assert camera_range.question.allowed_answers == (
        "under 1 m",
        "1 to under 2 m",
        "2 to under 4 m",
        "4 m or more",
    )


def test_visual_families_do_not_require_point_cloud_support() -> None:
    frame, detection = _frame_and_detection()
    frame = CalibratedFrame(
        frame.id,
        frame.image,
        PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32)),
        frame.camera_info,
        frame.pointcloud_to_camera,
        True,
    )
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    presence = agent.answer(QuestionIntent(kind="presence", object_query="chair"))
    count = agent.answer(QuestionIntent(kind="visible_count", object_query="chair"))
    direction = agent.answer(QuestionIntent(kind="horizontal_direction", object_query="chair"))
    distance = agent.answer(
        QuestionIntent(kind="within_distance", object_query="chair", threshold_m=3.0)
    )

    assert presence.answer == "yes"
    assert count.answer == "1-2"
    assert direction.answer == "center"
    assert distance.status == "rejected"


def test_horizontal_direction_rejects_multiple_visual_instances() -> None:
    frame, detection = _frame_and_detection()
    other = Detection2DSeg(
        (0.0, 0.0, 1.0, 5.0),
        1,
        -1,
        1.0,
        "chair",
        0.0,
        frame.image,
        detection.mask,
    )
    agent = _agent(
        frame,
        _MultiDetector(frame.image, [detection, other]),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(QuestionIntent(kind="horizontal_direction", object_query="chair"))

    assert result.status == "rejected"
    assert result.reason == "ambiguous_visual_object"


def test_deterministic_answerer_compares_pairwise_left_right(monkeypatch: object) -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )
    chair = GroundedObject("chair-0", "chair", 8, 1.0, "left")
    table = GroundedObject("table-0", "table", 8, 2.0, "right")
    monkeypatch.setattr(
        agent.context,
        "ground",
        lambda query: GroundingResult((chair,) if query == "chair" else (table,), ()),
    )
    monkeypatch.setattr(
        agent.context.primitives,
        "horizontal_offset_m",
        lambda first, second: -0.5,
    )

    result = agent.answer(
        QuestionIntent(kind="compare_left_right", object_query="chair", comparison_query="table")
    )

    assert result.status == "answered"
    assert result.answer == "left"
    assert result.question.allowed_answers == ("left", "right")

    monkeypatch.setattr(agent.context.primitives, "horizontal_offset_m", lambda first, second: 0.0)
    ambiguous = agent.answer(
        QuestionIntent(kind="compare_left_right", object_query="chair", comparison_query="table")
    )
    assert ambiguous.status == "rejected"
    assert ambiguous.reason == "ambiguous_horizontal_relation"
