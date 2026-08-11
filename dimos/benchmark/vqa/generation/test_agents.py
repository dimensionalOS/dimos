# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from typing import cast

import numpy as np

from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.primitives.contracts import (
    HeightMeasurementResult,
    HorizontalRelationResult,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.question_agent import OpenAIQuestionAgent
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundingConfig,
    GroundPlaneEstimate,
    OracleMeasurement,
    QuestionIntent,
)
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
        assert "point clouds" in prompt
        return '```json\n["chair"]\n```'


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
    config: GroundingConfig = GroundingConfig(),
) -> VqaGroundTruthGenerator:
    return VqaGroundTruthGenerator(
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


def test_question_agent_returns_constrained_intents() -> None:
    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _QuestionModel())).propose(frame.image)

    assert intents == [
        QuestionIntent(kind="presence", object_query="chair"),
        QuestionIntent(kind="horizontal_direction", object_query="chair"),
        QuestionIntent(kind="within_distance", object_query="chair", threshold_m=3.0),
        QuestionIntent(kind="visible_count", object_query="chair"),
        QuestionIntent(kind="camera_range", object_query="chair"),
        QuestionIntent(kind="compare_nearest_by_side", object_query="chair"),
    ]


def test_question_agent_uses_image_selected_structured_intents() -> None:
    class _StructuredQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            assert "Do not return bare object names" in prompt
            assert "opposite image sides" in prompt
            return '[{"kind":"compare_nearest_by_side","object_query":"chair"}]'

    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _StructuredQuestionModel())).propose(
        frame.image
    )

    assert intents == [QuestionIntent(kind="compare_nearest_by_side", object_query="chair")]


def test_question_agent_accepts_door_state_intent() -> None:
    class _DoorQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            assert "door_state" in prompt
            return '[{"kind":"door_state","object_query":"door"}]'

    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _DoorQuestionModel())).propose(frame.image)

    assert intents == [QuestionIntent(kind="door_state", object_query="door")]


def test_question_agent_accepts_closest_object_intent() -> None:
    class _ClosestQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            assert "candidate_queries" in prompt
            return (
                '[{"kind":"closest_object","object_query":"chair",'
                '"candidate_queries":["table","lamp"]}]'
            )

    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _ClosestQuestionModel())).propose(
        frame.image
    )

    assert intents == [QuestionIntent("closest_object", "chair", None, ("table", "lamp"))]


def test_question_agent_accepts_forward_path_intent() -> None:
    class _ForwardPathQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            assert "forward_path" in prompt
            return '[{"kind":"forward_path","object_query":"forward path"}]'

    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _ForwardPathQuestionModel())).propose(
        frame.image
    )

    assert intents == [QuestionIntent(kind="forward_path", object_query="forward path")]


def test_question_agent_accepts_count_range_and_height_comparison_intents() -> None:
    class _GeometryQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            assert "visible_count" in prompt
            assert "comparison_query" in prompt
            return """[
                {"kind":"visible_count","object_query":"chair"},
                {"kind":"camera_range","object_query":"lamp"},
                {"kind":"compare_left_right","object_query":"chair","comparison_query":"table"},
                {"kind":"compare_height","object_query":"chair","comparison_query":"table"}
            ]"""

    frame, _ = _frame_and_detection()

    intents = OpenAIQuestionAgent(cast("OpenAIVlModel", _GeometryQuestionModel())).propose(
        frame.image
    )

    assert intents == [
        QuestionIntent(kind="visible_count", object_query="chair"),
        QuestionIntent(kind="camera_range", object_query="lamp"),
        QuestionIntent(kind="compare_left_right", object_query="chair", comparison_query="table"),
        QuestionIntent(kind="compare_height", object_query="chair", comparison_query="table"),
    ]


def test_ground_truth_agent_records_tools_and_rejects_unsupported_question() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=1),
    )

    answered = agent.answer(
        frame, QuestionIntent(kind="within_distance", object_query="chair", threshold_m=3.0)
    )
    rejected = agent.answer(
        frame, QuestionIntent(kind="horizontal_direction", object_query="table")
    )
    absent = agent.answer(frame, QuestionIntent(kind="presence", object_query="table"))

    assert answered.status == "answered"
    assert answered.answer == "yes"
    assert answered.evidence[0].point_count == 3
    assert [item.tool for item in answered.trace] == [
        "detect_objects",
        "segment_objects",
        "get_foreground_geometry",
    ]
    assert rejected.status == "rejected"
    assert rejected.reason == "no_grounded_object"
    assert absent.status == "rejected"


def test_ground_truth_agent_rejects_small_masks() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=37),
    )

    result = agent.answer(frame, QuestionIntent(kind="presence", object_query="chair"))

    assert result.status == "rejected"


def test_ground_truth_agent_falls_back_to_point_prompt() -> None:
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
        config=GroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(frame, QuestionIntent(kind="presence", object_query="plant"))

    assert result.answer == "yes"
    assert [item.tool for item in result.trace] == [
        "detect_objects",
        "locate_object_point",
        "segment_object_point",
        "get_foreground_geometry",
    ]


def test_ground_truth_agent_compares_nearest_objects_by_side() -> None:
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
        config=GroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(
        frame, QuestionIntent(kind="compare_nearest_by_side", object_query="chair")
    )

    assert result.status == "answered"
    assert result.answer == "left"
    assert result.question.object_ids == ("frame-1-chair-0", "frame-1-chair-1")


def test_ground_truth_agent_rejects_side_comparison_without_both_sides() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=1),
    )

    result = agent.answer(
        frame, QuestionIntent(kind="compare_nearest_by_side", object_query="chair")
    )

    assert result.status == "rejected"
    assert result.reason == "missing_grounded_side"


def test_ground_truth_agent_buckets_visible_count_and_camera_range() -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=1),
    )

    count = agent.answer(frame, QuestionIntent(kind="visible_count", object_query="chair"))
    camera_range = agent.answer(frame, QuestionIntent(kind="camera_range", object_query="chair"))

    assert count.answer == "1-2"
    assert count.question.allowed_answers == ("1-2", "3-4", "5-7", "8+")
    assert camera_range.answer == "1 to under 2 m"
    assert camera_range.question.allowed_answers == (
        "under 1 m",
        "1 to under 2 m",
        "2 to under 4 m",
        "4 m or more",
    )


def test_ground_truth_agent_compares_ground_plane_relative_heights(monkeypatch: object) -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=1),
    )
    chair = GroundedObject("chair-0", "chair", 8, 1.0, "left")
    table = GroundedObject("table-0", "table", 8, 2.0, "right")
    plane = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)
    monkeypatch.setattr(
        agent,
        "ground",
        lambda _frame, query: ([chair] if query == "chair" else [table], ()),
    )
    monkeypatch.setattr(
        agent.primitives,
        "fit_ground_plane",
        lambda: type("Fit", (), {"estimate": plane, "rejection_reason": None})(),
    )
    monkeypatch.setattr(
        agent.primitives,
        "measure_height",
        lambda item, accepted_plane: HeightMeasurementResult(
            item,
            accepted_plane,
            OracleMeasurement(0.8 if item == chair else 0.5, "m", 0.05, (), ()),
            (),
        ),
    )

    result = agent.answer(
        frame,
        QuestionIntent(kind="compare_height", object_query="chair", comparison_query="table"),
    )

    assert result.status == "answered"
    assert result.answer == "chair"
    assert result.question.allowed_answers == ("chair", "table")


def test_ground_truth_agent_compares_pairwise_left_right(monkeypatch: object) -> None:
    frame, detection = _frame_and_detection()
    agent = _agent(
        frame,
        _Detector(frame.image, detection),
        _Segmenter(),
        config=GroundingConfig(min_mask_area_px=1),
    )
    chair = GroundedObject("chair-0", "chair", 8, 1.0, "left")
    table = GroundedObject("table-0", "table", 8, 2.0, "right")
    monkeypatch.setattr(
        agent,
        "ground",
        lambda _frame, query: ([chair] if query == "chair" else [table], ()),
    )
    monkeypatch.setattr(
        agent.primitives,
        "classify_horizontal_relation",
        lambda first, second: HorizontalRelationResult("left", ("camera_frame_support_centroids",)),
    )

    result = agent.answer(
        frame,
        QuestionIntent(kind="compare_left_right", object_query="chair", comparison_query="table"),
    )

    assert result.status == "answered"
    assert result.answer == "left"
    assert result.question.allowed_answers == ("left", "right")
