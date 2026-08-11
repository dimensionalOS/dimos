# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import json
from typing import Any, cast

from langchain_core.messages import AIMessage
import numpy as np

from dimos.benchmark.vqa.generation.geometry import project_visible_points
from dimos.benchmark.vqa.generation.oracle import (
    PrivateToolCallingOracle,
    SemanticEvidenceValidation,
    validate_oracle_answer,
)
from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry
from dimos.benchmark.vqa.generation.primitives.choices import (
    camera_range_choice,
    count_choice,
    height_choice_window,
)
from dimos.benchmark.vqa.generation.primitives.contracts import (
    HeightMeasurementResult,
    HorizontalRelationResult,
    ObjectOnSupportResult,
    OpeningWidthResult,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.geometry import (
    ForwardCorridorMeasurement,
    PlaneFitResult,
    classify_forward_corridor,
    estimate_ground_plane,
    measure_opening_width,
    measure_relative_plane_angle,
)
from dimos.benchmark.vqa.generation.question_agent import (
    AGENTIC_QUESTION_PROMPT,
    OpenAIFreeformQuestionAuthor,
)
from dimos.benchmark.vqa.models import (
    BooleanAnswerContract,
    CalibratedFrame,
    ChoiceAnswerContract,
    DeferredHeightChoiceContract,
    GroundedObject,
    GroundingConfig,
    GroundPlaneEstimate,
    OracleEvidence,
    OracleMeasurement,
    OracleToolResult,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class _QuestionModel:
    def query(self, image: Image, prompt: str) -> str:
        assert "answer_contract" in prompt
        return '[{"id":"chair-presence","question":"Is there a chair?","answer_contract":{"kind":"boolean"},"object_queries":["chair"]}]'


class _Grounding:
    frame = cast("Any", object())

    def detect_objects(self, query: str) -> list[Any]:
        return []

    def segment_detections(self, query: str) -> list[Any]:
        return []

    def ground_masks(self, query: str) -> list[Any]:
        return [
            type(
                "Object",
                (),
                {
                    "id": "synthetic-chair-0",
                    "label": query,
                    "range_m": 1.0,
                    "horizontal_direction": "left",
                    "point_count": 4,
                },
            )()
        ]


def _measurement_frame(points: np.ndarray | None = None) -> CalibratedFrame:
    ground = [[x, 1.0, z] for z in (3.0, 4.0, 5.0) for x in (-1.0, -0.5, 0.0, 0.5, 1.0)]
    object_points = [[2.0, 1.0 - height, 4.0] for height in np.linspace(0.2, 1.0, 9)]
    image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    return CalibratedFrame(
        id="synthetic",
        image=image,
        pointcloud=PointCloud2.from_numpy(
            points
            if points is not None
            else np.asarray([*ground, *object_points], dtype=np.float32)
        ),
        camera_info=CameraInfo.from_intrinsics(50.0, 50.0, 50.0, 50.0, 100, 100),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )


class _MaskDetector:
    def __init__(self, mask: np.ndarray) -> None:
        self._mask = mask

    def detect(self, image: Image, query: str) -> ImageDetections2D:
        detection = Detection2DSeg(
            (0.0, 0.0, float(image.width - 1), float(image.height - 1)),
            0,
            -1,
            1.0,
            query,
            0.0,
            image,
            self._mask,
        )
        return ImageDetections2D(image, [detection])


class _IdentitySegmenter:
    def segment(self, detections: Any) -> Any:
        return detections


def _frame_primitives(
    frame: CalibratedFrame, mask: np.ndarray | None = None
) -> FramePerceptionPrimitives:
    if mask is None:
        mask = np.full((frame.image.height, frame.image.width), 255, dtype=np.uint8)
    return FramePerceptionPrimitives(
        frame, _MaskDetector(mask), _IdentitySegmenter(), config=GroundingConfig(min_mask_area_px=1)
    )


class _BoundModel:
    def __init__(self) -> None:
        self._calls = 0

    def bind_tools(self, tools: Any) -> _BoundModel:
        return self

    def invoke(self, messages: Any) -> AIMessage:
        self._calls += 1
        if self._calls == 1:
            return AIMessage(
                content="",
                tool_calls=[
                    {
                        "name": "get_object_pose",
                        "args": {"object_id": "synthetic-chair-0"},
                        "id": "call-2",
                    }
                ],
            )
        return AIMessage(
            content='{"answer":"yes","evidence_ids":["grounding:v1:synthetic-chair-0"]}'
        )


class _ScriptedModel:
    def __init__(self, responses: list[AIMessage]) -> None:
        self._responses = responses

    def bind_tools(self, tools: Any) -> _ScriptedModel:
        return self

    def invoke(self, messages: Any) -> AIMessage:
        return self._responses.pop(0)


class _SemanticValidator:
    def __init__(self, verdict: SemanticEvidenceValidation) -> None:
        self.verdict = verdict
        self.calls: list[tuple[QuestionProposal, str | float, tuple[OracleToolResult, ...]]] = []

    def validate(
        self,
        proposal: QuestionProposal,
        answer: str | float,
        cited_results: tuple[OracleToolResult, ...],
    ) -> SemanticEvidenceValidation:
        self.calls.append((proposal, answer, cited_results))
        return self.verdict


def test_freeform_question_author_parses_public_contract() -> None:
    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposals = OpenAIFreeformQuestionAuthor(cast("OpenAIVlModel", _QuestionModel())).propose(image)

    assert proposals == [
        QuestionProposal(
            "chair-presence", "Is there a chair?", BooleanAnswerContract(), ("chair",), ()
        )
    ]


def test_freeform_author_prompt_prioritizes_geometric_questions() -> None:
    assert "chooses its own sequence" in AGENTIC_QUESTION_PROMPT
    assert "closest to a named target" in AGENTIC_QUESTION_PROMPT
    assert "visibly repeated object" in AGENTIC_QUESTION_PROMPT
    assert "diverse set of questions" in AGENTIC_QUESTION_PROMPT
    assert "visibility/presence" in AGENTIC_QUESTION_PROMPT


def test_freeform_question_author_assigns_missing_ids() -> None:
    class _ModelWithoutId:
        def query(self, image: Image, prompt: str) -> str:
            return '[{"question":"Is there a chair?","answer_contract":{"kind":"boolean"}}]'

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposals = OpenAIFreeformQuestionAuthor(cast("OpenAIVlModel", _ModelWithoutId())).propose(
        image
    )

    assert proposals[0].id == "proposal-01"


def test_freeform_question_author_rejects_numeric_contracts() -> None:
    class _NumericContractModel:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"numeric","unit":"m","tolerance":0.1}}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    try:
        OpenAIFreeformQuestionAuthor(cast("OpenAIVlModel", _NumericContractModel())).propose(image)
    except ValueError as exc:
        assert "unsupported answer contract" in str(exc)
    else:
        raise AssertionError("numeric answer contract was accepted")


def test_freeform_question_author_parses_deferred_height_contract() -> None:
    class _HeightContractModel:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"deferred_height_choice","strategy":"height-window-v1"},'
                '"object_queries":["chair"]}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposal = OpenAIFreeformQuestionAuthor(cast("OpenAIVlModel", _HeightContractModel())).propose(
        image
    )[0]

    assert proposal.answer_contract == DeferredHeightChoiceContract()
    assert proposal.object_queries == ("chair",)


def test_freeform_question_author_normalizes_optional_query_hints() -> None:
    class _ModelWithStringHints:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"choice","choices":["under 0.5 m","0.5-1.0 m"]},'
                '"object_queries":"chair","tool_hints":null}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposal = OpenAIFreeformQuestionAuthor(cast("OpenAIVlModel", _ModelWithStringHints())).propose(
        image
    )[0]

    assert proposal.object_queries == ("chair",)
    assert proposal.tool_hints == ()


def test_freeform_question_author_ignores_malformed_optional_query_hints() -> None:
    class _ModelWithMalformedHints:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"choice","choices":["under 0.5 m","0.5-1.0 m"]},'
                '"object_queries":{"query":"chair"}}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposal = OpenAIFreeformQuestionAuthor(
        cast("OpenAIVlModel", _ModelWithMalformedHints())
    ).propose(image)[0]

    assert proposal.object_queries == ()


def test_local_tool_returns_geometry_and_evidence_ids() -> None:
    registry = LocalOracleToolRegistry(_frame_primitives(_measurement_frame()))

    detection = json.loads(registry.detect_objects("chair"))
    masks = json.loads(registry.segment_detection(detection["detections"][0]["detection_id"]))
    payload = json.loads(registry.ground_mask(masks["mask_ids"][0]))

    assert "detection_id" not in detection
    assert payload["object_id"].startswith("object:v1:")
    assert payload["objects"][0]["evidence_id"] == f"grounding:v1:{payload['object_id']}"
    assert registry.results[-1].version == "v1"


def test_ground_plane_estimator_fits_visible_lower_band() -> None:
    fit = estimate_ground_plane(_measurement_frame())

    assert fit.rejection_reason is None
    assert fit.estimate is not None
    assert fit.estimate.inlier_count >= 12
    assert fit.estimate.residual_m < 0.001
    assert np.allclose(fit.estimate.normal, (0.0, -1.0, 0.0), atol=0.001)
    assert fit.estimate.offset_m == 1.0


def test_ground_plane_tool_returns_quality_gated_rejection() -> None:
    frame = _measurement_frame(np.asarray([[0.0, 1.0, 3.0]], dtype=np.float32))
    registry = LocalOracleToolRegistry(_frame_primitives(frame))

    payload = json.loads(registry.fit_ground_plane())

    assert payload["measurement"] is None
    assert payload["rejection_reason"] == "insufficient_support"
    assert "insufficient_ground_band_points" in payload["quality_flags"]


def test_height_tool_measures_visible_object_points_above_plane() -> None:
    frame = _measurement_frame()
    mask = np.zeros((100, 100), dtype=np.uint8)
    projected = project_visible_points(frame)
    for (x, y), point in zip(projected.pixels, projected.camera_points, strict=True):
        if point[0] > 1.5:
            mask[y, x] = 255
    registry = LocalOracleToolRegistry(_frame_primitives(frame, mask))

    detection = json.loads(registry.detect_objects("chair"))
    masks = json.loads(registry.segment_detection(detection["detections"][0]["detection_id"]))
    grounded = json.loads(registry.ground_mask(masks["mask_ids"][0]))
    plane = json.loads(registry.fit_ground_plane())
    payload = json.loads(registry.measure_height(grounded["object_id"], plane["plane_id"]))

    assert payload["measurement"]["unit"] == "m"
    assert 0.8 < payload["measurement"]["value"] < 1.0
    assert payload["measurement"]["tolerance"] >= 0.05
    assert payload["objects"][0]["evidence_id"] == f"height:v1:{grounded['object_id']}"
    assert "visible_point_cloud_height" in payload["quality_flags"]


def test_local_registry_exposes_geometry_tools() -> None:
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))

    assert {tool.name for tool in registry.tools()} == {
        "detect_objects",
        "segment_detection",
        "ground_mask",
        "fit_ground_plane",
        "get_object_pose",
        "fit_object_surface_plane",
        "fit_mask_surrounding_plane",
        "measure_object_pair_distance",
        "measure_relative_plane_angle",
        "measure_object_plane_relation",
        "measure_aperture_geometry",
        "measure_forward_corridor",
        "measure_height",
    }


def test_local_registry_exposes_forward_corridor_metrics(monkeypatch: Any) -> None:
    primitives = _frame_primitives(_measurement_frame())
    registry = LocalOracleToolRegistry(primitives)
    plane = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)
    registry._planes["ground"] = plane
    registry._ground_planes.add("ground")
    monkeypatch.setattr(
        primitives,
        "measure_forward_corridor",
        lambda accepted_plane: ForwardCorridorMeasurement((3, 4, 5), 6, 24),
    )

    payload = json.loads(registry.measure_forward_corridor("ground"))
    rejected = json.loads(registry.measure_forward_corridor("unknown"))

    assert payload["metrics"] == {
        "ground_band_1_count": 3.0,
        "ground_band_2_count": 4.0,
        "ground_band_3_count": 5.0,
        "elevated_obstacle_count": 6.0,
        "corridor_point_count": 24.0,
    }
    assert payload["objects"][0]["evidence_id"] == "forward-corridor:v1:synthetic"
    assert rejected["rejection_reason"] == "unknown_ground_plane_id"


def test_height_choice_window_is_local_and_deterministic() -> None:
    choices, answer = height_choice_window(0.42)

    assert choices == (
        "under 0.2 m",
        "0.2-0.6 m",
        "0.6-1.0 m",
        "over 1.0 m",
    )
    assert answer == "0.2-0.6 m"
    assert height_choice_window(3.0)[1] == "over 2.0 m"


def test_count_and_camera_range_choices_are_fixed_and_non_overlapping() -> None:
    assert [count_choice(value) for value in (1, 2, 3, 4, 5, 7, 8)] == [
        "1-2",
        "1-2",
        "3-4",
        "3-4",
        "5-7",
        "5-7",
        "8+",
    ]
    assert [camera_range_choice(value) for value in (0.99, 1.0, 2.0, 4.0)] == [
        "under 1 m",
        "1 to under 2 m",
        "2 to under 4 m",
        "4 m or more",
    ]


def test_local_registry_buckets_count_range_and_pairwise_side() -> None:
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))
    registry._objects["synthetic-chair-0"] = GroundedObject(
        "synthetic-chair-0", "chair", 4, 1.0, "left"
    )
    left = GroundedObject("left", "chair", 8, 1.0, "left")
    right = GroundedObject("right", "chair", 8, 2.0, "right")
    registry._objects = {left.id: left, right.id: right}

    count = json.loads(registry.count_grounded_objects([left.id, right.id]))
    camera_range = json.loads(registry.bucket_camera_range(right.id))
    side = json.loads(registry.compare_nearest_by_side([left.id, right.id]))

    assert count["choice"] == "1-2"
    assert camera_range["choice"] == "2 to under 4 m"
    assert side["choice"] == "left"


def test_local_registry_compares_pairwise_relation_and_height(monkeypatch: Any) -> None:
    primitives = _frame_primitives(_measurement_frame())
    registry = LocalOracleToolRegistry(primitives)
    chair = GroundedObject("chair", "chair", 8, 1.0, "left")
    table = GroundedObject("table", "table", 8, 2.0, "right")
    plane = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)
    registry._objects = {chair.id: chair, table.id: table}
    registry._planes = {"plane": plane}
    monkeypatch.setattr(
        primitives,
        "classify_horizontal_relation",
        lambda first, second: HorizontalRelationResult("left", ("camera_frame_support_centroids",)),
    )
    monkeypatch.setattr(
        primitives,
        "measure_height",
        lambda item, accepted_plane: HeightMeasurementResult(
            item,
            accepted_plane,
            OracleMeasurement(0.8 if item == chair else 0.5, "m", 0.05, (), ()),
            (),
        ),
    )

    relation = json.loads(registry.compare_left_right(chair.id, table.id))
    height = json.loads(registry.compare_heights(chair.id, table.id, "plane"))

    assert relation["choice"] == "left"
    assert height["choice"] == "chair"


def test_local_registry_verifies_support_and_measures_opening(monkeypatch: Any) -> None:
    primitives = _frame_primitives(_measurement_frame())
    registry = LocalOracleToolRegistry(primitives)
    box = GroundedObject("box", "box", 8, 1.0, "left")
    table = GroundedObject("table", "table", 8, 2.0, "right")
    registry._objects = {box.id: box, table.id: table}
    registry._masks = {"mask": "doorway"}
    measurement = OracleMeasurement(0.9, "m", 0.05, (), ())
    monkeypatch.setattr(
        primitives,
        "classify_object_on_support",
        lambda item, support: ObjectOnSupportResult(
            item, support, ("contact_band_supported",), answer="yes"
        ),
    )
    monkeypatch.setattr(
        primitives,
        "measure_opening_width",
        lambda query: OpeningWidthResult(measurement, ("stable_opening_scanlines",)),
    )

    support = json.loads(registry.classify_object_on_support(box.id, table.id))
    opening = json.loads(registry.measure_opening_width("mask"))

    assert support["choice"] == "yes"
    assert opening["measurement"]["value"] == 0.9


def test_opening_width_uses_wall_geometry_outside_the_aperture() -> None:
    image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    camera_info = CameraInfo.from_intrinsics(50.0, 50.0, 50.0, 50.0, 100, 100)
    opening = np.zeros((100, 100), dtype=np.uint8)
    opening[35:61, 40:61] = 255
    points: list[list[float]] = []
    for y in range(20, 61, 4):
        for x in range(28, 73, 4):
            if opening[y, x]:
                continue
            points.append([(x - 50) / 50 * 5.0, (y - 50) / 50 * 5.0, 5.0])
    for y in (65, 75, 85, 95):
        for x in range(0, 100, 10):
            depth = 1.0 / ((y - 50) / 50)
            points.append([(x - 50) / 50 * depth, 1.0, depth])
    frame = CalibratedFrame(
        "opening",
        image,
        PointCloud2.from_numpy(np.asarray(points, dtype=np.float32)),
        camera_info,
        Transform.identity(),
        True,
    )
    ground = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)

    result = measure_opening_width(frame, opening, ground)

    assert result.rejection_reason is None
    assert result.width_m is not None
    assert 1.9 < result.width_m < 2.1


def test_relative_plane_angle_measurement_is_unsigned() -> None:
    door = GroundPlaneEstimate((1.0, 0.0, 0.0), 0.0, 20, 20, 0.01)
    closed = GroundPlaneEstimate((1.0, 0.0, 0.0), 0.0, 20, 20, 0.01)
    open_door = GroundPlaneEstimate((0.0, 0.0, 1.0), 0.0, 20, 20, 0.01)
    ajar_door = GroundPlaneEstimate((0.95, 0.0, 0.31), 0.0, 20, 20, 0.01)

    assert measure_relative_plane_angle(door, closed).value == 0.0
    assert measure_relative_plane_angle(door, open_door).value == 90.0
    assert 17.0 < measure_relative_plane_angle(door, ajar_door).value < 19.0


def test_closest_object_uses_point_cloud_centroids_and_rejects_ties(monkeypatch: Any) -> None:
    target = GroundedObject("target", "chair", 8, 1.0, "left")
    close = GroundedObject("close", "table", 8, 2.0, "center")
    far = GroundedObject("far", "lamp", 8, 3.0, "right")
    primitives = _frame_primitives(_measurement_frame())
    centers = {
        "target": np.zeros((6, 3)),
        "close": np.tile((1.0, 0.0, 0.0), (6, 1)),
        "far": np.tile((2.0, 0.0, 0.0), (6, 1)),
    }
    monkeypatch.setattr(primitives, "_object_points", lambda item: centers[item.id])

    selected = primitives.select_closest_object(target, [close, far])

    assert selected.object == close
    assert selected.distance_m == 1.0
    monkeypatch.setattr(
        primitives,
        "_object_points",
        lambda item: np.tile((1.0, 0.0, 0.0), (6, 1)) if item.id != "target" else centers["target"],
    )
    assert (
        primitives.select_closest_object(target, [close, far]).rejection_reason
        == "ambiguous_object_proximity"
    )


def test_horizontal_relation_uses_camera_frame_support_centroids(monkeypatch: Any) -> None:
    left = GroundedObject("left", "chair", 8, 1.0, "left")
    right = GroundedObject("right", "table", 8, 2.0, "right")
    primitives = _frame_primitives(_measurement_frame())
    centers = {
        "left": np.tile((-0.3, 0.0, 1.0), (6, 1)),
        "right": np.tile((0.3, 0.0, 1.0), (6, 1)),
    }
    monkeypatch.setattr(primitives, "_object_points", lambda item: centers[item.id])

    assert primitives.classify_horizontal_relation(left, right).relation == "left"
    monkeypatch.setattr(primitives, "_object_points", lambda item: np.zeros((6, 3)))
    assert (
        primitives.classify_horizontal_relation(left, right).rejection_reason
        == "ambiguous_horizontal_relation"
    )


def test_object_on_support_keeps_clearly_separated_objects_as_no(monkeypatch: Any) -> None:
    box = GroundedObject("box", "box", 8, 1.0, "left")
    support = GroundedObject("table", "table", 8, 2.0, "right")
    plane = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)
    primitives = _frame_primitives(_measurement_frame())
    points = {
        box.id: np.tile((2.0, 0.5, 4.0), (6, 1)),
        support.id: np.tile((0.0, 1.0, 4.0), (6, 1)),
    }
    monkeypatch.setattr(primitives, "_object_points", lambda item: points[item.id])
    monkeypatch.setattr(primitives, "fit_ground_plane", lambda: PlaneFitResult(plane, ()))
    monkeypatch.setattr(
        "dimos.benchmark.vqa.generation.primitives.frame.fit_surface_plane",
        lambda support_points: PlaneFitResult(plane, ("surface_plane_accepted",)),
    )

    result = primitives.classify_object_on_support(box, support)

    assert result.answer == "no"
    assert "objects_separated_in_plane" in result.quality_flags


def test_forward_corridor_requires_ground_support_and_detects_obstacles() -> None:
    ground = GroundPlaneEstimate((0.0, -1.0, 0.0), 1.0, 20, 20, 0.01)
    floor = np.asarray(
        [
            [0.0, 1.0, depth]
            for depth in (0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8)
        ]
    )

    assert classify_forward_corridor(floor, ground)[0] == "clear"
    obstacle = np.repeat(np.asarray([[0.0, 0.5, 1.0]]), 4, axis=0)
    assert classify_forward_corridor(np.vstack((floor, obstacle)), ground)[0] == "blocked"


def test_oracle_validates_evidence_and_answer_contract() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract())
    result = OracleToolResult(
        "ground", "chair", (OracleEvidence("e1", "v1", "o1", "chair", 1.0, "left", 3),)
    )

    assert validate_oracle_answer(proposal, "yes", ["e1"], (result,)) == "yes"
    try:
        validate_oracle_answer(proposal, "maybe", ["e1"], (result,))
    except ValueError as exc:
        assert "boolean" in str(exc)
    else:
        raise AssertionError("invalid boolean answer was accepted")
    try:
        validate_oracle_answer(proposal, "yes", ["unknown"], (result,))
    except ValueError as exc:
        assert "unknown evidence" in str(exc)
    else:
        raise AssertionError("unknown evidence was accepted")


def test_oracle_derives_deferred_height_answer_from_cited_measurement() -> None:
    proposal = QuestionProposal(
        "q", "How tall is the chair?", DeferredHeightChoiceContract(), ("chair",)
    )
    measurement = OracleMeasurement(0.42, "m", 0.05, (), ())
    evidence = OracleEvidence("height-1", "v1", "chair-1", "chair", 1.0, "left", 8, measurement)
    result = OracleToolResult(
        "measure_height",
        "chair",
        (evidence,),
        measurement=measurement,
    )

    assert validate_oracle_answer(proposal, None, ["height-1"], (result,)) == "0.2-0.6 m"
    try:
        validate_oracle_answer(proposal, None, [], (result,))
    except ValueError as exc:
        assert "evidence_ids" in str(exc)
    else:
        raise AssertionError("deferred height answer without evidence was accepted")


def test_private_oracle_runs_direct_structured_tool() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))
    registry._objects["synthetic-chair-0"] = GroundedObject(
        "synthetic-chair-0", "chair", 4, 1.0, "left"
    )
    validator = _SemanticValidator(SemanticEvidenceValidation(True, "chair grounding supports yes"))

    result = PrivateToolCallingOracle(
        cast("Any", _BoundModel()), semantic_validator=validator
    ).answer(proposal, registry)

    assert result.answer == "yes"
    assert result.evidence_ids == ("grounding:v1:synthetic-chair-0",)
    assert validator.calls[0][2][-1].evidence[0].id == "grounding:v1:synthetic-chair-0"
    assert result.trace[-1].detail == "accepted:chair grounding supports yes"


def test_private_oracle_rejects_unsupported_measurement_claim() -> None:
    class _ChoiceModel(_BoundModel):
        def invoke(self, messages: Any) -> AIMessage:
            self._calls += 1
            if self._calls == 1:
                return AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "name": "detect_objects",
                            "args": {"query": "chair"},
                            "id": "call-1",
                        }
                    ],
                )
            return AIMessage(
                content='{"answer":"0.5-1.0 m","evidence_ids":["grounding:v1:synthetic-chair-0"]}'
            )

    proposal = QuestionProposal(
        "q", "How tall is the chair?", ChoiceAnswerContract(("under 0.5 m", "0.5-1.0 m"))
    )
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))
    validator = _SemanticValidator(
        SemanticEvidenceValidation(False, "range and side do not measure height")
    )

    result = PrivateToolCallingOracle(
        cast("Any", _ChoiceModel()), semantic_validator=validator
    ).answer(proposal, registry)

    assert result.reason == "invalid_final_answer:answer cites unknown evidence"


def test_private_oracle_allows_explicit_rejection_without_answer_resolution() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))
    validator = _SemanticValidator(SemanticEvidenceValidation(True, "should not be called"))
    model = _ScriptedModel(
        [
            AIMessage(
                content="",
                tool_calls=[{"name": "detect_objects", "args": {"query": "chair"}, "id": "call-1"}],
            ),
            AIMessage(
                content='{"status":"rejected","reason":"chair cannot be verified","evidence_ids":[]}'
            ),
        ]
    )

    result = PrivateToolCallingOracle(cast("Any", model), semantic_validator=validator).answer(
        proposal, registry
    )

    assert isinstance(result, RejectedOracleResult)
    assert result.reason == "chair cannot be verified"
    assert result.tool_results == registry.results
    assert len(result.trace) == 1
    assert result.trace[0].operation == "tool"
    assert validator.calls == []


def test_private_oracle_rejects_malformed_explicit_rejection() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract())
    validator = _SemanticValidator(SemanticEvidenceValidation(True, "should not be called"))
    model = _ScriptedModel([AIMessage(content='{"status":"rejected","reason":""}')])

    result = PrivateToolCallingOracle(cast("Any", model), semantic_validator=validator).answer(
        proposal, LocalOracleToolRegistry(cast("Any", _Grounding()))
    )

    assert isinstance(result, RejectedOracleResult)
    assert result.reason.startswith("invalid_final_answer:")
    assert validator.calls == []


def test_agentic_oracle_never_uses_legacy_answer_program() -> None:
    class _GroundingWithoutLegacyAnswer(_Grounding):
        def answer(self, frame: Any, intent: Any) -> None:
            raise AssertionError("agentic oracle must not call legacy answer")

    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = LocalOracleToolRegistry(cast("Any", _GroundingWithoutLegacyAnswer()))
    registry._objects["synthetic-chair-0"] = GroundedObject(
        "synthetic-chair-0", "chair", 4, 1.0, "left"
    )
    validator = _SemanticValidator(SemanticEvidenceValidation(True, "chair grounding supports yes"))

    result = PrivateToolCallingOracle(
        cast("Any", _BoundModel()), semantic_validator=validator
    ).answer(proposal, registry)

    assert result.answer == "yes"
