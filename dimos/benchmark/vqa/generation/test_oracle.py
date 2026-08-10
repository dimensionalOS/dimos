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
from dimos.benchmark.vqa.generation.primitives.choices import height_choice_window
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.geometry import estimate_ground_plane
from dimos.benchmark.vqa.generation.question_agent import (
    AGENTIC_QUESTION_PROMPT,
    OpenAIFreeformQuestionAuthor,
)
from dimos.benchmark.vqa.models import (
    BooleanAnswerContract,
    CalibratedFrame,
    ChoiceAnswerContract,
    DeferredHeightChoiceContract,
    GroundingConfig,
    OracleEvidence,
    OracleToolResult,
    QuestionProposal,
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
                tool_calls=[{"name": "detect_objects", "args": {"query": "chair"}, "id": "call-1"}],
            )
        if self._calls == 2:
            return AIMessage(
                content="",
                tool_calls=[
                    {
                        "name": "segment_detections",
                        "args": {"detection_id": "detection:v1:0001"},
                        "id": "call-2",
                    }
                ],
            )
        if self._calls == 3:
            return AIMessage(
                content="",
                tool_calls=[
                    {"name": "ground_masks", "args": {"mask_id": "mask:v1:0002"}, "id": "call-3"}
                ],
            )
        return AIMessage(
            content='{"answer":"yes","evidence_ids":["grounding:v1:synthetic-chair-0"]}'
        )


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
    assert "measure_height" in AGENTIC_QUESTION_PROMPT
    assert "two named objects is closer" in AGENTIC_QUESTION_PROMPT
    assert "closer (choice" in AGENTIC_QUESTION_PROMPT
    assert "Use visibility/presence questions only" in AGENTIC_QUESTION_PROMPT


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
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))

    detection = json.loads(registry.detect_objects("chair"))
    masks = json.loads(registry.segment_detections(detection["detection_id"]))
    payload = json.loads(registry.ground_masks(masks["mask_id"]))

    assert payload["objects"][0] == {
        "evidence_id": "grounding:v1:synthetic-chair-0",
        "id": "synthetic-chair-0",
        "label": "chair",
        "range_m": 1.0,
        "side": "left",
        "point_count": 4,
    }
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
    masks = json.loads(registry.segment_detections(detection["detection_id"]))
    grounded = json.loads(registry.ground_masks(masks["mask_id"]))
    plane = json.loads(registry.fit_ground_plane())
    payload = json.loads(registry.measure_height(grounded["object_ids"][0], plane["plane_id"]))

    assert payload["measurement"]["unit"] == "m"
    assert 0.8 < payload["measurement"]["value"] < 1.0
    assert payload["measurement"]["tolerance"] >= 0.05
    assert payload["objects"][0]["evidence_id"] == "height:v1:synthetic-chair-0"
    assert "visible_point_cloud_height" in payload["quality_flags"]


def test_local_registry_exposes_geometry_tools() -> None:
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))

    assert {tool.name for tool in registry.tools()} == {
        "detect_objects",
        "segment_detections",
        "ground_masks",
        "select_nearest_object",
        "fit_ground_plane",
        "measure_height",
        "bucket_measurement",
    }


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


def test_oracle_derives_deferred_height_answer_from_measurement_bucket() -> None:
    proposal = QuestionProposal(
        "q", "How tall is the chair?", DeferredHeightChoiceContract(), ("chair",)
    )
    evidence = OracleEvidence("height-1", "v1", "chair-1", "chair", 1.0, "left", 8)
    result = OracleToolResult(
        "bucket_measurement",
        "chair",
        (evidence,),
        choice="0.2-0.6 m",
        choices=("under 0.2 m", "0.2-0.6 m", "0.6-1.0 m", "over 1.0 m"),
    )

    assert validate_oracle_answer(proposal, "0.2-0.6 m", ["height-1"], (result,)) == "0.2-0.6 m"
    try:
        validate_oracle_answer(proposal, "under 0.2 m", ["height-1"], (result,))
    except ValueError as exc:
        assert "does not match measurement bucket" in str(exc)
    else:
        raise AssertionError("non-derived deferred height answer was accepted")


def test_private_oracle_runs_direct_structured_tool() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = LocalOracleToolRegistry(cast("Any", _Grounding()))
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


def test_agentic_oracle_never_uses_legacy_answer_program() -> None:
    class _GroundingWithoutLegacyAnswer(_Grounding):
        def answer(self, frame: Any, intent: Any) -> None:
            raise AssertionError("agentic oracle must not call legacy answer")

    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = LocalOracleToolRegistry(cast("Any", _GroundingWithoutLegacyAnswer()))
    validator = _SemanticValidator(SemanticEvidenceValidation(True, "chair grounding supports yes"))

    result = PrivateToolCallingOracle(
        cast("Any", _BoundModel()), semantic_validator=validator
    ).answer(proposal, registry)

    assert result.answer == "yes"
