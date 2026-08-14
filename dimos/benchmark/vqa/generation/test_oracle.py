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

import json
from typing import Any, cast

from langchain_core.messages import AIMessage
import numpy as np
import pytest

from dimos.benchmark.vqa.contracts import (
    BooleanAnswerContract,
    CalibratedFrame,
    ChoiceAnswerContract,
    GroundedObject,
    OracleEvidence,
    OracleToolResult,
    PrimitiveGroundingConfig,
    QuestionProposal,
    RejectedOracleResult,
    VisualObject,
)
from dimos.benchmark.vqa.generation.agentic_answerer import (
    AgenticAnswerer,
    validate_oracle_answer,
)
from dimos.benchmark.vqa.generation.agentic_tools import VqaPrimitiveToolRegistry
from dimos.benchmark.vqa.generation.answer_choices import (
    camera_range_choice,
    count_choice,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.question_authors import (
    AGENTIC_QUESTION_PROMPT,
    AgenticQuestionAuthor,
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

    def visual_objects(self, query: str) -> list[Any]:
        return [VisualObject("visual-chair", query, 1.0, (0.0, 0.0, 1.0, 1.0), "left")]

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
        frame,
        _MaskDetector(mask),
        _IdentitySegmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
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
                        "name": "detect_objects",
                        "args": {"query": "chair"},
                        "id": "call-1",
                    }
                ],
            )
        return AIMessage(content='{"answer":"yes","evidence_ids":["visual:v1:visual-chair"]}')


class _ScriptedModel:
    def __init__(self, responses: list[AIMessage]) -> None:
        self._responses = responses

    def bind_tools(self, tools: Any) -> _ScriptedModel:
        return self

    def invoke(self, messages: Any) -> AIMessage:
        return self._responses.pop(0)


def test_freeform_question_author_parses_public_contract() -> None:
    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposals = AgenticQuestionAuthor(cast("OpenAIVlModel", _QuestionModel())).propose(image)

    assert proposals == [
        QuestionProposal("chair-presence", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    ]


def test_freeform_author_prompt_prioritizes_geometric_questions() -> None:
    assert "chooses its own sequence" in AGENTIC_QUESTION_PROMPT
    assert "camera range" in AGENTIC_QUESTION_PROMPT
    assert "visibly repeated object" in AGENTIC_QUESTION_PROMPT
    assert "diverse set of questions" in AGENTIC_QUESTION_PROMPT
    assert "visibility/presence" in AGENTIC_QUESTION_PROMPT


def test_freeform_question_author_assigns_missing_ids() -> None:
    class _ModelWithoutId:
        def query(self, image: Image, prompt: str) -> str:
            return '[{"question":"Is there a chair?","answer_contract":{"kind":"boolean"}}]'

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposals = AgenticQuestionAuthor(cast("OpenAIVlModel", _ModelWithoutId())).propose(image)

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
        AgenticQuestionAuthor(cast("OpenAIVlModel", _NumericContractModel())).propose(image)
    except ValueError as exc:
        assert "unsupported answer contract" in str(exc)
    else:
        raise AssertionError("numeric answer contract was accepted")


def test_freeform_question_author_normalizes_string_query() -> None:
    class _ModelWithStringHints:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"choice","choices":["under 0.5 m","0.5-1.0 m"]},'
                '"object_queries":"chair"}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposal = AgenticQuestionAuthor(cast("OpenAIVlModel", _ModelWithStringHints())).propose(image)[
        0
    ]

    assert proposal.object_queries == ("chair",)


def test_freeform_question_author_ignores_malformed_optional_queries() -> None:
    class _ModelWithMalformedHints:
        def query(self, image: Image, prompt: str) -> str:
            return (
                '[{"question":"How tall is the chair?","answer_contract":'
                '{"kind":"choice","choices":["under 0.5 m","0.5-1.0 m"]},'
                '"object_queries":{"query":"chair"}}]'
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposal = AgenticQuestionAuthor(cast("OpenAIVlModel", _ModelWithMalformedHints())).propose(
        image
    )[0]

    assert proposal.object_queries == ()


def test_freeform_question_author_retries_invalid_json_once() -> None:
    class _RetryingModel:
        def __init__(self) -> None:
            self.calls = 0

        def query(self, image: Image, prompt: str) -> str:
            self.calls += 1
            if self.calls == 1:
                return "not json"
            return '[{"question":"Is there a chair?","answer_contract":{"kind":"boolean"}}]'

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))
    model = _RetryingModel()

    proposals = AgenticQuestionAuthor(cast("OpenAIVlModel", model)).propose(image)

    assert len(proposals) == 1
    assert model.calls == 2


def test_freeform_question_author_allows_many_questions_and_repairs_duplicate_ids() -> None:
    class _ManyQuestionModel:
        def query(self, image: Image, prompt: str) -> str:
            return json.dumps(
                [
                    {
                        "id": "question",
                        "question": f"Is object {index} visible?",
                        "answer_contract": {"kind": "boolean"},
                    }
                    for index in range(20)
                ]
            )

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))

    proposals = AgenticQuestionAuthor(cast("OpenAIVlModel", _ManyQuestionModel())).propose(image)

    assert len(proposals) == 20
    assert proposals[0].id == "question"
    assert proposals[-1].id == "question-20"


def test_local_tool_returns_grounding_and_evidence_ids() -> None:
    registry = VqaPrimitiveToolRegistry(_frame_primitives(_measurement_frame()))

    detection = json.loads(registry.detect_objects("chair"))
    masks = json.loads(registry.segment_object(detection["detections"][0]["detection_id"]))
    payload = json.loads(registry.ground_mask(masks["mask_ids"][0]))

    assert "detection_id" not in detection
    assert payload["object_id"] == "object:v1:synthetic:0001"
    assert payload["objects"][0]["evidence_id"] == f"grounding:v1:{payload['object_id']}"
    assert registry.results[-1].version == "v1"


def test_detection_returns_citable_visual_evidence() -> None:
    registry = VqaPrimitiveToolRegistry(_frame_primitives(_measurement_frame()))

    payload = json.loads(registry.detect_objects("chair"))

    assert payload["objects"][0]["evidence_id"].startswith("visual:v1:visual-object:v1:")
    assert payload["objects"][0]["bbox"] == [0.0, 0.0, 99.0, 99.0]
    assert payload["objects"][0]["range_m"] is None


def test_empty_segmentation_is_an_explicit_rejection() -> None:
    class _EmptySegmenter:
        def segment(self, detections: Any) -> ImageDetections2D:
            return ImageDetections2D(detections.image, [])

    frame = _measurement_frame()
    primitives = FramePerceptionPrimitives(
        frame,
        _MaskDetector(np.full((100, 100), 255, dtype=np.uint8)),
        _EmptySegmenter(),
        config=PrimitiveGroundingConfig(min_mask_area_px=1),
    )
    registry = VqaPrimitiveToolRegistry(primitives)
    detection = json.loads(registry.detect_objects("chair"))

    payload = json.loads(registry.segment_object(detection["detections"][0]["detection_id"]))

    assert payload["mask_ids"] == []
    assert payload["rejection_reason"] == "no_segmentation_mask"


def test_grounding_reuses_canonical_object_across_queries_and_registries() -> None:
    primitives = _frame_primitives(_measurement_frame())
    chair_registry = VqaPrimitiveToolRegistry(primitives)
    seat_registry = VqaPrimitiveToolRegistry(primitives)

    chair_detection = json.loads(chair_registry.detect_objects("chair"))
    chair_masks = json.loads(
        chair_registry.segment_object(chair_detection["detections"][0]["detection_id"])
    )
    chair = json.loads(chair_registry.ground_mask(chair_masks["mask_ids"][0]))

    seat_detection = json.loads(seat_registry.detect_objects("seat"))
    seat_masks = json.loads(
        seat_registry.segment_object(seat_detection["detections"][0]["detection_id"])
    )
    seat = json.loads(seat_registry.ground_mask(seat_masks["mask_ids"][0]))

    assert chair["object_id"] == "object:v1:synthetic:0001"
    assert seat["object_id"] == chair["object_id"]
    assert seat["objects"][0]["label"] == "chair"


def test_grounding_reuses_canonical_object_for_repeated_mask() -> None:
    registry = VqaPrimitiveToolRegistry(_frame_primitives(_measurement_frame()))
    detection = json.loads(registry.detect_objects("chair"))
    masks = json.loads(registry.segment_object(detection["detections"][0]["detection_id"]))

    first = json.loads(registry.ground_mask(masks["mask_ids"][0]))
    second = json.loads(registry.ground_mask(masks["mask_ids"][0]))

    assert second["object_id"] == first["object_id"]


def test_grounding_does_not_merge_ambiguous_overlapping_masks() -> None:
    frame = _measurement_frame()
    full_mask = np.full((frame.image.height, frame.image.width), 255, dtype=np.uint8)
    partial_mask = full_mask.copy()
    partial_mask[:10] = 0
    primitives = _frame_primitives(frame, full_mask)

    first = primitives.ground_mask(
        Detection2DSeg((0.0, 0.0, 99.0, 99.0), 0, -1, 1.0, "chair", 0.0, frame.image, full_mask)
    )
    second = primitives.ground_mask(
        Detection2DSeg((0.0, 10.0, 99.0, 99.0), 1, -1, 1.0, "chair", 0.0, frame.image, partial_mask)
    )

    assert first is not None
    assert second is not None
    assert first.id != second.id


def test_local_registry_exposes_core_perception_tools() -> None:
    registry = VqaPrimitiveToolRegistry(cast("Any", _Grounding()))

    assert {tool.name for tool in registry.tools()} == {
        "detect_objects",
        "segment_object",
        "ground_mask",
    }


def test_count_and_camera_range_choices_are_fixed_and_non_overlapping() -> None:
    assert [count_choice(value) for value in (0, 1, 2, 3, 4, 5, 7, 8)] == [
        "0",
        "1-2",
        "1-2",
        "3-4",
        "3-4",
        "5-7",
        "5-7",
        "8+",
    ]
    with pytest.raises(ValueError, match="non-negative"):
        count_choice(-1)
    assert [camera_range_choice(value) for value in (0.99, 1.0, 2.0, 4.0)] == [
        "under 1 m",
        "1 to under 2 m",
        "2 to under 4 m",
        "4 m or more",
    ]


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


@pytest.mark.parametrize(
    ("answer", "expected"),
    [(True, "yes"), (False, "no"), ("TRUE", "yes"), (" No ", "no")],
)
def test_oracle_normalizes_boolean_answers(answer: Any, expected: str) -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract())
    result = OracleToolResult(
        "ground", "chair", (OracleEvidence("e1", "v1", "o1", "chair", 1.0, "left", 3),)
    )

    assert validate_oracle_answer(proposal, answer, ["e1"], (result,)) == expected


def test_oracle_normalizes_choice_answers() -> None:
    proposal = QuestionProposal("q", "Which side?", ChoiceAnswerContract(("left", "right")))
    result = OracleToolResult(
        "detect", "chair", (OracleEvidence("e1", "v1", "o1", "chair", None, "left", None),)
    )

    assert validate_oracle_answer(proposal, " LEFT. ", ["e1"], (result,)) == "left"


def test_private_oracle_runs_direct_structured_tool() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
    result = AgenticAnswerer(cast("Any", _BoundModel())).answer(proposal, registry)

    assert result.answer == "yes"
    assert result.evidence_ids == ("visual:v1:visual-chair",)
    assert result.trace[-1].operation == "tool"


def test_private_oracle_rejects_unknown_evidence() -> None:
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
    registry = VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
    result = AgenticAnswerer(cast("Any", _ChoiceModel())).answer(proposal, registry)

    assert result.reason == "invalid_final_answer:answer cites unknown evidence"


def test_private_oracle_allows_explicit_rejection_without_answer_resolution() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
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

    result = AgenticAnswerer(cast("Any", model)).answer(proposal, registry)

    assert isinstance(result, RejectedOracleResult)
    assert result.reason == "chair cannot be verified"
    assert result.tool_results == registry.results
    assert len(result.trace) == 1
    assert result.trace[0].operation == "tool"


def test_private_oracle_returns_tool_errors_to_model() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    registry = VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
    model = _ScriptedModel(
        [
            AIMessage(
                content="",
                tool_calls=[{"name": "unsupported", "args": {}, "id": "call-1"}],
            ),
            AIMessage(
                content='{"status":"rejected","reason":"tool unavailable","evidence_ids":[]}'
            ),
        ]
    )

    result = AgenticAnswerer(cast("Any", model)).answer(proposal, registry)

    assert isinstance(result, RejectedOracleResult)
    assert result.reason == "tool unavailable"
    assert result.trace[0].operation == "tool_error"


def test_private_oracle_rejects_malformed_tool_call_id() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract(), ("chair",))
    model = _ScriptedModel(
        [
            cast(
                "AIMessage",
                type(
                    "MalformedToolResponse",
                    (),
                    {
                        "content": "",
                        "tool_calls": [{"name": "detect_objects", "args": {"query": "chair"}}],
                    },
                )(),
            )
        ]
    )

    result = AgenticAnswerer(cast("Any", model)).answer(
        proposal, VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
    )

    assert isinstance(result, RejectedOracleResult)
    assert result.reason == "malformed_tool_call_id"


def test_private_oracle_rejects_malformed_explicit_rejection() -> None:
    proposal = QuestionProposal("q", "Is there a chair?", BooleanAnswerContract())
    model = _ScriptedModel([AIMessage(content='{"status":"rejected","reason":""}')])

    result = AgenticAnswerer(cast("Any", model)).answer(
        proposal, VqaPrimitiveToolRegistry(cast("Any", _Grounding()))
    )

    assert isinstance(result, RejectedOracleResult)
    assert result.reason.startswith("invalid_final_answer:")
