# Copyright 2026 Dimensional Inc.
"""Image-only constrained VQA question proposal."""

from __future__ import annotations

import json
import re
from typing import Any

from dimos.benchmark.vqa.models import (
    AnswerContract,
    BooleanAnswerContract,
    ChoiceAnswerContract,
    DeferredHeightChoiceContract,
    QuestionIntent,
    QuestionProposal,
)
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image

QUESTION_PROMPT = """Select up to 5 challenging but visually well-supported single-frame VQA intents.
Inspect only this image. Do not assume depth, point clouds, calibration, metadata, or temporal context.
Return JSON only: an array of objects with kind, object_query, and threshold_m only for
within_distance, plus candidate_queries only for closest_object. kind must be one of presence,
horizontal_direction, within_distance, compare_nearest_by_side, door_state, or closest_object.
Use threshold_m: 3.0 for within_distance.
Use image context to select only intents likely to produce a useful geometric case: emit
compare_nearest_by_side only when at least two visible instances of the same object appear on
opposite image sides; emit horizontal_direction only for a visible object; and prefer relational
or directional intents over presence. Emit door_state only for a clearly visible door with nearby
visible structure. Diversify object classes and question families when the scene supports them.
Emit closest_object only when one target and at least two distinct candidate object types are
visible; candidate_queries must name the visible candidate types.
Do not return bare object names, floors, walls, ceilings, background surfaces,
questions, answers, explanations, Markdown, or information not visible in the image."""

AGENTIC_QUESTION_PROMPT = """Author up to 5 challenging, visually answerable single-frame VQA questions.
Inspect only this image. Do not use or infer depth, point clouds, calibration, metadata, or answers.
Return JSON only: an array of objects with question, answer_contract, optional object_queries,
and optional tool_hints. answer_contract is {"kind":"boolean"}, {"kind":"choice","choices":[...]},
or {"kind":"deferred_height_choice","strategy":"height-window-v1"}.
Prioritize questions that a private point-cloud oracle can validate. For the height of one upright
object resting on visible ground, use deferred_height_choice. Its choices are generated privately
from a successful height measurement. Aim for a diverse set of questions that use the visible scene
composition: relative left/center/right position, which listed object is closest to a named target,
count choices for a visibly repeated object, and height for an upright grounded object. For
closest-object questions, use distinct visible candidate object types as the fixed choices. Use concise,
mutually exclusive fixed choices with two to four options. For a clearly visible door with nearby
visible structure, you may ask whether it is open or closed with exactly ["open", "closed"]. Use
object_queries for every referenced
object and tool_hints from "detect_objects", "segment_detections", "ground_masks",
"fit_ground_plane", "select_nearest_object", "measure_height", "classify_door_state", or
"select_closest_object", "bucket_measurement" when applicable. Use visibility/presence questions only when no stronger
geometric question is available.
Do not ask about color, material, text, intent, full physical size, hidden parts, or exact
metric distances without a supplied choice contract. Use only visible objects. Do not include
answers, explanations, Markdown, or background surfaces."""

_UNSUPPORTED_QUERIES = {"background", "ceiling", "floor", "ground", "room", "wall"}


class OpenAIQuestionAgent:
    """Propose constrained VQA intents from an image without geometry access."""

    def __init__(self, model: OpenAIVlModel) -> None:
        self._model = model

    def propose(self, image: Image) -> list[QuestionIntent]:
        try:
            payload: Any = _parse_json_array(self._model.query(image, QUESTION_PROMPT))
        except json.JSONDecodeError as exc:
            raise ValueError("question agent did not return JSON") from exc
        if not isinstance(payload, list) or len(payload) > 5:
            raise ValueError("question agent must return an array of at most five intents")
        intents: list[QuestionIntent] = []
        if all(isinstance(item, str) for item in payload):
            return [
                intent
                for query in dict.fromkeys(item.strip() for item in payload if item.strip())
                if query.lower() not in _UNSUPPORTED_QUERIES
                for intent in _intents_for_query(query)
            ]
        for item in payload:
            if not isinstance(item, dict):
                raise ValueError("question intent must be an object")
            kind, query, threshold = (
                item.get("kind"),
                item.get("object_query"),
                item.get("threshold_m"),
            )
            if kind not in (
                "presence",
                "horizontal_direction",
                "within_distance",
                "compare_nearest_by_side",
                "door_state",
                "closest_object",
            ):
                raise ValueError(f"unsupported question kind: {kind!r}")
            if not isinstance(query, str) or not query:
                raise ValueError("question intent requires object_query")
            if kind == "within_distance" and (
                not isinstance(threshold, (int, float)) or threshold <= 0
            ):
                raise ValueError("within_distance requires a positive threshold_m")
            if kind != "within_distance":
                threshold = None
            candidates = _string_tuple(item.get("candidate_queries"), "candidate_queries")
            if kind == "closest_object" and (len(candidates) < 2 or query in candidates):
                raise ValueError("closest_object requires two distinct candidate_queries")
            intents.append(QuestionIntent(kind, query, threshold, candidates))
        return intents


class OpenAIFreeformQuestionAuthor:
    """Image-only author for generic public questions and answer contracts."""

    def __init__(self, model: OpenAIVlModel, max_questions: int = 5) -> None:
        self._model = model
        self._max_questions = max_questions

    def propose(self, image: Image) -> list[QuestionProposal]:
        try:
            payload: Any = _parse_json_array(self._model.query(image, AGENTIC_QUESTION_PROMPT))
        except json.JSONDecodeError as exc:
            raise ValueError("question author did not return JSON") from exc
        if not isinstance(payload, list) or len(payload) > self._max_questions:
            raise ValueError("question author returned too many questions")
        proposals = [
            _proposal_from_json(item, index) for index, item in enumerate(payload, start=1)
        ]
        if len({item.id for item in proposals}) != len(proposals):
            raise ValueError("question ids must be unique")
        return proposals


def _proposal_from_json(item: Any, index: int) -> QuestionProposal:
    if not isinstance(item, dict):
        raise ValueError("question proposal must be an object")
    identifier, question = item.get("id"), item.get("question")
    if not isinstance(question, str) or not question:
        raise ValueError("question proposal requires question")
    if not isinstance(identifier, str) or not identifier:
        identifier = f"proposal-{index:02d}"
    queries = _string_tuple(item.get("object_queries", []), "object_queries")
    hints = _string_tuple(item.get("tool_hints", []), "tool_hints")
    contract = item.get("answer_contract")
    if not isinstance(contract, dict):
        raise ValueError("question proposal requires answer_contract")
    kind = contract.get("kind")
    if kind == "boolean":
        answer_contract: AnswerContract = BooleanAnswerContract()
    elif kind == "choice":
        choices = _string_tuple(contract.get("choices"), "choices")
        if len(choices) < 2:
            raise ValueError("choice contract requires at least two choices")
        answer_contract = ChoiceAnswerContract(choices)
    elif kind == "deferred_height_choice" and contract.get("strategy") == "height-window-v1":
        answer_contract = DeferredHeightChoiceContract()
    else:
        raise ValueError("unsupported answer contract")
    return QuestionProposal(identifier, question, answer_contract, queries, hints)


def _string_tuple(value: Any, name: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        value = [value]
    if not isinstance(value, list):
        return ()
    return tuple(item.strip() for item in value if isinstance(item, str) and item.strip())


def _intents_for_query(query: str) -> list[QuestionIntent]:
    return [
        QuestionIntent(kind="presence", object_query=query),
        QuestionIntent(kind="horizontal_direction", object_query=query),
        QuestionIntent(kind="within_distance", object_query=query, threshold_m=3.0),
        QuestionIntent(kind="compare_nearest_by_side", object_query=query),
    ]


def _parse_json_array(response: str) -> Any:
    stripped = response.strip()
    if stripped.startswith("```"):
        stripped = re.sub(r"^```(?:json)?\s*|\s*```$", "", stripped, flags=re.IGNORECASE)
    start, end = stripped.find("["), stripped.rfind("]")
    if start < 0 or end < start:
        raise json.JSONDecodeError("expected JSON array", stripped, 0)
    return json.loads(stripped[start : end + 1])
