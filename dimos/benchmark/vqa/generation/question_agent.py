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

QUESTION_PROMPT = """Select up to 15 challenging but visually well-supported single-frame VQA intents.
Inspect only this image. Do not assume depth, point clouds, calibration, metadata, or temporal context.
Return JSON only: an array of objects with kind, object_query, and threshold_m only for
within_distance, candidate_queries only for closest_object, and comparison_query only for
compare_height. kind must be one of presence, horizontal_direction, within_distance, visible_count,
camera_range, compare_nearest_by_side, compare_left_right, compare_height, door_state, closest_object,
object_on_support, opening_width, or forward_path.
Use threshold_m: 3.0 for within_distance.
Use image context to select only intents likely to produce a useful geometric case: emit
compare_nearest_by_side only when at least two visible instances of the same object appear on
opposite image sides; emit horizontal_direction only for a visible object; and prefer relational
or directional intents over presence. Emit door_state only for a clearly visible door with nearby
visible structure. Diversify object classes and question families when the scene supports them.
Emit visible_count only when at least one repeated visible object is present. Emit camera_range only
for a visible object. Emit compare_height only for two distinct visible upright object types resting
on the visible ground; comparison_query must name the other object type. Emit compare_left_right only
for two distinct visible object types; comparison_query must name the other object type. Emit closest_object only when
one target and at least two distinct candidate object types are visible; candidate_queries must name
the visible candidate types.
Emit object_on_support only when one visible object is clearly resting on a distinct visible table,
bench, or other horizontal support; comparison_query must name that support. Emit opening_width only
for one clearly visible doorway aperture with nearby wall structure.
Emit forward_path only when the center foreground and visible floor provide enough context to judge
the local path directly ahead. Use object_query: "forward path" for forward_path.
Do not return bare object names, floors, walls, ceilings, background surfaces,
questions, answers, explanations, Markdown, or information not visible in the image."""

AGENTIC_QUESTION_PROMPT = """Author up to 15 challenging, visually answerable single-frame VQA questions.
Inspect only this image. Do not use or infer depth, point clouds, calibration, metadata, or answers.
Return JSON only: an array of objects with question, answer_contract, optional object_queries,
and optional tool_hints. answer_contract is {"kind":"boolean"}, {"kind":"choice","choices":[...]},
or {"kind":"deferred_height_choice","strategy":"height-window-v1"}.
Prioritize questions that a private point-cloud oracle can validate. For the height of one upright
object resting on visible ground, use deferred_height_choice. Its choices are generated privately
from a successful height measurement. Aim for a diverse set of questions that use the visible scene
composition: relative left/center/right position, which listed object is closest to a named target,
and height for an upright grounded object. For a visibly repeated object, count questions must use exactly
["1-2", "3-4", "5-7", "8+"]; camera-distance questions must use exactly ["under 1 m",
"1 to under 2 m", "2 to under 4 m", "4 m or more"]. For a pairwise left/right range question,
use exactly ["left", "right"]. For an A/B left-right relation question, use exactly ["left", "right"]
and name both objects. For a pairwise height question, use the two distinct object types as the choices.
For closest-object questions, use distinct visible candidate object types as the fixed choices. Use concise,
mutually exclusive fixed choices with two to four options. For a clearly visible door with nearby visible
structure, you may ask whether it is open or closed with exactly ["open", "closed"]. Use the fixed choices
["clear", "blocked"] only for a visibly supported local path directly ahead.
Use object_queries for every referenced object. The private oracle chooses its own sequence of reusable
perception and geometry tools; do not prescribe an answer-level tool sequence. Use visibility/presence questions
only when no stronger geometric question is available.
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
        if not isinstance(payload, list) or len(payload) > 15:
            raise ValueError("question agent must return an array of at most 15 intents")
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
                continue
            kind, query, threshold = (
                item.get("kind"),
                item.get("object_query"),
                item.get("threshold_m"),
            )
            if kind not in (
                "presence",
                "horizontal_direction",
                "within_distance",
                "visible_count",
                "camera_range",
                "compare_nearest_by_side",
                "compare_left_right",
                "compare_height",
                "object_on_support",
                "opening_width",
                "door_state",
                "closest_object",
                "forward_path",
            ):
                continue
            if not isinstance(query, str) or not query:
                continue
            if kind == "within_distance" and (
                not isinstance(threshold, (int, float)) or threshold <= 0
            ):
                continue
            if kind != "within_distance":
                threshold = None
            candidates = _string_tuple(item.get("candidate_queries"), "candidate_queries")
            comparison_query = item.get("comparison_query")
            if kind == "closest_object" and (len(candidates) < 2 or query in candidates):
                continue
            if kind == "forward_path" and query != "forward path":
                continue
            if kind in ("compare_left_right", "compare_height", "object_on_support") and (
                not isinstance(comparison_query, str)
                or not comparison_query
                or comparison_query == query
            ):
                continue
            if kind not in ("compare_left_right", "compare_height", "object_on_support"):
                comparison_query = None
            intents.append(QuestionIntent(kind, query, threshold, candidates, comparison_query))
        return intents


class OpenAIFreeformQuestionAuthor:
    """Image-only author for generic public questions and answer contracts."""

    def __init__(self, model: OpenAIVlModel, max_questions: int = 15) -> None:
        self._model = model
        self._max_questions = max_questions

    def propose(self, image: Image) -> list[QuestionProposal]:
        try:
            payload: Any = _parse_json_array(self._model.query(image, AGENTIC_QUESTION_PROMPT))
        except json.JSONDecodeError as exc:
            raise ValueError("question author did not return JSON") from exc
        if not isinstance(payload, list) or len(payload) > self._max_questions:
            raise ValueError("question author returned too many questions")
        proposals = []
        errors: list[ValueError] = []
        for index, item in enumerate(payload, start=1):
            try:
                proposals.append(_proposal_from_json(item, index))
            except ValueError as exc:
                errors.append(exc)
                continue
        if not proposals and errors:
            raise errors[0]
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
        QuestionIntent(kind="visible_count", object_query=query),
        QuestionIntent(kind="camera_range", object_query=query),
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
