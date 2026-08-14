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
"""Image-only constrained VQA question proposal."""

from __future__ import annotations

import json
import re
from typing import Any

from dimos.benchmark.vqa.contracts import (
    AnswerContract,
    BooleanAnswerContract,
    ChoiceAnswerContract,
    QuestionIntent,
    QuestionProposal,
)
from dimos.benchmark.vqa.generation.answer_choices import COUNT_CHOICES
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image

QUESTION_PROMPT = """Select challenging but visually well-supported single-frame VQA intents.
Inspect only this image. Do not assume depth, point clouds, calibration, metadata, or temporal context.
Return JSON only: an array of objects with kind, object_query, and threshold_m only for
within_distance, and comparison_query only for compare_left_right. kind must be one of presence,
horizontal_direction, within_distance, visible_count, camera_range, compare_nearest_by_side, or
compare_left_right.
Choose a useful positive threshold_m for each within_distance intent based on the visible scene.
Use image context to select only intents likely to produce a useful geometric case: emit
compare_nearest_by_side only when at least two visible instances of the same object appear on
opposite image sides; emit horizontal_direction only for a visible object; and prefer relational
or directional intents over presence. Diversify object classes and question families when the scene
supports them.
Use grammatically correct object names and questions. Emit visible_count only when at least one
repeated visible object is clearly present. The zero choice is only an exhaustive distractor; never
propose a count for an absent class. Emit camera_range only
for a visible object. Emit compare_left_right only for two distinct visible object types;
comparison_query must name the other object type.
Prioritize high-quality, relevant questions over exhaustively covering every visible object.
Do not return bare object names, questions, answers, explanations, Markdown, or information not
visible in the image."""

AGENTIC_QUESTION_PROMPT = """Author challenging, visually answerable single-frame VQA questions.
Inspect only this image. Do not use or infer depth, point clouds, calibration, metadata, or answers.
Return JSON only: an array of objects with question, answer_contract, and optional object_queries.
answer_contract is {"kind":"boolean"} or
{"kind":"choice","choices":[...]}.
Prioritize questions that a private point-cloud oracle can validate from grounded object range and
horizontal position. Aim for a diverse set of questions about presence, count, left/right position,
camera range, and which same-class instance is nearest by image side. Use grammatical singular
object queries and complete concise questions. For a visibly repeated object, count questions must use exactly
["0", "1-2", "3-4", "5-7", "8+"]; zero is an exhaustive distractor, not permission to ask about
an absent class. Camera-distance questions must use exactly ["under 1 m",
"1 to under 2 m", "2 to under 4 m", "4 m or more"]. For a pairwise left/right range question,
use exactly ["left", "right"]. For an A/B left-right relation question, use exactly ["left", "right"]
and name both objects.
Use concise, mutually exclusive fixed choices. Prioritize high-quality, relevant questions over
exhaustively covering every visible object.
Use object_queries for every referenced object. The private oracle chooses its own sequence of reusable
perception and grounding primitives; no answer-level composite tools are available. For counts, enumerate,
segment, and ground every returned detection for the named query, cite every grounded instance, and reject
if completeness is uncertain. Use visibility/presence questions
only when no stronger geometric question is available.
Do not ask about color, material, text, intent, full physical size, hidden parts, or exact
metric distances without a supplied choice contract. Use only visible objects. Do not include
answers, explanations, Markdown, or background surfaces."""


class ConstrainedQuestionAuthor:
    """Propose constrained VQA intents from an image without geometry access."""

    def __init__(self, model: OpenAIVlModel) -> None:
        self._model = model
        self.rejections: list[str] = []

    def propose(self, image: Image) -> list[QuestionIntent]:
        self.rejections = []
        payload = _query_json_array(self._model, image, QUESTION_PROMPT, "question agent")
        intents: list[QuestionIntent] = []
        for index, item in enumerate(payload, start=1):
            if not isinstance(item, dict):
                self.rejections.append(f"intent-{index}:not_an_object")
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
            ):
                self.rejections.append(f"intent-{index}:unsupported_kind")
                continue
            if not isinstance(query, str) or not query.strip():
                self.rejections.append(f"intent-{index}:missing_object_query")
                continue
            query = query.strip()
            if kind == "within_distance" and (
                not isinstance(threshold, (int, float)) or threshold <= 0
            ):
                self.rejections.append(f"intent-{index}:invalid_distance_threshold")
                continue
            if kind != "within_distance":
                threshold = None
            comparison_query = item.get("comparison_query")
            if kind == "compare_left_right" and (
                not isinstance(comparison_query, str)
                or not comparison_query
                or comparison_query == query
            ):
                self.rejections.append(f"intent-{index}:invalid_comparison_query")
                continue
            if kind != "compare_left_right":
                comparison_query = None
            intents.append(QuestionIntent(kind, query, threshold, comparison_query))
        return list(dict.fromkeys(intents))


class AgenticQuestionAuthor:
    """Image-only author for generic public questions and answer contracts."""

    def __init__(
        self,
        model: OpenAIVlModel,
    ) -> None:
        self._model = model
        self.rejections: list[str] = []

    def propose(self, image: Image) -> list[QuestionProposal]:
        self.rejections = []
        payload = _query_json_array(self._model, image, AGENTIC_QUESTION_PROMPT, "question author")
        proposals = []
        errors: list[ValueError] = []
        for index, item in enumerate(payload, start=1):
            try:
                proposals.append(_proposal_from_json(item, index))
            except ValueError as exc:
                errors.append(exc)
                self.rejections.append(f"proposal-{index}:{exc}")
                continue
        if not proposals and errors:
            raise errors[0]
        return _uniquify_proposal_ids(proposals)


def _proposal_from_json(item: Any, index: int) -> QuestionProposal:
    if not isinstance(item, dict):
        raise ValueError("question proposal must be an object")
    identifier, question = item.get("id"), item.get("question")
    if not isinstance(question, str) or not question:
        raise ValueError("question proposal requires question")
    if not isinstance(identifier, str) or not identifier:
        identifier = f"proposal-{index:02d}"
    queries = _string_tuple(item.get("object_queries", []), "object_queries")
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
        normalized_choices = [choice.rstrip(".,;:").casefold() for choice in choices]
        if len(set(normalized_choices)) != len(normalized_choices):
            raise ValueError("choice contract choices must be unique")
        answer_contract = ChoiceAnswerContract(choices)
    else:
        raise ValueError("unsupported answer contract")
    is_count = (
        re.match(r"^\s*(?:how many|what (?:is )?the number of|what number of)\b", question, re.I)
        is not None
    )
    if isinstance(answer_contract, ChoiceAnswerContract) and is_count:
        if not queries:
            raise ValueError("count questions require visible object queries")
        if answer_contract.choices != COUNT_CHOICES:
            raise ValueError("count questions require the fixed exhaustive count choices")
    return QuestionProposal(identifier, question, answer_contract, queries)


def _string_tuple(value: Any, name: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        value = [value]
    if not isinstance(value, list):
        return ()
    return tuple(item.strip() for item in value if isinstance(item, str) and item.strip())


def _query_json_array(model: OpenAIVlModel, image: Image, prompt: str, label: str) -> list[Any]:
    last_error: Exception | None = None
    for _ in range(2):
        try:
            payload: Any = _parse_json_array(model.query(image, prompt))
            if isinstance(payload, list):
                return payload
            last_error = ValueError("response is not an array")
        except json.JSONDecodeError as exc:
            last_error = exc
    raise ValueError(f"{label} did not return a JSON array after one retry") from last_error


def _uniquify_proposal_ids(proposals: list[QuestionProposal]) -> list[QuestionProposal]:
    counts: dict[str, int] = {}
    unique: list[QuestionProposal] = []
    for proposal in proposals:
        count = counts.get(proposal.id, 0) + 1
        counts[proposal.id] = count
        identifier = proposal.id if count == 1 else f"{proposal.id}-{count}"
        unique.append(
            QuestionProposal(
                identifier,
                proposal.question,
                proposal.answer_contract,
                proposal.object_queries,
            )
        )
    return unique


def _parse_json_array(response: str) -> Any:
    stripped = response.strip()
    if stripped.startswith("```"):
        stripped = re.sub(r"^```(?:json)?\s*|\s*```$", "", stripped, flags=re.IGNORECASE)
    start, end = stripped.find("["), stripped.rfind("]")
    if start < 0 or end < start:
        raise json.JSONDecodeError("expected JSON array", stripped, 0)
    return json.loads(stripped[start : end + 1])
