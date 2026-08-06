# Copyright 2026 Dimensional Inc.
"""Image-only constrained VQA question proposal."""

from __future__ import annotations

import json
import re
from typing import Any

from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.vqa.models import QuestionIntent

QUESTION_PROMPT = """You generate challenging but visually answerable single-frame VQA questions.
Inspect only this image. Do not assume depth, point clouds, metadata, or temporal context.
Return JSON only: an array of at most 5 visible, salient object names as strings.
Do not return floors, walls, ceilings, background surfaces, questions, explanations, Markdown,
or information not visible in the image."""

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
            intents.append(QuestionIntent(kind=kind, object_query=query, threshold_m=threshold))
        return intents


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
