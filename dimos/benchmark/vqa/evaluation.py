# Copyright 2026 Dimensional Inc.
"""Dataset-backed VQA cases for the standard evaluation runner."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
from typing import Any, cast

from dimos.evals.types import EvalCase, EvalResult, EvalRig, Suite
from dimos.msgs.sensor_msgs.Image import Image


@dataclass(frozen=True, kw_only=True)
class MultipleChoiceVqaCase(EvalCase):
    """Evaluate one public image question against its private expected choice."""

    image: Path
    choices: tuple[str, ...]
    expected: str

    def evaluate(self, rig: EvalRig) -> EvalResult:
        context = (
            []
            if rig.blind
            else cast("list[dict[str, Any]]", Image.from_file(self.image).agent_encode())
        )
        response = rig.ask(context, _question_prompt(self.inputs, self.choices))
        answer = _parse_choice(response, self.choices)
        return EvalResult(
            case_id=self.id,
            outputs=response,
            score=float(answer == self.expected),
        )

    def preflight(self, rig: EvalRig) -> None:
        if not self.image.is_file():
            raise ValueError(f"missing VQA image: {self.image}")


def load_suite(dataset: str | Path) -> Suite:
    """Load one generated VQA dataset as standard evaluation cases."""
    root = Path(dataset).expanduser().resolve()
    cases = _load_jsonl(root / "cases.jsonl")
    labels = _load_labels(_load_jsonl(root / "labels.jsonl"))
    case_ids = [_required_string(item, "id") for item in cases]
    if len(set(case_ids)) != len(case_ids):
        raise ValueError("VQA cases must have unique IDs")
    if set(case_ids) != set(labels):
        raise ValueError("VQA case and label IDs must match exactly")

    suite: list[EvalCase] = []
    for item in cases:
        identifier = _required_string(item, "id")
        choices = _choices(item)
        expected = labels[identifier]
        if expected not in choices:
            raise ValueError(f"private label for {identifier} is not an allowed choice")
        image = (root / _required_string(item, "image")).resolve()
        if not image.is_relative_to(root):
            raise ValueError(f"VQA image for {identifier} escapes the dataset directory")
        suite.append(
            MultipleChoiceVqaCase(
                id=identifier,
                inputs=_required_string(item, "question"),
                image=image,
                choices=choices,
                expected=expected,
                tags=frozenset({"vqa", "image", "generated"}),
            )
        )
    return suite


def _question_prompt(question: str, choices: tuple[str, ...]) -> str:
    return (
        f"{question}\n\nChoices: {', '.join(choices)}.\n"
        "Use only the supplied image. End with exactly `ANSWER: <choice>`."
    )


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise ValueError(f"missing VQA dataset file: {path}")
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        item: Any = json.loads(line)
        if not isinstance(item, dict):
            raise ValueError(f"VQA dataset rows must be objects: {path}")
        rows.append(item)
    return rows


def _load_labels(rows: list[dict[str, Any]]) -> dict[str, str]:
    labels: dict[str, str] = {}
    for row in rows:
        identifier = _required_string(row, "id")
        if identifier in labels:
            raise ValueError(f"duplicate VQA label ID: {identifier}")
        labels[identifier] = _required_string(row, "answer")
    return labels


def _required_string(item: dict[str, Any], key: str) -> str:
    value = item.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"VQA case requires non-empty {key}")
    return value


def _choices(case: dict[str, Any]) -> tuple[str, ...]:
    value = case.get("choices")
    if (
        not isinstance(value, list)
        or len(value) < 2
        or not all(isinstance(item, str) for item in value)
    ):
        raise ValueError("VQA case requires at least two string choices")
    return tuple(value)


def _parse_choice(response: str, choices: tuple[str, ...]) -> str | None:
    match = re.search(r"^ANSWER:\s*(.+?)\s*$", response, re.MULTILINE)
    if match is None:
        return None
    answer = match.group(1).strip().casefold()
    aliases = {"true": "yes", "false": "no"} if set(choices) == {"yes", "no"} else {}
    answer = aliases.get(answer, answer)
    return next((choice for choice in choices if choice.casefold() == answer), None)
