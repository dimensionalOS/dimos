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
"""Host-only scoring against explicitly supplied private oracle data."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from dimos.benchmark.spatial.models import (
    Answer,
    AnswerType,
    ReviewAction,
    ReviewOverride,
    SpatialModel,
)
from dimos.benchmark.spatial.pi_baseline.records import Prediction
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json


class PrivateScore(SpatialModel):
    """Versioned private result; never write this record below a public root."""

    record_type: Literal["pi-score"] = "pi-score"
    schema_version: Literal["1.0"] = "1.0"
    instance_id: str = Field(min_length=1)
    answer_type: AnswerType
    value: bool | int
    run_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    case_id: str = Field(min_length=1)
    mode: str = Field(min_length=1)
    release_id: str = Field(min_length=1)
    scorer_revision: str = Field(min_length=1)
    outcome: Literal["correct", "incorrect", "excluded"]
    expected_value: bool | int | None = None
    override_id: str | None = None
    scored_at: datetime

    @model_validator(mode="after")
    def validate_value_type(self) -> PrivateScore:
        if self.answer_type is AnswerType.BOOLEAN and type(self.value) is not bool:
            raise ValueError("boolean scores require a bool")
        if self.answer_type is AnswerType.INTEGER and (
            type(self.value) is not int or self.value < 0
        ):
            raise ValueError("integer scores require a non-negative int")
        return self


class ScoreKey(SpatialModel):
    """Stable identity of one append-only score ledger entry."""

    record_type: Literal["pi-score-key"] = "pi-score-key"
    schema_version: Literal["1.0"] = "1.0"
    run_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    case_id: str = Field(min_length=1)
    mode: str = Field(min_length=1)
    release_id: str = Field(min_length=1)
    scorer_revision: str = Field(min_length=1)


def score_case(
    case: Path | dict[str, JsonValue],
    prediction: Prediction,
    *,
    oracle_root: Path,
    run_id: str,
    mode: str,
    release_id: str,
    scorer_revision: str,
    ledger_path: Path | None = None,
) -> PrivateScore:
    """Score a canonical public case using only an explicit private oracle root."""
    payload = _load_case(case)
    instance = _mapping(payload, "instance")
    question = _mapping(payload, "question")
    case_id = _string(instance, "instance_id")
    answer_type = AnswerType(_string(question, "answer_type"))
    if prediction.instance_id != case_id or prediction.answer_type is not answer_type:
        raise ValueError("prediction does not match the public case identity or answer type")
    answer, override = validate_private_case(payload, oracle_root)
    expected: bool | int | None
    if override is not None and override.action is ReviewAction.EXCLUDE:
        outcome: Literal["correct", "incorrect", "excluded"] = "excluded"
        expected = None
    else:
        selected = override.corrected_value if override is not None else answer.value
        expected = _answer_value(selected, answer_type)
        outcome = "correct" if prediction.value == expected else "incorrect"
    result = PrivateScore(
        instance_id=prediction.instance_id,
        answer_type=answer_type,
        value=prediction.value,
        run_id=run_id,
        case_id=case_id,
        mode=mode,
        release_id=release_id,
        scorer_revision=scorer_revision,
        outcome=outcome,
        expected_value=expected,
        override_id=override.override_id if override is not None else None,
        scored_at=datetime.now(timezone.utc),
    )
    if ledger_path is not None:
        append_score(ledger_path, result)
    return result


def validate_private_case(
    case: dict[str, JsonValue], oracle_root: Path
) -> tuple[Answer, ReviewOverride | None]:
    """Validate the complete private binding for one public case before execution."""
    question = _mapping(case, "question")
    question_id = _string(question, "question_id")
    predicate = _string(question, "predicate")
    answer_type = AnswerType(_string(question, "answer_type"))
    answer = _load_answer(oracle_root, case, question_id)
    if answer.question_id != question_id or answer.predicate.value != predicate:
        raise ValueError("private answer does not match the public question")
    _answer_value(answer.value, answer_type)
    override = _load_override(oracle_root, case, question_id)
    if override is not None and override.action is ReviewAction.CORRECT:
        if override.corrected_value is None:
            raise ValueError("correct override is missing its value")
        _answer_value(override.corrected_value, answer_type)
    return answer, override


def append_score(path: Path, score: PrivateScore) -> None:
    """Append one score and reject duplicate keys; existing bytes are immutable."""
    key = _score_key(score)
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if line and _score_key(PrivateScore.model_validate_json(line)) == key:
                raise ValueError("score key already exists in append-only ledger")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("ab") as ledger:
        ledger.write(canonical_json(score.model_dump(mode="json")) + b"\n")


def _load_case(case: Path | dict[str, JsonValue]) -> dict[str, JsonValue]:
    if isinstance(case, Path):
        value = json.loads(case.read_text(encoding="utf-8"))
    else:
        value = case
    if not isinstance(value, dict) or value.get("schema_version") != "1.0":
        raise ValueError("case must be a case.v1 record")
    return value


def _load_answer(root: Path, case: dict[str, JsonValue], question_id: str) -> Answer:
    path = _oracle_path(root, case, "answers.jsonl")
    matches = [
        Answer.model_validate_json(line)
        for line in path.read_text().splitlines()
        if line and json.loads(line).get("question_id") == question_id
    ]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one private answer for {question_id}")
    return matches[0]


def _load_override(
    root: Path, case: dict[str, JsonValue], question_id: str
) -> ReviewOverride | None:
    scene = _string(_mapping(case, "scene"), "scene_id")
    trajectory = _string(_mapping(case, "trajectory"), "trajectory_id")
    path = root / "scenes" / scene / "trajectories" / trajectory / "review_overrides.jsonl"
    if not path.exists():
        return None
    matches = [
        ReviewOverride.model_validate_json(line)
        for line in path.read_text().splitlines()
        if line and json.loads(line).get("question_id") == question_id
    ]
    if len(matches) > 1:
        raise ValueError(f"multiple private review overrides for {question_id}")
    return matches[0] if matches else None


def _oracle_path(root: Path, case: dict[str, JsonValue], filename: str) -> Path:
    scene = _string(_mapping(case, "scene"), "scene_id")
    trajectory = _string(_mapping(case, "trajectory"), "trajectory_id")
    path = root / "scenes" / scene / "trajectories" / trajectory / filename
    if not path.is_file():
        raise ValueError(f"required private oracle artifact is missing: {path}")
    return path


def _answer_value(value: object, answer_type: AnswerType) -> bool | int:
    result = getattr(value, "value", None)
    if answer_type is AnswerType.BOOLEAN and type(result) is bool:
        return result
    if answer_type is AnswerType.INTEGER and type(result) is int and result >= 0:
        return result
    raise ValueError("oracle answer type does not match the public AnswerType")


def _score_key(score: PrivateScore) -> tuple[str, str, str, str, str]:
    return score.run_id, score.case_id, score.mode, score.release_id, score.scorer_revision


def _mapping(value: dict[str, JsonValue], key: str) -> dict[str, JsonValue]:
    child = value.get(key)
    if not isinstance(child, dict):
        raise ValueError(f"case field {key!r} must be an object")
    return child


def _string(value: dict[str, JsonValue], key: str) -> str:
    child = value.get(key)
    if not isinstance(child, str) or not child:
        raise ValueError(f"case field {key!r} must be non-empty text")
    return child
