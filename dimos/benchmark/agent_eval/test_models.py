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

"""Guarantees the tagged evaluation contracts owe their case files."""

import json

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.models import (
    CompactEvalResult,
    EvalCase,
    ExternalEvaluatorRef,
    FrozenRecordingSource,
    IntegerQuestionTask,
    NoEnvironmentSource,
    VerbatimPromptTask,
)
from dimos.constants import DIMOS_PROJECT_ROOT

SHIPPED_CASE = (
    DIMOS_PROJECT_ROOT
    / "dimos"
    / "benchmark"
    / "short_horizon_qa"
    / "cases"
    / "demo_go2_hongkong_office-room-count-smoke"
    / "case.json"
)


def _case(**overrides: object) -> str:
    """A case wired entirely from the environment-free kinds."""
    case: dict[str, object] = {
        "case_id": "x",
        "source": {"kind": "none"},
        "task": {"kind": "verbatim_prompt", "prompt": "Q?"},
        "validator": {"kind": "external_evaluator", "benchmark": "space", "revision": "abc123"},
    }
    return json.dumps(case | overrides)


def _result(**overrides: object) -> str:
    result: dict[str, object] = {
        "case_id": "x",
        "model": "gpt-5.6-luna",
        "thinking_level": "medium",
        "prediction_status": "not_evaluated",
        "validator_revision": "abc123",
        "tool_call_count": 0,
        "duration_seconds": 1.5,
    }
    return json.dumps(result | overrides)


def test_shipped_case_file_round_trips_field_for_field() -> None:
    case = EvalCase.model_validate_json(SHIPPED_CASE.read_bytes())
    assert case.model_dump(mode="json") == json.loads(SHIPPED_CASE.read_text())


def test_case_wired_from_the_new_kinds_parses() -> None:
    case = EvalCase.model_validate_json(_case())
    assert isinstance(case.source, NoEnvironmentSource)
    assert isinstance(case.task, VerbatimPromptTask)
    assert isinstance(case.validator, ExternalEvaluatorRef)
    assert case.validator.benchmark == "space"
    assert case.validator.revision == "abc123"


def test_verbatim_prompt_adds_nothing_to_the_prompt() -> None:
    case = EvalCase.model_validate_json(_case())
    assert not hasattr(case.task, "answer_marker")
    assert case.task.model_dump(mode="json") == {
        "schema_version": "1.0",
        "kind": "verbatim_prompt",
        "prompt": "Q?",
    }


@pytest.mark.parametrize(
    ("override", "source_kind", "task_kind"),
    [
        (
            {
                "source": {
                    "kind": "frozen_memory",
                    "recording": "go2_hongkong_office",
                    "progress": 1.0,
                }
            },
            FrozenRecordingSource,
            VerbatimPromptTask,
        ),
        (
            {"task": {"kind": "integer_question", "prompt": "How many rooms in total?"}},
            NoEnvironmentSource,
            IntegerQuestionTask,
        ),
    ],
)
def test_sources_and_tasks_pair_in_either_direction(
    override: dict[str, object], source_kind: type, task_kind: type
) -> None:
    case = EvalCase.model_validate_json(_case(**override))
    assert isinstance(case.source, source_kind)
    assert isinstance(case.task, task_kind)


@pytest.mark.parametrize(
    ("field", "spec"),
    [
        ("source", {"kind": "mystery"}),
        ("task", {"kind": "mystery", "prompt": "Q?"}),
        ("validator", {"kind": "mystery", "revision": "abc123"}),
    ],
)
def test_unknown_kind_is_rejected(field: str, spec: dict[str, str]) -> None:
    with pytest.raises(ValidationError, match="does not match any of the expected tags"):
        EvalCase.model_validate_json(_case(**{field: spec}))


@pytest.mark.parametrize(
    ("field", "spec"),
    [
        ("source", {"kind": "none", "recording": "go2_hongkong_office"}),
        ("task", {"kind": "verbatim_prompt", "prompt": "Q?", "answer_marker": "ANSWER:"}),
        (
            "validator",
            {
                "kind": "external_evaluator",
                "benchmark": "space",
                "revision": "abc123",
                "private_path": "private/oracle.json",
            },
        ),
    ],
)
def test_new_kinds_forbid_unknown_fields(field: str, spec: dict[str, str]) -> None:
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        EvalCase.model_validate_json(_case(**{field: spec}))


@pytest.mark.parametrize(
    ("field", "spec"),
    [
        ("task", {"kind": "verbatim_prompt", "prompt": ""}),
        ("validator", {"kind": "external_evaluator", "benchmark": "", "revision": "abc123"}),
        ("validator", {"kind": "external_evaluator", "benchmark": "space", "revision": ""}),
    ],
)
def test_new_kinds_reject_empty_strings(field: str, spec: dict[str, str]) -> None:
    with pytest.raises(ValidationError, match="String should have at least 1 character"):
        EvalCase.model_validate_json(_case(**{field: spec}))


def test_new_kinds_stay_frozen_and_strict() -> None:
    task = VerbatimPromptTask(prompt="Q?")
    with pytest.raises(ValidationError, match="Instance is frozen"):
        task.prompt = "rewritten"
    with pytest.raises(ValidationError, match="Input should be a valid string"):
        VerbatimPromptTask(prompt=123)


def test_result_keeps_the_recorded_environment_when_the_case_had_one() -> None:
    result = CompactEvalResult.model_validate_json(
        _result(recording="go2_hongkong_office", progress=1.0)
    )
    assert result.recording == "go2_hongkong_office"
    assert result.progress == 1.0
    assert CompactEvalResult.model_validate(result.model_dump(mode="json")) == result


def test_result_reports_no_environment_when_the_case_declared_none() -> None:
    result = CompactEvalResult.model_validate_json(_result())
    assert result.recording is None
    assert result.progress is None
    assert CompactEvalResult.model_validate(result.model_dump(mode="json")) == result


def test_result_defaults_the_two_environment_fields_independently() -> None:
    """Neither field implies the other: a half-written environment stays half-written."""
    recording_only = CompactEvalResult.model_validate_json(_result(recording="go2_hongkong_office"))
    assert recording_only.recording == "go2_hongkong_office"
    assert recording_only.progress is None
    progress_only = CompactEvalResult.model_validate_json(_result(progress=1.0))
    assert progress_only.recording is None
    assert progress_only.progress == 1.0
