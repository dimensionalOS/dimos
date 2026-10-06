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

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import upstream
from dimos.evals.suites.lib.space.constants import SOURCE_SHA256
from dimos.evals.suites.lib.space.data import SpacePaths
from dimos.evals.suites.lib.space.upstream import OfficialSpace, RawReplyAgent, question_digest


def run_git(source: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(source), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


@pytest.fixture
def source(tmp_path: Path, mocker: MockerFixture) -> Path:
    checkout = tmp_path / "source"
    checkout.mkdir()
    run_git(checkout, "init")
    hashes: dict[str, str] = {}
    for relative in SOURCE_SHA256:
        path = checkout / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# Synthetic source-integrity fixture.\n")
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    run_git(checkout, "add", "space")
    run_git(
        checkout,
        "-c",
        "user.name=SPACE test",
        "-c",
        "user.email=space-test@example.invalid",
        "commit",
        "-m",
        "synthetic fixture",
    )
    mocker.patch.object(upstream, "SPACE_REVISION", run_git(checkout, "rev-parse", "HEAD"))
    mocker.patch.object(upstream, "SOURCE_SHA256", hashes)
    return checkout


def test_verify_source_accepts_the_pinned_clean_tree(source: Path) -> None:
    upstream.verify_source(source)
    assert run_git(source, "status", "--porcelain") == ""


def test_verify_source_rejects_another_revision(source: Path, mocker: MockerFixture) -> None:
    mocker.patch.object(upstream, "SPACE_REVISION", "0" * 40)
    with pytest.raises(ValueError, match="revision"):
        upstream.verify_source(source)


@pytest.mark.parametrize("tracked", [True, False])
def test_verify_source_rejects_changed_executable_source(source: Path, tracked: bool) -> None:
    relative = "space/evaluate_qas.py" if tracked else "space/injected.py"
    (source / relative).write_text("# unexpected change\n")
    with pytest.raises(ValueError, match="local changes"):
        upstream.verify_source(source)


def test_verify_source_checks_the_recorded_digest(source: Path, mocker: MockerFixture) -> None:
    mocker.patch.object(upstream, "SOURCE_SHA256", {"space/evaluate_qas.py": "0" * 64})
    with pytest.raises(ValueError, match="hash mismatch"):
        upstream.verify_source(source)


def test_prepare_rejects_an_existing_foreign_space_package(
    source: Path, tmp_path: Path, mocker: MockerFixture
) -> None:
    foreign = ModuleType("space")
    foreign.__file__ = str(tmp_path / "elsewhere" / "__init__.py")
    mocker.patch.dict(sys.modules, {"space": foreign})
    with pytest.raises(RuntimeError, match="outside the pinned checkout"):
        OfficialSpace(source).prepare()


def test_prepare_restores_import_path_when_dependencies_are_missing(
    source: Path, mocker: MockerFixture
) -> None:
    before = list(sys.path)
    mocker.patch(
        "dimos.evals.suites.lib.space.upstream.importlib.import_module",
        side_effect=ImportError("missing torch"),
    )
    with pytest.raises(ImportError, match="missing torch"):
        OfficialSpace(source).prepare()
    assert sys.path == before


def test_score_uses_the_official_metric_and_exact_question(
    source: Path, mocker: MockerFixture
) -> None:
    evaluator = SimpleNamespace()
    registry = SimpleNamespace()
    evaluate = mocker.Mock(return_value=({"accuracy": 37.5}, 2, {}))
    evaluator.evaluate_on_qa = evaluate
    registry.AGENTS_REGISTRY = {}
    registry.register_agent = mocker.Mock()
    modules = {"space.evaluate_qas": evaluator, "space.registry": registry}
    mocker.patch(
        "dimos.evals.suites.lib.space.upstream.importlib.import_module",
        side_effect=modules.__getitem__,
    )
    qa: dict[str, Any] = {
        "question": "Unchanged\nquestion",
        "answer": 2,
        "metadata": {"untouched": True},
    }
    raw = '  {"answer":2}\n'
    scorer = OfficialSpace(source)
    scorer.prepare()
    result = scorer.score(qa, raw)
    assert result.accuracy == 37.5
    assert result.prediction == 2
    evaluate.assert_called_once_with(
        agent_name="RawReplyAgent",
        agent_cfg={"responses": {question_digest(qa["question"]): raw}},
        qa=qa,
    )
    registry.register_agent.assert_called_once_with(RawReplyAgent)


def test_replay_accepts_official_config_and_never_selects_by_label(mocker: MockerFixture) -> None:
    parser = SimpleNamespace()
    parse = mocker.Mock(return_value=3)
    parser.QA_Agent = type("QA_Agent", (), {"parse_answer_from_response": parse})
    mocker.patch(
        "dimos.evals.suites.lib.space.upstream.importlib.import_module", return_value=parser
    )
    raw = ' {"answer":"3"} '
    agent = RawReplyAgent({question_digest("question"): raw}, use_vllm=False)
    assert agent.get_prediction("question", 1) == 3
    assert agent.get_prediction("question", 4) == 3
    assert parse.call_args_list == [mocker.call(None, raw), mocker.call(None, raw)]


def test_replay_rejects_enabling_a_model_backend() -> None:
    with pytest.raises(ValueError, match="cannot enable a model backend"):
        RawReplyAgent({}, use_vllm=True)


def test_unprepared_scoring_fails_before_using_a_response(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="Prepare"):
        OfficialSpace(tmp_path).score({"question": "q", "answer": 1}, '{"answer":1}')


@pytest.mark.self_hosted
def test_native_upstream_parser_and_metric(native_space_paths: SpacePaths) -> None:
    """Explicit integration gate: requires the documented SPACE setup and extra."""
    scorer = OfficialSpace(native_space_paths.source)
    scorer.prepare()
    examples: list[tuple[str, Any, float, Any]] = [
        ('{"answer":"2"}', 2, 100.0, 2),
        ('{"answer":2}', 1, 0.0, 2),
        ("no JSON", 1, 0.0, None),
        ('{"answer":2.9}', 2, 100.0, 2),
    ]
    for response, answer, accuracy, prediction in examples:
        result = scorer.score(
            {"question": "Synthetic contract question", "answer": answer}, response
        )
        assert result.accuracy == accuracy
        assert result.prediction == prediction
