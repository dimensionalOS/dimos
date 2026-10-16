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

"""Integration tests against the pinned official SPACE evaluator."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import sys
from typing import Any

from langchain_core.language_models.fake_chat_models import FakeListChatModel
import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.evals import cli
from dimos.evals.agents.text_question import TextQuestion
from dimos.evals.runner import EvalRunner
from dimos.evals.suites import space_map_sketching
from dimos.evals.suites.lib.space.data import SpacePaths, load_examples, setup
from dimos.evals.suites.lib.space.process import run_process
from dimos.evals.suites.lib.space.suite import SpaceSuite
from dimos.evals.suites.lib.space.upstream import OfficialSpace, question_digest


@pytest.fixture(scope="session")
def native_space_paths() -> SpacePaths:
    paths = SpacePaths()
    setup(paths)
    return paths


@pytest.mark.self_hosted
def test_native_subset_covers_layouts_and_answer_positions(native_space_paths: SpacePaths) -> None:
    examples = load_examples(native_space_paths)
    assert len(examples) == 20
    assert len({e.index for e in examples}) == 20
    assert len({e.question for e in examples}) == 20
    assert len({e.environment for e in examples}) == 20
    layouts = Counter(e.environment.rsplit("_", 1)[0] for e in examples)
    assert len(layouts) == 10
    assert set(layouts.values()) == {2}
    assert Counter(e.qa["answer"] for e in examples) == {1: 5, 2: 5, 3: 5, 4: 5}


@pytest.mark.self_hosted
def test_native_upstream_parser_and_metric(native_space_paths: SpacePaths) -> None:
    scorer = OfficialSpace()
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


@pytest.mark.self_hosted
def test_native_worker_uses_official_aggregate_with_spawn(
    tmp_path: Path, native_space_paths: SpacePaths
) -> None:
    questions = tmp_path / "questions.json"
    replies = tmp_path / "replies.json"
    output = tmp_path / "official"
    questions.write_text(
        json.dumps([{"question": f"Synthetic question {i}", "answer": i + 1} for i in range(4)])
    )
    replies.write_text(
        json.dumps(
            dict(
                zip(
                    [question_digest(f"Synthetic question {i}") for i in range(4)],
                    ['{"answer":1}', '{"answer":1}', "invalid", '{"answer":"4"}'],
                    strict=True,
                )
            )
        )
    )
    log = tmp_path / "worker.log"
    returncode = run_process(
        [
            sys.executable,
            "-c",
            "import multiprocessing, runpy; "
            "multiprocessing.set_start_method('spawn'); "
            "runpy.run_module('dimos.evals.suites.lib.space.worker', "
            "run_name='__main__', alter_sys=True)",
            "--questions",
            str(questions),
            "--replies",
            str(replies),
            "--output",
            str(output),
        ],
        log,
        90.0,
    )
    assert returncode == 0, log.read_text()
    paths = list(output.glob("*/*/results.json"))
    assert len(paths) == 1, log.read_text()
    record = json.loads(paths[0].read_text())
    assert record["all_predictions"] == [1, 1, None, 4]
    assert record["all_metrics"] == [
        {"accuracy": 100.0},
        {"accuracy": 0.0},
        {"accuracy": 0.0},
        {"accuracy": 100.0},
    ]
    assert record["mean_metrics"] == {"accuracy": 50.0}


@pytest.mark.self_hosted
def test_native_standard_cli_preserves_prompt_and_official_score(
    native_space_paths: SpacePaths,
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    examples = load_examples(native_space_paths)[:2]
    assert examples[0].qa["answer"] != examples[1].qa["answer"]
    responses = [json.dumps({"answer": example.qa["answer"]}) for example in examples]
    runner = EvalRunner(out_dir=tmp_path)
    mocker.patch("dimos.evals.runner.EvalRunner", return_value=runner)
    mocker.patch.object(space_map_sketching, "SUITE", SpaceSuite())
    # Only the provider is replaced; suite discovery, execution and grading stay real.
    mocker.patch.object(
        cli,
        "agent_class",
        return_value=lambda **kwargs: TextQuestion(
            chat_model=FakeListChatModel(responses=responses), **kwargs
        ),
    )
    result = CliRunner().invoke(
        cli.app,
        [
            "run",
            "dimos.evals.suites.space_map_sketching",
            "--agent",
            "dimos.evals.agents.text_question",
            "--limit",
            "2",
        ],
    )
    assert result.exit_code == 0, result.output
    scored = CliRunner().invoke(cli.app, ["score", "space", str(runner.run_dir)])
    assert scored.exit_code == 0, scored.output
    report = json.loads(scored.output)
    assert report["complete"]
    assert report["scored_denominator"] == 2
    assert not report["fixed_subset_complete"]
    saved = [
        json.loads(line) for line in (runner.run_dir / "results.jsonl").read_text().splitlines()
    ]
    official = json.loads(Path(report["official_results_path"]).read_text())
    assert report["official_percent"] == official["mean_metrics"]["accuracy"] == 100.0
    for example, response, result, metric in zip(
        examples, responses, saved, official["all_metrics"], strict=True
    ):
        assert result["case_id"] == example.case_id
        assert result["final_answer"] == response
        assert result["tool_calls"] == 0
        assert not result["error"]
        assert result["score"] == 1.0
        assert result["score"] * 100 == metric["accuracy"]
        (request,) = (runner.run_dir / example.case_id / "raw").glob("*-request.json")
        messages = json.loads(request.read_text())["messages"]
        assert len(messages) == 1
        assert messages[0]["type"] == "human"
        assert messages[0]["content"] == [{"type": "text", "text": example.question}]
