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

from collections.abc import Callable
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import worker
from dimos.evals.suites.lib.space.data import SpacePaths
from dimos.evals.suites.lib.space.process import run_process
from dimos.evals.suites.lib.space.upstream import question_digest


@pytest.fixture
def replay_files(tmp_path: Path) -> tuple[Path, Path]:
    questions = tmp_path / "questions.json"
    replies = tmp_path / "replies.json"
    questions.write_text(json.dumps([{"question": "Synthetic question", "answer": 2}]))
    replies.write_text(json.dumps({question_digest("Synthetic question"): ' {"answer":2}\n'}))
    return questions, replies


def test_load_replies_preserves_raw_strings(replay_files: tuple[Path, Path]) -> None:
    assert worker.load_replies(*replay_files) == {
        question_digest("Synthetic question"): ' {"answer":2}\n'
    }


@pytest.mark.parametrize("responses", [{}, {"unexpected-hash": "reply"}])
def test_load_replies_requires_the_exact_question_set(
    replay_files: tuple[Path, Path], responses: dict[str, str]
) -> None:
    questions, replies = replay_files
    replies.write_text(json.dumps(responses))
    with pytest.raises(ValueError, match="exactly match"):
        worker.load_replies(questions, replies)


@pytest.mark.parametrize(
    ("rows", "error"),
    [
        ([], "nonempty"),
        ([{"question": ["image"], "answer": 1}], "string questions"),
        ([{"question": "question"}], "missing its official answer"),
        (
            [{"question": "duplicate", "answer": 1}, {"question": "duplicate", "answer": 2}],
            "ambiguous",
        ),
    ],
)
def test_load_replies_rejects_malformed_or_ambiguous_questions(
    replay_files: tuple[Path, Path], rows: list[dict[str, Any]], error: str
) -> None:
    questions, replies = replay_files
    questions.write_text(json.dumps(rows))
    with pytest.raises(ValueError, match=error):
        worker.load_replies(questions, replies)


def test_load_replies_does_not_turn_missing_responses_into_empty_answers(
    replay_files: tuple[Path, Path],
) -> None:
    questions, replies = replay_files
    replies.write_text(json.dumps({question_digest("Synthetic question"): None}))
    with pytest.raises(ValueError, match="raw strings"):
        worker.load_replies(questions, replies)


def test_aggregate_invokes_official_main_with_saved_replies(
    replay_files: tuple[Path, Path], tmp_path: Path, mocker: MockerFixture
) -> None:
    questions, replies = replay_files
    output = tmp_path / "official"
    registry = SimpleNamespace()
    evaluator = SimpleNamespace()
    configs: dict[str, Callable[[], worker.ReplayConfig]] = {}

    def register(name: str) -> Callable[[Callable[[], worker.ReplayConfig]], None]:
        def accept(factory: Callable[[], worker.ReplayConfig]) -> None:
            configs[name] = factory

        return accept

    def main(**kwargs: Any) -> None:
        path = Path(kwargs["save_dir"]) / kwargs["model_name"] / "native-result"
        path.mkdir(parents=True)
        (path / "results.json").write_text(
            json.dumps({"all_metrics": [{"accuracy": 100.0}], "all_predictions": [2]})
        )

    registry.register_config = register
    evaluator.main = mocker.Mock(side_effect=main)
    modules = {"space.registry": registry, "space.evaluate_qas": evaluator}
    mocker.patch(
        "dimos.evals.suites.lib.space.worker.importlib.import_module",
        side_effect=modules.__getitem__,
    )
    worker.run_aggregate(questions, replies, output)
    evaluator.main.assert_called_once_with(
        model_name=worker.MODEL_NAME,
        data_path=str(questions.resolve()),
        save_dir=str(output.resolve()),
        n_workers=1,
    )
    config = configs[worker.MODEL_NAME]()
    assert config.responses == {question_digest("Synthetic question"): ' {"answer":2}\n'}
    assert config.agent_name == "RawReplyAgent"
    assert config.use_vllm is False


def test_aggregate_rejects_an_existing_result_directory(
    replay_files: tuple[Path, Path], tmp_path: Path
) -> None:
    output = tmp_path / "existing"
    output.mkdir()
    (output / "results.json").write_text("existing result")
    with pytest.raises(ValueError, match="must be empty"):
        worker.run_aggregate(*replay_files, output)
    assert (output / "results.json").read_text() == "existing result"


@pytest.mark.self_hosted
def test_native_worker_uses_official_aggregate_with_spawn(
    tmp_path: Path, native_space_paths: SpacePaths
) -> None:
    """Explicit integration gate with synthetic questions, never benchmark fixtures."""
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
            "-m",
            "dimos.evals.suites.lib.space.worker",
            "--source",
            str(native_space_paths.source),
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
