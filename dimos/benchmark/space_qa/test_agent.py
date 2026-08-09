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

"""What the agent does with a question, checked without SPACE and without a model.

The agent's own module needs upstream on ``sys.path``, which no default test
run has. So the three inherited methods it calls are stood up as a stub package
and the module is imported against that, then handed a question the way
``evaluate_qas`` would. Nothing here starts a kernel, spends a token, or reads
Apple's release; the one call that would reach an agent is captured instead.
"""

import importlib
import json
from pathlib import Path
import re
import sys
from types import ModuleType
from typing import Any

import pytest

from dimos.benchmark.agent_eval.models import CompactEvalResult
from dimos.benchmark.space_qa.adapter import SubsetSpec
from dimos.benchmark.space_qa.manifest import RECORD_NAME, RUN_DIR_ENV, case_dir
from dimos.benchmark.space_qa.run import prepare_run
from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE
from dimos.benchmark.space_qa.source import SPACE_REVISION
from dimos.benchmark.space_qa.suite import SpaceQAAdapter
from dimos.benchmark.space_qa.tasks import SpaceTextTask

AGENT_MODULE = "dimos.benchmark.space_qa.agent"
MODEL_NAME = "dimos-pi-gpt-5.6-luna"
TASK = SpaceTextTask(name="FAKE_text", groups=6)
SEED = 20260809


class _StubQAAgent:
    """Upstream's three reply-reading methods, in the shape ``get_prediction`` calls them.

    ``parse_answer_from_response`` follows the upstream one closely enough to
    tell a parsed answer from an unparsed one. What upstream's parser actually
    does is settled against the real checkout in ``test_upstream``, not here.
    """

    def preprocess_question(self, question_content: Any) -> Any:
        return question_content

    def postprocess_response(self, response_txt: str) -> str:
        return response_txt

    def parse_answer_from_response(self, text: str) -> Any:
        found = re.findall(r"({.*?})", text, re.DOTALL)
        if not found:
            return None
        return json.loads(found[-1]).get("answer")


def _install_space_stub() -> None:
    space = ModuleType("space")
    space.__path__ = []  # type: ignore[attr-defined]
    agents = ModuleType("space.agents")
    agents.__path__ = []  # type: ignore[attr-defined]
    qa_agent = ModuleType("space.agents.qa_agent")
    registry = ModuleType("space.registry")
    qa_agent.QA_Agent = _StubQAAgent  # type: ignore[attr-defined]
    registry.register_agent = lambda cls: cls  # type: ignore[attr-defined]
    registry.register_config = lambda _name: (lambda cls: cls)  # type: ignore[attr-defined]
    sys.modules.update(
        {
            "space": space,
            "space.agents": agents,
            "space.agents.qa_agent": qa_agent,
            "space.registry": registry,
        }
    )


def _stubbed_names() -> list[str]:
    return [
        name
        for name in list(sys.modules)
        if name == AGENT_MODULE or name == "space" or name.startswith("space.")
    ]


@pytest.fixture(scope="module")
def agent_module():
    """Import the agent against the stub, and leave ``sys.modules`` as it was found.

    ``test_upstream`` imports the same module against the real checkout when it
    is set up to; neither run may hand the other a half-stubbed one.
    """
    displaced = {name: sys.modules.pop(name) for name in _stubbed_names()}
    _install_space_stub()
    try:
        yield importlib.import_module(AGENT_MODULE)
    finally:
        for name in _stubbed_names():
            del sys.modules[name]
        sys.modules.update(displaced)


class _Writer:
    def write(self, text: str) -> None:
        self.text = text


class _Dialog:
    """The dialog surface ``get_prediction`` touches; upstream's ``reset`` builds the real one."""

    def __init__(self) -> None:
        self.messages: list[tuple[str, Any]] = []
        self.log_writer = _Writer()

    def add_user_message(self, content: Any) -> None:
        self.messages.append(("user", content))

    def add_assistant_message(self, content: Any) -> None:
        self.messages.append(("assistant", content))

    def log_token_usage(self, prompt_tokens: Any, completion_tokens: Any, cost: float) -> None:
        self.usage = (prompt_tokens, completion_tokens, cost)

    def log_response_time(self, seconds: float) -> None:
        self.response_time = seconds

    def delete_last_message(self) -> None:
        self.messages.pop()


def _rows() -> list[dict[str, Any]]:
    return [
        {
            "question": f"Where is marker {ordinal}?",
            "answer": ordinal % DEFAULT_GROUP_SIZE + 1,
            "task": "FAKE",
        }
        for ordinal in range(TASK.expected_rows)
    ]


def _prepared_run(run_dir: Path, monkeypatch) -> str:
    """Write the run a worker would find, and point the environment at it."""
    adapter = SpaceQAAdapter(TASK, _rows())
    items = adapter.iter_items(SubsetSpec(seed=SEED, groups=1))
    prepare_run(
        run_dir,
        adapter,
        items,
        task_name=TASK.name,
        seed=SEED,
        groups=1,
        data_sha256=None,
    )
    monkeypatch.setenv(RUN_DIR_ENV, str(run_dir))
    return items[0].question


def _ready_agent(agent_module, save_dir: Path):
    agent = agent_module.DimosQAAgent(model_name=MODEL_NAME, save_dir=str(save_dir))
    # What upstream's `reset` leaves behind, minus the markdown transcript.
    agent.dialog = _Dialog()
    agent.completion_tokens = 0
    agent.prompt_tokens = 0
    return agent


def _result(case_id: str, final_text: str) -> CompactEvalResult:
    return CompactEvalResult(
        case_id=case_id,
        model="gpt-5.6-luna",
        thinking_level="medium",
        final_response=final_text,
        prediction_status="not_evaluated",
        validator_revision=SPACE_REVISION,
        tool_call_count=4,
        duration_seconds=1.25,
    )


def _record(save_dir: Path) -> dict[str, Any]:
    return json.loads((save_dir / RECORD_NAME).read_text(encoding="utf-8"))


def test_a_question_is_answered_by_running_the_case_it_was_drawn_as(
    agent_module, tmp_path, monkeypatch
) -> None:
    run_dir = tmp_path / "run"
    save_dir = tmp_path / "space" / "qa_00000"
    question = _prepared_run(run_dir, monkeypatch)
    captured: dict[str, Any] = {}

    def execute(case_path, *, config, output):
        captured.update(case_path=case_path, config=config, output=output)
        return _result(json.loads(case_path.read_bytes())["case_id"], '{"answer": 3}')

    monkeypatch.setattr(agent_module, "execute_single_case", execute)

    prediction = _ready_agent(agent_module, save_dir).get_prediction(question, 3)

    assert prediction == 3
    assert captured["case_path"] == case_dir(run_dir, 0) / "case.json"
    assert captured["output"] == save_dir / "dimos"
    record = _record(save_dir)
    assert (record["pred"], record["space_parse_status"], record["infra_error"]) == (
        3,
        "parsed",
        None,
    )
    assert record["tool_call_count"] == 4


def test_a_question_that_could_not_be_run_is_recorded_rather_than_raised(
    agent_module, tmp_path, monkeypatch
) -> None:
    """A worker that dies takes its pool slot and the rest of the round with it."""
    run_dir = tmp_path / "run"
    save_dir = tmp_path / "space" / "qa_00000"
    question = _prepared_run(run_dir, monkeypatch)

    def refuse(*_args: Any, **_kwargs: Any) -> Any:
        raise RuntimeError("the Pi CLI is not built")

    monkeypatch.setattr(agent_module, "execute_single_case", refuse)

    prediction = _ready_agent(agent_module, save_dir).get_prediction(question, 3)

    assert prediction is None
    record = _record(save_dir)
    assert record["infra_error"] == "RuntimeError: the Pi CLI is not built"
    assert "the Pi CLI is not built" in record["infra_traceback"]
    assert record["pred"] is None


def test_a_question_the_run_directory_does_not_hold_is_refused_before_the_agent_runs(
    agent_module, tmp_path, monkeypatch
) -> None:
    """The subset SPACE is reading and the cases this run wrote have to be one subset."""
    run_dir = tmp_path / "run"
    save_dir = tmp_path / "space" / "qa_00000"
    _prepared_run(run_dir, monkeypatch)
    monkeypatch.setattr(
        agent_module,
        "execute_single_case",
        lambda *a, **k: pytest.fail("a mismatched question must never reach the agent"),
    )

    prediction = _ready_agent(agent_module, save_dir).get_prediction("Where is marker 999?", 3)

    assert prediction is None
    assert "was built for a different question" in _record(save_dir)["infra_error"]


def test_a_record_that_cannot_be_written_does_not_take_the_worker_down(
    agent_module, tmp_path, monkeypatch
) -> None:
    """`_write_record` runs in a finally: raising there would end the whole round."""
    run_dir = tmp_path / "run"
    save_dir = tmp_path / "space" / "qa_00000"
    question = _prepared_run(run_dir, monkeypatch)
    monkeypatch.setattr(
        agent_module,
        "execute_single_case",
        lambda case_path, **_kwargs: _result(
            json.loads(case_path.read_bytes())["case_id"], '{"answer": 3}'
        ),
    )
    save_dir.mkdir(parents=True)
    save_dir.chmod(0o500)

    try:
        prediction = _ready_agent(agent_module, save_dir).get_prediction(question, 3)
    finally:
        save_dir.chmod(0o700)

    assert prediction == 3
    assert not (save_dir / RECORD_NAME).exists()
